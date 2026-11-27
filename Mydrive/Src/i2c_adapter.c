#include "FreeRTOS.h"
#include "task.h"
#include "i2c_adapter.h"
#include "main.h"

/* ==========================================================================
 * 完成通知
 * ========================================================================== */

/**
 * @brief  粗粒度微秒级延时（仅用于总线恢复位操作，~84 cycles/µs @ 84 MHz）
 */
static void i2c_delay_us(uint32_t us)
{
    for (uint32_t n = us * 42u; n != 0u; --n) {
        __NOP();
    }
}

/** 本次传输的结果，由中断写入、由 BusTask 读取 */
static volatile I2cResult s_xfer_result = I2C_BUS_FAULT;
/** 等待中的任务句柄（BusTask），由 I2C_BusRead/Write 在进入时捕获 */
static TaskHandle_t s_owner = NULL;

/**
 * @brief  把 HAL 的 ErrorCode 映射成本模块的结果码
 */
static I2cResult MapError(uint32_t err)
{
    if ((err & HAL_I2C_ERROR_AF) != 0u) {
        return I2C_NACK;          /* 从设备无应答 */
    }
    if ((err & HAL_I2C_ERROR_TIMEOUT) != 0u) {
        return I2C_TIMEOUT;
    }
    return I2C_BUS_FAULT;         /* BERR / ARLO / OVR 等 */
}

/** 传输完成：只置标志并唤醒等待者 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) {
        return;                    /* I2C2(OLED) 的完成与本模块无关 */
    }
    BaseType_t woken = pdFALSE;
    s_xfer_result = I2C_OK;
    if (s_owner != NULL) {
        vTaskNotifyGiveFromISR(s_owner, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) {
        return;
    }
    BaseType_t woken = pdFALSE;
    s_xfer_result = I2C_OK;
    if (s_owner != NULL) {
        vTaskNotifyGiveFromISR(s_owner, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) {
        return;
    }
    BaseType_t woken = pdFALSE;
    s_xfer_result = MapError(hi2c->ErrorCode);
    if (s_owner != NULL) {
        vTaskNotifyGiveFromISR(s_owner, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

/* ==========================================================================
 * 审计
 * --------------------------------------------------------------------------
 * 目的：按"从设备地址"分别跟踪 I2C 事务的成败历史，判断整条总线是否
 * 处于降级状态（I2C_AuditDegraded），供安全层/上层做决策（如拒飞、
 * 丢弃可疑传感器数据）。
 *
 * 设计要点：
 *  - 每个设备一个槽位，互不影响：某一个外设坏了不会把另一个正常外设
 *    也拖成降级，但任一个降级都会让整条总线被判定为"不健康"。
 *  - 进入/解除降级采用不对称阈值（迟滞，hysteresis）：
 *      连续 3 次失败  -> 立即置降级（宁可误报，快速反映故障）
 *      连续 20 次成功 -> 才解除降级（防止偶发成功让状态来回抖动）
 *    在 BusTask 约 1 kHz 的采样节奏下，分别对应约 3 ms / 20 ms。
 *  - 全部为静态存储、无 malloc、无锁：AuditRecord 只在 BusTask 上下文
 *    被调用（I2C_BusRead/Write 与 I2C_QueueDrain 都限定在 BusTask），
 *    不存在并发写入；I2C_AuditDegraded 可被其他任务调用，只读 bool，
 *    最坏读到的是上一拍的旧状态，下个周期自然修正，可接受。
 * ========================================================================== */

#define AUDIT_SLOTS   4u            /* 最多同时跟踪 4 个从设备；超出只影响记录，不影响事务本身 */
#define AUDIT_DEGRADE_AFTER   3u    /* 连续失败多少次置降级 */
#define AUDIT_RECOVER_AFTER   20u   /* 连续成功多少次解除降级 */

/**
 * @brief  单个从设备的健康状态
 * @note   fail_run/ok_run 是"连续"计数：任一方向的成功/失败都会把
 *         另一方向的计数清零，从而保证计数语义严格是"连续"而非累计。
 *         两者饱和在 0xFF（uint8_t 防溢出回绕）：阈值远小于 0xFF，
 *         计数超过阈值后具体数值没有意义，只需保证不回绕成小值。
 */
typedef struct {
    uint16_t dev8;      /**< 左移后的 8 位设备地址（槽位归属标识） */
    uint8_t  fail_run;  /**< 当前连续失败次数（饱和 0xFF） */
    uint8_t  ok_run;    /**< 当前连续成功次数（饱和 0xFF） */
    bool     degraded;  /**< true = 该设备处于降级状态 */
    bool     used;      /**< true = 槽位已被某设备占用 */
} AuditSlot;

static AuditSlot s_audit[AUDIT_SLOTS];

/**
 * @brief  查找（或按需创建）某设备的审计槽
 * @param  dev8   设备地址
 * @param  create true = 找不到时占用一个空闲槽；false = 仅查询
 * @retval 槽指针；NULL 表示未找到（create=false）或槽位已耗尽
 * @note   槽位仅 AUDIT_SLOTS 个且设备不会"毕业"释放槽位：这是有意的
 *         简化——I2C1 上的外设数量有限（IMU、气压计等），4 个足够。
 *         槽满时返回 NULL，调用方（AuditRecord）静默放弃记录，
 *         绝不崩溃、也绝不阻塞事务——审计是旁路观测，不是数据通路。
 */
static AuditSlot *AuditFind(uint16_t dev8, bool create)
{
    /* 先线性查找已登记的该设备 */
    for (uint32_t i = 0u; i < AUDIT_SLOTS; ++i) {
        if (s_audit[i].used && (s_audit[i].dev8 == dev8)) {
            return &s_audit[i];
        }
    }
    if (!create) {
        return NULL;
    }
    /* 首次见到的设备：占用第一个空闲槽并初始化为"健康"状态 */
    for (uint32_t i = 0u; i < AUDIT_SLOTS; ++i) {
        if (!s_audit[i].used) {
            s_audit[i].dev8     = dev8;
            s_audit[i].fail_run = 0u;
            s_audit[i].ok_run   = 0u;
            s_audit[i].degraded = false;
            s_audit[i].used     = true;
            return &s_audit[i];
        }
    }
    return NULL;   /* 槽位耗尽：不记录，不崩溃 */
}

/**
 * @brief  记录一次事务结果并更新降级状态（带迟滞的状态机）
 * @param  dev8  本次事务的目标设备地址
 * @param  r     事务结果；只有 I2C_OK 算成功，其余（BUSY/TIMEOUT/NACK/
 *               BUS_FAULT）一律计入失败——BUSY 也失败是刻意的：
 *               外设持续繁忙同样是总线不健康的表现
 * @note   只能在 BusTask 上下文调用（见本节头部"无锁"说明）。
 */
static void AuditRecord(uint16_t dev8, I2cResult r)
{
    AuditSlot *s = AuditFind(dev8, true);
    if (s == NULL) {
        return;   /* 槽位耗尽：静默跳过，不影响本次事务的返回 */
    }
    if (r == I2C_OK) {
        /* 成功：清零失败连击；成功连击饱和递增；
         * 已降级的设备需连续成功 AUDIT_RECOVER_AFTER 次才解除 */
        s->fail_run = 0u;
        if (s->ok_run < 0xFFu) {
            s->ok_run++;
        }
        if (s->degraded && (s->ok_run >= AUDIT_RECOVER_AFTER)) {
            s->degraded = false;
        }
    } else {
        /* 失败：清零成功连击；失败连击饱和递增；
         * 达到阈值即置降级（幂等：已降级时重复置位无副作用） */
        s->ok_run = 0u;
        if (s->fail_run < 0xFFu) {
            s->fail_run++;
        }
        if (s->fail_run >= AUDIT_DEGRADE_AFTER) {
            s->degraded = true;
        }
    }
}

/**
 * @brief  查询整条总线是否降级
 * @retval true  至少一个被跟踪的从设备处于降级状态（短路返回）
 * @retval false 全部健康，或尚无任何设备被记录过
 * @note   语义是"总线级一票否决"：任一外设持续失败，上层就应把
 *         整条 I2C1 的数据（包括正常外设的）视为可疑。
 *         可从任意任务上下文调用（只读，无副作用）。
 */
/**
 * @brief  查询整条总线是否处于降级状态
 * @retval true 至少一个被跟踪的从设备处于持续失败状态
 */
bool I2C_AuditDegraded(void)
{
    for (uint32_t i = 0u; i < AUDIT_SLOTS; ++i) {
        if (s_audit[i].used && s_audit[i].degraded) {
            return true;
        }
    }
    return false;
}

/**
 * @brief  查询某个从设备是否处于降级状态
 * @param  dev8 左移后的 8 位设备地址
 * @retval true  该设备连续失败达到阈值，当前降级
 * @note   只读、无副作用，可在任意任务调用。AuditFind(create=false) 只扫描
 *         已登记槽位；used 由 BusTask 最后写入，因此跨任务读到 used==true 时
 *         dev8/degraded 已初始化，属于良性竞态（最坏读到上一拍旧值）。
 */
bool I2C_AuditDeviceDegraded(uint16_t dev8)
{
    const AuditSlot *s = AuditFind(dev8, false);
    return (s != NULL) && s->degraded;
}

/* ==========================================================================
 * 读写
 * ========================================================================== */

/**
 * @brief  强制复位 I²C1 总线与外设（只能在 BusTask 中调用）
 * @note   处理两类卡死：
 *         1) 从机把 SDA 拉死（传输中途死机/掉电），外设 SR2.BUSY 恒置位，
 *            任何 _IT 启动都立即返回 HAL_BUSY——光复位 HAL 状态机没用，
 *            必须用 GPIO 位操作补 9 个 SCL 时钟 + STOP 强迫从机释放总线；
 *         2) HAL 状态机残留 BUSY——DeInit 无条件把 State 清为 RESET，
 *            随后 Init 重新执行 MspInit（GPIO AF + 时钟 + NVIC）。
 */
void I2C_BusRecover(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_I2C_DISABLE(&hi2c1);

    /* PB8/PB9 切开漏输出，手动位操作 */
    io.Pin   = IMU_SCL_Pin | IMU_SDA_Pin;
    io.Mode  = GPIO_MODE_OUTPUT_OD;
    io.Pull  = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &io);

    HAL_GPIO_WritePin(GPIOB, IMU_SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, IMU_SDA_Pin, GPIO_PIN_SET);

    /* 9 个 SCL 时钟：卡在字节中间的从机在第 9 个沿释放 SDA */
    for (uint8_t i = 0u; i < 9u; ++i) {
        HAL_GPIO_WritePin(GPIOB, IMU_SCL_Pin, GPIO_PIN_RESET);
        i2c_delay_us(2u);
        HAL_GPIO_WritePin(GPIOB, IMU_SCL_Pin, GPIO_PIN_SET);
        i2c_delay_us(2u);
    }

    /* STOP：SCL 高期间 SDA 低→高，终结任何残留传输 */
    HAL_GPIO_WritePin(GPIOB, IMU_SDA_Pin, GPIO_PIN_RESET);
    i2c_delay_us(2u);
    HAL_GPIO_WritePin(GPIOB, IMU_SCL_Pin, GPIO_PIN_SET);
    i2c_delay_us(2u);
    HAL_GPIO_WritePin(GPIOB, IMU_SDA_Pin, GPIO_PIN_SET);
    i2c_delay_us(2u);

    (void)HAL_I2C_DeInit(&hi2c1);
    (void)HAL_I2C_Init(&hi2c1);
}

/**
 * @brief  等待本次 IT 传输完成（任务通知实现的"一次性信号量"）
 * @param  timeout_ms 超时；0 会被钳制成 1 tick，保证必然有界返回
 * @retval 回调写入的实际结果，或 I2C_TIMEOUT（回调始终没来）
 */
static I2cResult WaitDone(uint32_t timeout_ms)
{
    /* 清掉可能残留的通知，避免把上一次的完成当成这一次的。
     * 场景：上一次事务超时返回后，它的完成中断才"迟到"，通知已经挂在
     * 本任务的通知值上；不清掉，本次会把旧完成误认成本次的，读到错误的
     * s_xfer_result。clearOnExit=pdTRUE 使取走即清零，通知值不会跨事务累积 */
    (void)ulTaskNotifyTake(pdTRUE, 0u);

    /* pdMS_TO_TICKS(0)==0，而 FreeRTOS 的超时 0 表示"无限等待"：
     * 总线卡死时 BusTask 会永久挂起，这是最危险的失败模式，故钳制成 1 tick */
    const uint32_t ticks = pdMS_TO_TICKS(timeout_ms);
    if (ulTaskNotifyTake(pdTRUE, (ticks == 0u) ? 1u : ticks) == 0u) {
        return I2C_TIMEOUT;   /* 返回 0 = 超时未获通知 */
    }
    /* 回调"先写 s_xfer_result、后发通知"，该顺序保证任务被唤醒时读到的
     * 必然是本次传输的结果；volatile 与唤醒路径上的调度器临界区共同保证可见性 */
    return s_xfer_result;
}

/**
 * @brief  读寄存器：启动 IT 传输并阻塞等待完成
 * @note   只能在 BusTask 中调用；返回时 buf 内容已完整有效
 *         （事务失败时 buf 不可信）
 */
/**
 * @brief  读寄存器（阻塞语义，只能在 BusTask 中调用）
 * @param  dev8       左移后的 8 位设备地址（例如 MPU6050 = 0xD0）
 * @param  reg        寄存器地址
 * @param  buf        输出缓冲
 * @param  len        读取字节数
 * @param  timeout_ms 等待完成的超时（建议 5 ms）
 * @retval I2cResult
 * @note   底层使用 HAL_I2C_Mem_Read_IT；本函数返回时 buf 内容已完整。
 *         超时会自动执行一次总线恢复。
 */
I2cResult I2C_BusRead(uint16_t dev8, uint8_t reg, uint8_t *buf,
                      uint16_t len, uint32_t timeout_ms)
{
    if ((buf == NULL) || (len == 0u)) {
        return I2C_BUS_FAULT;
    }

    /* 登记属主：中断回调据此唤醒正确的任务。
     * 契约是同一时刻只有 BusTask 调用本模块，s_owner 才无需加锁 */
    s_owner = xTaskGetCurrentTaskHandle();
    /* 预置失败默认值：即使出现"通知到了但结果没写对"的极端竞态，
     * 也只会误报失败，不会把上一次传输的成功结果当真 */
    s_xfer_result = I2C_BUS_FAULT;

    /* _IT 接口只负责"启动"：外设状态机非空闲（上一笔还没结束）时立即
     * 返回非 HAL_OK。此时传输根本没开始、不会有回调，
     * 所以直接报 BUSY 返回，绝不能进 WaitDone 干等 */
    if (HAL_I2C_Mem_Read_IT(&hi2c1, dev8, (uint16_t)reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len) != HAL_OK) {
        /* 启动失败 = 总线被拉死（SR2.BUSY 恒置位），不会有回调。
         * 立即执行一次位操作解锁 + 重初始化，让下一次重试能真正启动，
         * 否则句柄永远卡在 BUSY，重试循环只会无限返回 r=1 */
        I2C_BusRecover();
        I2cResult r = I2C_BUSY;
        AuditRecord(dev8, r);
        return r;
    }

    I2cResult r = WaitDone(timeout_ms);
    if (r == I2C_TIMEOUT) {
        /* 超时说明外设/HAL 状态机很可能卡在 BUSY（如 SCL 被从机拉死），
         * 此后所有 _IT 调用都会永远返回非 HAL_OK，必须复位外设才能
         * 恢复后续事务；超时后迟到的完成中断只会多挂一个通知，
         * 由下一次 WaitDone 入口的空取清掉 */
        I2C_BusRecover();
    }
    AuditRecord(dev8, r);
    return r;
}

/**
 * @brief  写寄存器：与 I2C_BusRead 同一套"启动/等待/超时复位/审计"协议
 */
/**
 * @brief  写寄存器（阻塞语义，只能在 BusTask 中调用）
 * @param  dev8       左移后的 8 位设备地址
 * @param  reg        寄存器地址
 * @param  val        待写数据
 * @param  len        写入字节数
 * @param  timeout_ms 等待完成的超时
 * @retval I2cResult
 */
I2cResult I2C_BusWrite(uint16_t dev8, uint8_t reg, const uint8_t *val,
                       uint16_t len, uint32_t timeout_ms)
{
    if ((val == NULL) || (len == 0u)) {
        return I2C_BUS_FAULT;
    }

    s_owner = xTaskGetCurrentTaskHandle();   /* 登记属主，同 I2C_BusRead */
    s_xfer_result = I2C_BUS_FAULT;           /* 预置失败默认值，同上 */

    /* HAL 的 IT 写接口签名要求非 const 缓冲，(uint8_t *)val 只是去掉
     * const——写方向 HAL 只读取该缓冲，不会修改它。但调用方必须保证
     * val 指向的数据存活到本函数返回：IT 传输期间中断服务程序仍在
     * 逐字节读取它，传栈上临时数组后提前退出是典型错误用法 */
    if (HAL_I2C_Mem_Write_IT(&hi2c1, dev8, (uint16_t)reg,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)val, len) != HAL_OK) {
        /* 启动失败=外设/总线卡死，同 I2C_BusRead：解锁后再报 BUSY */
        I2C_BusRecover();
        I2cResult r = I2C_BUSY;   /* 不会有回调，直接返回 */
        AuditRecord(dev8, r);
        return r;
    }

    I2cResult r = WaitDone(timeout_ms);
    if (r == I2C_TIMEOUT) {
        I2C_BusRecover();   /* 复位卡死的状态机，同 I2C_BusRead */
    }
    AuditRecord(dev8, r);
    return r;
}

/* ==========================================================================
 * 慢路径队列
 * --------------------------------------------------------------------------
 * 为什么存在：I2C1 总线只允许 BusTask 触碰（s_owner/审计都按单写者设计），
 * 但"改一个配置寄存器"这类请求可能来自任意任务（ControlTask 改量程、
 * SafetyTask 解锁外设等）。本队列把这类请求从生产者任务解耦出来：
 * 生产者非阻塞入队，BusTask 每 1 ms 排空并串行执行。
 * 只支持"单字节写"：配置寄存器均为 8 位，定长小请求让队列实现最简。
 * ========================================================================== */

#define I2C_QDEPTH 8u               /* 深度 8：突发配置写入也够用，满则拒绝 */

typedef struct {
    uint16_t dev8;    /* 目标设备地址 */
    uint8_t  reg;     /* 寄存器地址 */
    uint8_t  val;     /* 写入值（定长 1 字节，故请求可整体拷贝） */
} I2cReq;

static I2cReq   s_queue[I2C_QDEPTH];
static uint8_t  s_q_head = 0u;   /* 下一个可写位置（生产者） */
static uint8_t  s_q_tail = 0u;   /* 下一个可读位置（消费者） */
static uint8_t  s_q_cnt  = 0u;   /* 当前元素个数：区分"满"和"空"，
                                  * 仅靠 head==tail 无法分辨这两种状态 */

/**
 * @brief  提交一个单字节写请求到慢路径队列（生产者接口，任意任务上下文可调用，非阻塞）
 *
 * @param  dev8  设备 7 位地址（不含读写位）
 * @param  reg   目标寄存器地址
 * @param  val   要写入的数据
 *
 * @retval I2C_OK   入队成功，请求已挂在队列中等待 BusTask 执行
 * @retval I2C_BUSY 队列已满，本次请求未入队，由调用方自行决定重试或放弃
 *
 * @note
 *   1. 本函数是慢路径生产者：只负责把写请求放入环形缓冲，立即返回，绝不阻塞。
 *      即使调用方优先级低于 BusTask，也不会发生反转等待——满时直接返回 BUSY。
 *
 *   2. 慢路径用于低频、非实时的寄存器配置（量程、滤波、校准等），
 *      与 1ms 周期 IMU 数据读取的快路径互不干扰。
 *
 *   3. 多任务并发调用时，用 taskENTER_CRITICAL() 保护队列索引与计数，
 *      临界区只做内存拷贝，无任何等待，开销可忽略。
 *
 *   4. 请求的实际执行由 BusTask 每 1 ms 调用 I2C_QueueDrain() 完成，
 *      执行顺序为 FIFO，保证寄存器写操作的时序一致性。
 */
I2cResult I2C_QueueSubmit(uint16_t dev8, uint8_t reg, uint8_t val)
{
    I2cResult r = I2C_OK;

    /* 多任务并发提交：用调度器临界区保护环形缓冲的索引与计数。
     * 临界区内只做几条拷贝，不含任何等待，代价可忽略 */
    taskENTER_CRITICAL();
    if (s_q_cnt >= I2C_QDEPTH) {
        r = I2C_BUSY;
    } else {
        s_queue[s_q_head].dev8 = dev8;
        s_queue[s_q_head].reg  = reg;
        s_queue[s_q_head].val  = val;
        s_q_head = (uint8_t)((s_q_head + 1u) % I2C_QDEPTH);
        s_q_cnt++;
    }
    taskEXIT_CRITICAL();

    return r;
}

/**
 * @brief  消费者：执行慢路径队列中的全部请求（只能在 BusTask 任务上下文中调用）
 *
 * @note
 *   1. 本函数由 BusTask 每 1 ms 调用一次，按 FIFO 顺序串行执行队列中
 *      挂起的全部 I2C 写请求，是慢路径的消费端。
 *
 *   2. 出队与总线写入刻意分离：临界区只保护环形缓冲的索引与计数，
 *      将请求拷贝到栈上 req 后立即释放槽位；I2C_BusWrite() 可能阻塞
 *      最长 5 ms，绝不能关着中断或调度器执行它。
 *      释放后该槽位即可被生产者 I2C_QueueSubmit() 复用，实现生产者
 *      与消费者的解耦并发。
 *
 *   3. 最坏耗时有界：队列满时 8 条请求 × 每条 5 ms 超时 = 40 ms，
 *      且只有总线持续故障时才会逼近该上界；正常情况下远小于此值。
 *
 *   4. 任一请求失败只记录审计，不阻塞后续请求继续执行，更不会阻塞
 *      IMU 数据读取的快路径（快路径由中断/DMA 触发，独立于本队列）。
 */
void I2C_QueueDrain(void)
{
    for (;;) {
        I2cReq req;

        /* 出队与总线写入刻意分离：临界区只保护索引，
         * I2C_BusWrite 可能阻塞 5 ms，绝不能关着中断/调度器执行它。
         * 先拷贝到栈上 req 再释放槽位，写入期间该槽即可被生产者复用 */
        taskENTER_CRITICAL();
        if (s_q_cnt == 0u) {
            taskEXIT_CRITICAL();
            return;
        }
        req = s_queue[s_q_tail];
        s_q_tail = (uint8_t)((s_q_tail + 1u) % I2C_QDEPTH);
        s_q_cnt--;
        taskEXIT_CRITICAL();

        /* 失败只记录审计（I2C_BusWrite 内部完成）：单个配置寄存器
         * 写不进去不应阻塞后续请求，更不应阻塞 IMU 采样主路径 */
        (void)I2C_BusWrite(req.dev8, req.reg, &req.val, 1u, 5u);
    }
}
