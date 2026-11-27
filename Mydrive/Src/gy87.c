#include "gy87.h"
#include "calib.h"
#include "cmsis_os.h"

/* 换算常数（编译期确定，运行期零开销） */
#define ACC_LSB_PER_G      16384.0f        /* ±2 g  */
#define GYRO_LSB_PER_DPS   16.4f           /* ±2000 °/s */
#define DEG2RAD            0.017453292f

/* 磁力计读取抽取：1 kHz 下每 100 拍读一次 = 10 Hz */
#define MAG_DIVIDER        100u
/* 磁力计连续失败多少次判不健康（随后 Madgwick 自动退回 6 轴） */
#define MAG_FAIL_DEGRADE   3u

/** 磁力计最近一次读数（原始值） */
static float s_mx = 0.0f;
static float s_my = 0.0f;
static float s_mz = 0.0f;
/** 磁力计初始化是否成功（硬件在位） */
static bool  s_mag_ready = false;
/** 磁力计最近读取是否健康；连续失败达阈值后置 false */
static bool  s_mag_healthy = false;
/** 磁力计连续读取失败计数 */
static uint8_t s_mag_fail_run = 0u;
/** 采样计数，用于磁力计抽取 */
static uint32_t s_div = 0u;

/**
 * @brief  写一个 MPU/HMC 寄存器并检查结果
 */
static I2cResult RegWrite(uint16_t dev8, uint8_t reg, uint8_t val)
{
    return I2C_BusWrite(dev8, reg, &val, 1u, 5u);
}

/**
 * @brief  初始化 IMU
 * @retval I2C_OK          MPU6050 就绪（磁力计可能不可用）
 * @retval 其他             MPU6050 不可用，飞控不可起飞
 * @note   只能在 BusTask 中调用（内部有阻塞的 I²C 事务与 osDelay）。
 *         磁力计初始化失败不视为错误，只是 mag_ok 恒为 false。
 */
I2cResult GY87_Init(void)
{
    uint8_t id = 0u;
    I2cResult r;

    /* ---- 1. 确认设备在线 ---- */
    r = I2C_BusRead(GY87_MPU_ADDR, MPU_REG_WHO_AM_I, &id, 1u, 5u);
    if (r != I2C_OK) {
        return r;
    }
    if ((id != 0x68u) && (id != 0x71u)) {
        return I2C_BUS_FAULT;   /* 不是 MPU6050/MPU6500 系列 */
    }

    /* ---- 2. 复位并唤醒 ---- */
    (void)RegWrite(GY87_MPU_ADDR, MPU_REG_PWR_MGMT_1, 0x80u);  /* DEVICE_RESET */
    osDelay(50u);
    (void)RegWrite(GY87_MPU_ADDR, MPU_REG_PWR_MGMT_1, 0x00u);  /* 内部时钟、退出睡眠 */
    osDelay(10u);

    /* ---- 3. 采样率与滤波 ----
     * SMPLRT_DIV = 0            -> 1 kHz / (1+0) = 1 kHz
     * CONFIG     = 0x03         -> DLPF 使能，陀螺 42 Hz / 加速度 44 Hz，内部 1 kHz
     */
    if (RegWrite(GY87_MPU_ADDR, MPU_REG_SMPLRT_DIV, 0x00u) != I2C_OK) {
        return I2C_BUS_FAULT;
    }
    (void)RegWrite(GY87_MPU_ADDR, MPU_REG_CONFIG, 0x03u);

    /* ---- 4. 量程 ---- */
    (void)RegWrite(GY87_MPU_ADDR, MPU_REG_GYRO_CONFIG,  0x18u);  /* ±2000 °/s */
    (void)RegWrite(GY87_MPU_ADDR, MPU_REG_ACCEL_CONFIG, 0x00u);  /* ±2 g      */

    /* ---- 5. 旁路模式，让 I²C 总线能直达 HMC5883L ---- */
    (void)RegWrite(GY87_MPU_ADDR, MPU_REG_USER_CTRL,   0x00u);  /* 关闭内部 master */
    (void)RegWrite(GY87_MPU_ADDR, MPU_REG_INT_PIN_CFG, 0x02u);  /* BYPASS_EN       */

    /* ---- 6. 磁力计（可选） ---- */
    s_mag_ready    = false;
    s_mag_healthy  = false;
    s_mag_fail_run = 0u;
    if (RegWrite(GY87_HMC_ADDR, HMC_REG_CRA, 0x70u) == I2C_OK) {   /* 8 次平均，15 Hz */
        if (RegWrite(GY87_HMC_ADDR, HMC_REG_CRB, 0x20u) == I2C_OK) { /* 增益 1.3 Ga */
            if (RegWrite(GY87_HMC_ADDR, HMC_REG_MODE, 0x00u) == I2C_OK) { /* 连续测量 */
                s_mag_ready   = true;
                s_mag_healthy = true;
            }
        }
    }

    s_div = 0u;
    return I2C_OK;
}

/**
 * @brief  采样一次
 * @param  out : 输出快照（valid/mag_ok 由本函数填写，stamp 由调用者填写）
 * @param  now : 本次采样时刻，单位 µs
 * @retval I2C_OK 成功
 * @retval 其他   本次读取失败，out->valid 被置 false
 * @note   每 100 次调用读一次磁力计（1 kHz 下即 10 Hz）。
 *         必须在 BusTask 中调用。
 */
I2cResult GY87_Sample(SensorData *out, fc_usec_t now)
{
    uint8_t buf[14];
    I2cResult r;

    out->stamp.usec = now;
    out->valid      = false;
    out->mag_ok     = s_mag_ready && s_mag_healthy;

    /* ---- 1. 一次性读 acc(6) + temp(2) + gyro(6) = 14 字节 ---- */
    r = I2C_BusRead(GY87_MPU_ADDR, MPU_REG_ACCEL_XOUT_H, buf, 14u, 5u);
    if (r != I2C_OK) {
        return r;
    }

    /* 大端拼装：先转 uint16 再转 int16，避免直接左移符号位导致的符号扩展问题 */
    const int16_t ax = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    const int16_t ay = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
    const int16_t az = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);
    /* buf[6..7] 是温度，v1 不用 */
    const int16_t gx = (int16_t)(((uint16_t)buf[8]  << 8) | (uint16_t)buf[9]);
    const int16_t gy = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
    const int16_t gz = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);

    /* 模块坐标系 → 机体 FRD 系（X前 Y右 Z下）：
     * 模块安装为 X朝尾、Y朝右、Z朝上，即绕 Y 转 180°，b = (-x, y, -z)。
     * 平放时模块 Z(朝上) 读 +1g，换算后机体 Z(朝下) 读 -1g（NED 比力约定）。
     * 加计/陀螺/磁力计是同一旋转变换，必须在校准之前施加。 */
    out->ax_g     = -(float)ax / ACC_LSB_PER_G;
    out->ay_g     =  (float)ay / ACC_LSB_PER_G;
    out->az_g     = -(float)az / ACC_LSB_PER_G;

    out->gx_radps = -(float)gx / GYRO_LSB_PER_DPS * DEG2RAD;
    out->gy_radps =  (float)gy / GYRO_LSB_PER_DPS * DEG2RAD;
    out->gz_radps = -(float)gz / GYRO_LSB_PER_DPS * DEG2RAD;

    /* ---- 2. 磁力计抽取读取 ---- */
    s_div++;
    if (s_mag_ready && (s_div >= MAG_DIVIDER)) {
        uint8_t mbuf[6];
        s_div = 0u;
        if (I2C_BusRead(GY87_HMC_ADDR, HMC_REG_DATAX, mbuf, 6u, 5u) == I2C_OK) {
            /* HMC5883L 的数据寄存器顺序是 X, Z, Y；
             * 再按上面的 b = (-x, y, -z) 转到机体 FRD 系后缓存 */
            const int16_t mx = (int16_t)(((uint16_t)mbuf[0] << 8) | (uint16_t)mbuf[1]);
            const int16_t mz = (int16_t)(((uint16_t)mbuf[2] << 8) | (uint16_t)mbuf[3]);
            const int16_t my = (int16_t)(((uint16_t)mbuf[4] << 8) | (uint16_t)mbuf[5]);
            s_mx = -(float)mx;
            s_my =  (float)my;
            s_mz = -(float)mz;

            s_mag_fail_run = 0u;
            s_mag_healthy  = true;
        } else if (s_mag_fail_run < 0xFFu) {
            /* 连续失败达阈值：标记不健康，Madgwick 自动退回 6 轴；
             * 读取成功的下一拍即恢复（磁力计更新率本来就只有 10 Hz） */
            if (++s_mag_fail_run >= MAG_FAIL_DEGRADE) {
                s_mag_healthy = false;
            }
        }
    }

    out->mx     = s_mx;
    out->my     = s_my;
    out->mz     = s_mz;
    out->mag_ok = s_mag_ready && s_mag_healthy;
    out->valid  = true;


    Calib_GyroAutoCal(out);   /* 上电自动陀螺零偏校准（静止取平均，完成后为空操作） */
    Calib_Apply(out);   /* 施加 Flash 持久化的传感器校准（未校准项为恒等变换） */
    return I2C_OK;
}
