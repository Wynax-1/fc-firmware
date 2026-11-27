/**
 * @brief  匿名协议收发与姿态数据上报模块。
 * @note   本文件实现匿名飞控地面站协议的下行解析、上行数据打包发送，
 *         以及 PID 参数缓存与校准参数转发。
 */
#include "anonym.h"
#include "anon_fmt.h"
#include "atomic_publish.h"
#include "calib.h"
#include "altitude.h"
#include "fc_time.h"
#include "usart.h" 
#include "stm32f4xx_hal.h"
#include "safety.h"
#include <string.h>


/* ==========================================================================
 * 常量
 * ========================================================================== */
#define ANON_HEAD        0xAAu
#define ANON_ADDR_FC     0x05u
#define ANON_ADDR_HOST   0xAFu
#define ANON_ADDR_BCAST  0xFFu

#define ANON_MAX_DATA    40u
#define ANON_RX_BUF      (4u + ANON_MAX_DATA + 2u)   /* HEAD..DATA..SC..AC */
#define ANON_TX_BUF      (4u + ANON_MAX_DATA + 2u)

#define ANON_RAD2DEG     57.295779513f

/* 调试控制参数（ID 29~33，RAM 临时，不落盘） */
#define DBG_ID_MODE        29u   /* 0=DISABLED 1=ATT_HOLD 2=RATE */
#define DBG_ID_THROTTLE    30u   /* 测试油门 µs，1000~2000 */
#define DBG_ID_TGT_ROLL    31u   /* 目标 Roll 0.01°，±4000 即 ±40°（与 ATT_TILT_HARD_DEG 一致） */
#define DBG_ID_TGT_PITCH   32u   /* 目标 Pitch 0.01°，±4000 即 ±40°（与 ATT_TILT_HARD_DEG 一致） */
#define DBG_ID_TGT_YAW_R   33u   /* 目标 Yaw Rate 0.01°/s */

static volatile int32_t s_dbg_mode       = 0;
static volatile int32_t s_dbg_throttle   = 1000;
static volatile int32_t s_dbg_tgt_roll   = 0;
static volatile int32_t s_dbg_tgt_pitch  = 0;
static volatile int32_t s_dbg_tgt_yaw_r  = 0;


static volatile int16_t s_tw_sr = 0;      /* 0.01° */
static volatile int16_t s_tw_sp = 0;
static volatile int16_t s_tw_sy = 0;      /* 0.01°/s */
static volatile uint8_t s_tw_status = 0u;
static volatile uint8_t s_tw_mode   = 0u;

/* ==========================================================================
 * PID 参数缓存（ID 2000~2020，内部域 0~100，协议值 = 增益 × 1e5）
 * 与 AutoPID 上位机一致：PAR_ID = 2000 + 组号×3 + 增益序号(0=P,1=I,2=D)。
 * 组 0~5 为姿态/角速度环，组 6（2018~2020）为高度环。
 * 地面阶段只做存储与读取，供控制任务消费。
 * ========================================================================== */
#define ANON_PID_FIRST   2000u
#define ANON_PID_LAST    2020u
#define ANON_PID_COUNT   (ANON_PID_LAST - ANON_PID_FIRST + 1u)

static float s_pid_gain[ANON_PID_COUNT];   /* 内部域 0~100 */
static volatile uint32_t s_tx_drop_cnt = 0u;

/* ==========================================================================
 * 小端写入 / 读取、浮点定标
 * ========================================================================== */


/**
 * @brief  将偏航角归一化到 [-180°, 180°] 区间
 * @param  yaw_deg : 输入偏航角，单位度
 * @retval 归一化后的偏航角，单位度
 */
static float NormYawDeg(float yaw_deg)
{
	while (yaw_deg > 180.0f)  { yaw_deg -= 360.0f; }
	while (yaw_deg < -180.0f) { yaw_deg += 360.0f; }
	return yaw_deg;
}

/* ==========================================================================
 * 参数读写（PID 2000~2020 + 校准 1000~1020）
 * ========================================================================== */

/**
 * @brief  读取参数值
 * @param  id  : 参数 ID（2000~2020 为 PID，1000~1020 为校准参数）
 * @param  val : 输出参数值指针，协议域（增益×1e5）
 * @retval true  读取成功
 * @retval false 参数不存在或 val 为空
 * @note   PID 2000~2020 与校准 1000~1020 均有效；
 *         其他 ID 或 val 为空时返回 false，上层回 0x80000000。
 */
static bool ParamGet(uint16_t id, int32_t *val)
{
	if (val == NULL) { return false; }

	if ((id >= ANON_PID_FIRST) && (id <= ANON_PID_LAST)) {
		*val = F2I32(s_pid_gain[id - ANON_PID_FIRST] * 100000.0f);
		return true;
	}
	if ((id >= CAL_ID_FIRST) && (id <= CAL_ID_LAST)) {   /* 换 enum */
		return Calib_GetParam(id, val);
	}
	if (id == DBG_ID_MODE)      { *val = s_dbg_mode;       return true; }
	if (id == DBG_ID_THROTTLE)  { *val = s_dbg_throttle;   return true; }
	if (id == DBG_ID_TGT_ROLL)  { *val = s_dbg_tgt_roll;   return true; }
	if (id == DBG_ID_TGT_PITCH) { *val = s_dbg_tgt_pitch;  return true; }
	if (id == DBG_ID_TGT_YAW_R) { *val = s_dbg_tgt_yaw_r;  return true; }
	return false;
}


/**
 * @brief  写入参数值
 * @param  id  : 参数 ID（2000~2020 为 PID，1000~1020 为校准参数）
 * @param  val : 待写入值（协议域，PID 需限制在 0~10000000）
 * @retval true  写入成功
 * @retval false 参数不存在
 */
static bool ParamSet(uint16_t id, int32_t val)
{
	if ((id >= ANON_PID_FIRST) && (id <= ANON_PID_LAST)) {
		if (val < 0)         { val = 0; }
		if (val > 10000000)  { val = 10000000; }
		s_pid_gain[id - ANON_PID_FIRST] = (float)val / 100000.0f;
		return true;
	}
	if ((id >= CAL_ID_FIRST) && (id <= CAL_ID_LAST)) {   /* 换 enum */
		return Calib_SetParam(id, val);
	}
	if (id == DBG_ID_MODE)      { s_dbg_mode      = (val < 0) ? 0 : (val > 2 ? 2 : val);   return true; }
	if (id == DBG_ID_THROTTLE)  { s_dbg_throttle  = (val < 1000) ? 1000 : (val > 2000 ? 2000 : val); return true; }
	if (id == DBG_ID_TGT_ROLL)  { s_dbg_tgt_roll  = (val < -4000) ? -4000 : (val > 4000 ? 4000 : val); return true; }
	if (id == DBG_ID_TGT_PITCH) { s_dbg_tgt_pitch = (val < -4000) ? -4000 : (val > 4000 ? 4000 : val); return true; }
	if (id == DBG_ID_TGT_YAW_R) { s_dbg_tgt_yaw_r = (val < -2000) ? -2000 : (val > 2000 ? 2000 : val); return true; }
	return false;
}

/**
 * @brief  发送 0x00 数据校验帧（匿名协议安全通信要求）
 * @param  id_get : 被确认帧的功能码 ID
 * @param  sc_get : 被确认帧的和校验字节
 * @param  ac_get : 被确认帧的附加校验字节
 * @retval 无
 * @note   协议规定：参数读写类、命令控制类等非显示类帧均需回校验帧，
 *         上位机收不到会判定通信失败并重试，表现为“等待校验数据超时”。
 */
static void AnonSendCheck(uint8_t id_get, uint8_t sc_get, uint8_t ac_get)
{
    uint8_t d[3];
    d[0] = id_get;
    d[1] = sc_get;
    d[2] = ac_get;
    AnonSend(ANON_ADDR_BCAST, 0x00u, d, 3u);
}

/**
 * @brief  处理 0xE1 参数读取请求，回复 0xE2 参数值
 * @param  par_id : 请求读取的参数 ID
 * @param  sc     : 被确认帧的 SC 校验字节
 * @param  ac     : 被确认帧的 AC 校验字节
 * @retval 无
 */
static void AnonSendReadAck(uint16_t par_id, uint8_t sc, uint8_t ac)
{
    AnonSendCheck(0xE1u, sc, ac);   /* 先回校验帧，否则上位机等不到会重试 */

    int32_t val;
    if (!ParamGet(par_id, &val)) {
        val = (int32_t)0x80000000;   /* 本机未使用该参数 */
    }

    uint8_t d[6];
    PutU16(&d[0], par_id);
    PutI32(&d[2], val);
    AnonSend(ANON_ADDR_HOST, 0xE2u, d, 6u);   /* D_ADDR=0xAF */
}

/**
 * @brief  处理 0xE2 参数写入请求，无论成败均回广播 ACK 防上位机卡重试
 * @param  par_id : 待写入的参数 ID
 * @param  val    : 待写入的参数值（协议域）
 * @param  sc     : 被确认帧的 SC 校验字节
 * @param  ac     : 被确认帧的 AC 校验字节
 * @retval 无
 */
static void AnonHandleWrite(uint16_t par_id, int32_t val, uint8_t sc, uint8_t ac)
{
    (void)ParamSet(par_id, val);   /* 非法 ID / 越界值也回 ACK，防上位机卡重试 */

    AnonSendCheck(0xE2u, sc, ac);
}

/**
 * @brief  解析并分发下行协议帧
 * @param  f : 帧缓冲区（含 HEAD、D_ADDR、ID、LEN、DATA、SC、AC）
 * @param  n : 缓冲区长度
 * @retval 无
 * @note   所有帧均严格校验 SC/AC；0xE0/0xE1/0xE2 按安全通信要求回 0x00 校验帧。
 */
static void AnonHandleFrame(const uint8_t *f, uint8_t n)
{
	(void)n;
	const uint8_t daddr = f[1];
	const uint8_t id    = f[2];
	const uint8_t len   = f[3];

	/* 下行受理地址：点对点链路，除发往上位机自身的帧外全部受理。
	 * 匿名上位机参数页按"选中设备"填 D_ADDR（可能是 0x10/0x22/0x30/0x66
	 * 等任意地址），白名单过严会静默丢帧，表现为"等待校验数据超时"。 */
	if (daddr == ANON_ADDR_HOST) {
		return;
	}

	uint8_t sc = 0u, ac = 0u;
	const uint8_t total = (uint8_t)(4u + len);
	for (uint8_t i = 0u; i < total; ++i) {
		sc = (uint8_t)(sc + f[i]);
		ac = (uint8_t)(ac + sc);
	}
	if ((sc != f[4u + len]) || (ac != f[5u + len])) {
		return;
	}


	/* 参数读写类帧的 SC/AC：上位机靠它们比对校验帧，必须原帧回送 */
	const uint8_t sc_recv = f[4u + len];
	const uint8_t ac_recv = f[5u + len];

	switch (id) {
	case 0xE0u:
    /* 命令帧：CID = DATA[0]。已定义 CID：
     *   0x01  解锁请求   -> Safety_RequestArm(true)
     *   0x02  锁定请求   -> Safety_RequestArm(false)
     *   0x43  校准保存   -> 需 CMD0=0xA5 魔数，真正擦写 Flash
     *   0x44  校准原始值模式 -> CMD0=1 开 / 0 关（采样期旁路 Calib_Apply）
     */
    if (len >= 1u) {
			switch (f[4]) {
			case 0x01u:
				Safety_RequestArm(true);
				break;
			case 0x02u:
				Safety_RequestArm(false);
				break;
			case 0x43u:
				if ((len >= 2u) && (f[5] == 0xA5u)) {
						Calib_SaveToFlash();
				}
				break;
			case 0x44u:
				if (len >= 2u) {
					const bool raw = (f[5] != 0u);
					if (raw != Calib_IsRawMode()) {
						Calib_SetRawMode(raw);
						AnonLog(2u, raw ? "CAL RAW MODE ON" : "CAL RAW MODE OFF");
					}
				}
				break;
			default:
				break;
			}
    }
    AnonSendCheck(0xE0u, sc_recv, ac_recv);   /* 命令类帧仍需回校验 */
    break;

	case 0xE1u:
		if (len == 2u) {
			AnonSendReadAck(GetU16(&f[4]), sc_recv, ac_recv);
		}
		break;

	case 0xE2u:
		if (len == 6u) {
			AnonHandleWrite(GetU16(&f[4]), GetI32(&f[6]), sc_recv, ac_recv);
		}
		break;

	case 0x41u:
		/* RC 数据下行帧：飞控端只解析不发，忽略。 */
		break;

	default:
		/* 其余帧（显示类/未定义）不属安全通信范畴，不需回校验帧 */
		break;
	}
}

/* ==========================================================================
 * 接收状态机
 * ========================================================================== */
typedef enum {
    RX_HEAD = 0,
    RX_DADDR,
    RX_ID,
    RX_LEN,
    RX_DATA,
    RX_SC,
    RX_AC
} AnonRxState;

static AnonRxState s_rx_state = RX_HEAD;  /* 当前状态机位置（HEAD/DADDR/ID/LEN/DATA/SC/AC） */
static uint8_t    s_buf[ANON_RX_BUF];     /* 帧组装缓冲：HEAD + D_ADDR + ID + LEN + DATA + SC + AC */
static uint8_t    s_idx = 0u;             /* s_buf 当前写入位置 */
static uint8_t    s_len = 0u;             /* 当前帧 LEN 字段（数据区字节数） */
static uint8_t    s_data_idx = 0u;        /* 当前帧已收到的 DATA 字节数 */

/**
 * @brief  向匿名协议接收状态机喂入串口字节流
 * @param  data : 串口接收到的字节数组
 * @param  len  : 字节数组长度
 * @retval 无
 * @note   状态机逐字节解析，遇到非法长度自动回退重同步。
 */
void Anonym_RxPush(const uint8_t *data, uint16_t len)
{
    if (data == NULL) { return; }

    for (uint16_t i = 0u; i < len; ++i) {
        const uint8_t b = data[i];

        switch (s_rx_state) {
        case RX_HEAD:
            if (b == ANON_HEAD) {
                s_buf[0] = b;
                s_idx = 1u;
                s_rx_state = RX_DADDR;
            }
            break;

        case RX_DADDR:
            s_buf[s_idx++] = b;
            s_rx_state = RX_ID;
            break;

        case RX_ID:
            s_buf[s_idx++] = b;
            s_rx_state = RX_LEN;
            break;

        case RX_LEN:
            s_buf[s_idx++] = b;
            s_len = b;
            s_data_idx = 0u;
            if (s_len > ANON_MAX_DATA) {
                s_rx_state = RX_HEAD;     /* 非法长度，回退重同步 */
            } else if (s_len == 0u) {
                s_rx_state = RX_SC;
            } else {
                s_rx_state = RX_DATA;
            }
            break;

        case RX_DATA:
            s_buf[s_idx++] = b;
            if (++s_data_idx >= s_len) {
                s_rx_state = RX_SC;
            }
            break;

        case RX_SC:
            s_buf[s_idx++] = b;
            s_rx_state = RX_AC;
            break;

        case RX_AC:
            s_buf[s_idx++] = b;
            AnonHandleFrame(s_buf, s_idx);
            s_rx_state = RX_HEAD;
            break;

        default:
            s_rx_state = RX_HEAD;
            break;
        }
    }
}

/* ==========================================================================
 * 上行帧发送
 * ========================================================================== */

/**
 * @brief  发送一帧匿名协议上行数据
 * @param  daddr : 目标地址（0xAF 为主机，0xFF 为广播）
 * @param  id    : 帧 ID
 * @param  data  : 数据区指针
 * @param  len   : 数据长度，不得超过 ANON_MAX_DATA
 * @retval 无
 * @note   非阻塞：拼帧后交给 Uart1_TxEnqueue 排队，由 DMA 异步发出；
 *         TX 块池满时本帧直接丢弃并累加 s_tx_drop_cnt，不重试。
 */
void AnonSend(uint8_t daddr, uint8_t id, const uint8_t *data, uint8_t len)
{
    uint8_t buf[ANON_TX_BUF];
    if (len > ANON_MAX_DATA) { return; }

    buf[0] = ANON_HEAD;
    buf[1] = daddr;
    buf[2] = id;
    buf[3] = len;
    if ((len > 0u) && (data != NULL)) {
        (void)memcpy(&buf[4], data, len);
    }

    const uint8_t total = (uint8_t)(4u + len);
    uint8_t sc = 0u, ac = 0u;
    for (uint8_t i = 0u; i < total; ++i) {
        sc = (uint8_t)(sc + buf[i]);
        ac = (uint8_t)(ac + sc);
    }
    buf[total]      = sc;
    buf[total + 1u] = ac;

    /* 交给驱动层排队（非阻塞）；返回 false 表示 TX 池满，本帧丢弃。
     * 周期性上报帧丢一帧可接受；控制类帧调用方应关注丢帧计数。 */
		if (!Uart1_TxEnqueue(buf, (uint16_t)(total + 2u))) {
        s_tx_drop_cnt++;
    }
}

/**
 * @brief  发送 0x04 四元数帧
 * @param  att : 姿态快照指针，可为 NULL
 * @retval 无
 * @note   四元数放大 10000 倍后以 int16 发送；valid 标志放入 d[8]。
 */
static void AnonSendQuat(const AttitudeSnapshot *att)
{
    uint8_t d[9];
    if ((att != NULL) && att->valid) {
        PutI16(&d[0], F2I16(att->q0 * 10000.0f));
        PutI16(&d[2], F2I16(att->q1 * 10000.0f));
        PutI16(&d[4], F2I16(att->q2 * 10000.0f));
        PutI16(&d[6], F2I16(att->q3 * 10000.0f));
        d[8] = 1u;
    } else {
        (void)memset(d, 0, sizeof(d));
        d[8] = 0u;
    }
    AnonSend(ANON_ADDR_BCAST, 0x04u, d, 9u);
}

/**
 * @brief  发送 0xF1 调参波形帧
 * @param  att : 姿态快照指针，可为 NULL
 * @param  sen : 传感器快照指针，可为 NULL
 * @retval 无
 * @note   地面阶段无控制任务，SR/SP/SY、STEP、status、mode 全部填 0，
 *         仅保留姿态角、角速度观测与时间戳。
 */
static void AnonSendTuneWave(const AttitudeSnapshot *att,
                             const SensorData *sen)
{
    uint8_t d[26];

    PutI32(&d[0], (int32_t)(Fc_GetMicros() / 1000u));   /* ts: ms */

    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    if ((att != NULL) && att->valid) {
        roll  = att->roll_rad  * ANON_RAD2DEG;
        pitch = att->pitch_rad * ANON_RAD2DEG;
        yaw   = NormYawDeg(att->yaw_rad * ANON_RAD2DEG);
    }
    PutI16(&d[4],  F2I16(roll  * 100.0f));
    PutI16(&d[6],  F2I16(pitch * 100.0f));
    PutI16(&d[8],  F2I16(yaw   * 100.0f));

    if ((sen != NULL) && sen->valid) {
        PutI16(&d[10], F2I16(sen->gx_radps * ANON_RAD2DEG * 10.0f));
        PutI16(&d[12], F2I16(sen->gy_radps * ANON_RAD2DEG * 10.0f));
        PutI16(&d[14], F2I16(sen->gz_radps * ANON_RAD2DEG * 10.0f));
    } else {
        (void)memset(&d[10], 0, 6u);
    }

		/* SR/SP/SY（目标角 0.01°）+ STEP + status + mode */
    PutI16(&d[16], s_tw_sr);
    PutI16(&d[18], s_tw_sp);
    PutI16(&d[20], s_tw_sy);
    d[22] = 0u;                 /* STEP：本阶段恒 0，留给后续阶跃激励 */
    d[23] = s_tw_status;
    d[24] = s_tw_mode;
    d[25] = 0u;

    AnonSend(ANON_ADDR_BCAST, 0xF1u, d, 26u);
}

/**
 * @brief  发送 0x01 惯导帧（加速度 + 角速度）
 * @param  s : 传感器快照指针，可为 NULL
 * @retval 无
 * @note   V7 字段顺序：ACC_X/Y/Z、GYR_X/Y/Z（int16），SHOCK_STA 在最后。
 *         加速度单位为 g×1000，角速度单位为 °/s×100，SHOCK_STA 恒 0。
 */
static void AnonSendIns(const SensorData *s)
{
    uint8_t d[13];

    if ((s != NULL) && s->valid) {
        PutI16(&d[0],  F2I16(s->ax_g * 1000.0f));
        PutI16(&d[2],  F2I16(s->ay_g * 1000.0f));
        PutI16(&d[4],  F2I16(s->az_g * 1000.0f));
        PutI16(&d[6],  F2I16(s->gx_radps * ANON_RAD2DEG * 100.0f));
        PutI16(&d[8],  F2I16(s->gy_radps * ANON_RAD2DEG * 100.0f));
        PutI16(&d[10], F2I16(s->gz_radps * ANON_RAD2DEG * 100.0f));
    } else {
        (void)memset(d, 0, 12u);
    }
    d[12] = 0u;   /* SHOCK_STA 恒 0（V7：位于 DATA 末尾） */
    AnonSend(ANON_ADDR_BCAST, 0x01u, d, 13u);
}

/**
 * @brief  发送 0x02 罗盘 / 气压 / 温度帧
 * @param  s : 传感器快照指针，可为 NULL
 * @retval 无
 * @note   V7 字段顺序：MAG_X/Y/Z（int16）、ALT_BAR（int32）、TMP（int16）、
 *         BAR_STA、MAG_STA。地面校准阶段高度/气压暂缓，ALT_BAR 与
 *         BAR_STA 固定填 0；磁力计原始 LSB 保持发送，作为校准拟合关键输入。
 */
static void AnonSendBaroMag(const SensorData *s)
{
    uint8_t d[14];

    if ((s != NULL) && s->mag_ok) {
        PutI16(&d[0], F2I16(s->mx));
        PutI16(&d[2], F2I16(s->my));
        PutI16(&d[4], F2I16(s->mz));
    } else {
        (void)memset(&d[0], 0, 6u);
    }

    /* 从 altitude 快照读高度 */
    AltitudeSnapshot alt;
    if (FcChan_Read(&g_chan_altitude, &alt) && alt.valid) {
        PutI32(&d[6], (int32_t)(alt.height_m * 100.0f));   /* cm */
        d[12] = (alt.source == ALT_SRC_US) ? 2u : 1u;      /* 2=超声, 1=气压 */
    } else {
        PutI32(&d[6], 0);
        d[12] = 0u;
    }
    PutI16(&d[10], 0);                                     /* TMP：暂不发 */

    d[13] = ((s != NULL) && s->mag_ok) ? 1u : 0u;          /* MAG_STA */

    AnonSend(ANON_ADDR_BCAST, 0x02u, d, 14u);
}

/**
 * @brief  发送 0x03 飞控姿态欧拉角帧（V7 规定帧，7 字节）
 * @param  a : 姿态快照指针，可为 NULL
 * @retval 无
 * @note   V7 字段顺序：ROL/PIT/YAW（int16×100，度），FUSION_STA 在最后。
 *         注意：0x20/0x21 在 V7 中是 PWM/姿态控制量输出帧，不得挪用。
 */
static void AnonSendEuler(const AttitudeSnapshot *a)
{
    uint8_t d[7];
    if ((a != NULL) && a->valid) {
        PutI16(&d[0], F2I16(a->roll_rad  * ANON_RAD2DEG * 100.0f));
        PutI16(&d[2], F2I16(a->pitch_rad * ANON_RAD2DEG * 100.0f));
        PutI16(&d[4], F2I16(NormYawDeg(a->yaw_rad * ANON_RAD2DEG) * 100.0f));
        d[6] = 1u;   /* FUSION_STA：解算有效 */
    } else {
        (void)memset(d, 0, sizeof(d));
    }
    AnonSend(ANON_ADDR_BCAST, 0x03u, d, 7u);
}

/**
 * @brief  发送 0x05 高度数据帧
 * @retval 无
 * @note   V7 字段：ALT_FU（融合对地高度，cm）、ALT_ADD（超声测高，cm）、ALT_STA。
 *         ALT_STA：0=无高度数据，1=仅气压计，2=超声正常。
 */
static void AnonSendAlt(void)
{
    uint8_t d[9];
    AltitudeSnapshot alt;
    float   us_cm = 0.0f;
    int32_t fu_cm = 0;
    uint8_t sta   = 0u;

    if (FcChan_Read(&g_chan_altitude, &alt) && alt.valid) {
        fu_cm = (int32_t)(alt.height_m * 100.0f);
        sta   = (alt.source == ALT_SRC_US) ? 2u : 1u;
    }
    if (Altitude_GetUsHeightCm(&us_cm)) {
        sta = 2u;   /* 超声有数据即报"正常" */
    }

    PutI32(&d[0], fu_cm);
    PutI32(&d[4], (int32_t)us_cm);
    d[8] = sta;

    AnonSend(ANON_ADDR_BCAST, 0x05u, d, 9u);
}

/* ==========================================================================
 * 10 ms 调度
 * ========================================================================== */
static uint8_t s_tick = 0u;

/**
 * @brief  10 ms 周期任务：上报姿态/传感器数据
 * @param  无
 * @retval 无
 * @note   每拍发送 0x04 四元数和 0xF1 调参波形；
 *         10 Hz 轮转发送 0x03、0x05、0x01、0x02；
 *         遥控、模式、电机等地面阶段不发送。
 */
void Anonym_Step(void)
{
    AttitudeSnapshot att;
    SensorData       sen;

    const bool att_ok = FcChan_Read(&g_chan_attitude, &att);
    const bool sen_ok = FcChan_Read(&g_chan_sensor,   &sen);

    /* 每拍：0x04 四元数 + 0xF1 调参波形 */
    AnonSendQuat(att_ok ? &att : NULL);
    AnonSendTuneWave(att_ok ? &att : NULL,
                     sen_ok ? &sen : NULL);

    /* 10 Hz 轮转（V7 帧号），每 100 ms 一轮：
     * 空闲 → 0x03 → 0x05 → 空闲 → 0x01 → 空闲 → 空闲 → 0x02 → 空闲 → 空闲
     *   0x03 欧拉角、0x05 高度、0x01 惯导 ACC/GYR、0x02 磁力计原始 LSB
     * 原始数据不再用 0x21（V7 中为姿态控制量帧）重复上报；
     * 遥控 0x40、模式 0x06、电机 0x20 在地面阶段仍不发送。 */
    static const uint8_t rot_ids[10] = {
        0x00u, 0x03u, 0x05u, 0x00u, 0x01u,
        0x00u, 0x00u, 0x02u, 0x00u, 0x00u
    };

    switch (rot_ids[s_tick]) {
    case 0x03u: AnonSendEuler(att_ok ? &att : NULL); break;
    case 0x05u: AnonSendAlt(); break;
    case 0x01u: AnonSendIns(sen_ok ? &sen : NULL); break;
    case 0x02u: AnonSendBaroMag(sen_ok ? &sen : NULL); break;
    default:    break;   /* 空闲拍 */
    }

    if (++s_tick >= 10u) {
        s_tick = 0u;
    }
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

/**
 * @brief  匿名协议模块初始化，复位状态机、清空 PID 参数缓存
 * @param  无
 * @retval 无
 * @note   姿态/角速度 PID 默认全 0，需上位机经 0xE2 下发；
 *         高度环 PID（ID 2018~2020）填入台架默认值，上位机可直接读取。
 */
void Anonym_Init(void)
{
    s_rx_state = RX_HEAD;
    s_idx = 0u;
    s_len = 0u;
    s_data_idx = 0u;
    s_tick = 0u;

    /* PID 默认值：姿态/角速度环全 0，需上位机经 0xE2 下发。
     * 若希望上电即有可用初值，可在此填写，例如：
     *   s_pid_gain[0] = 5.0f;   // ID 2000 姿态环 Roll P
     */
    (void)memset(s_pid_gain, 0, sizeof(s_pid_gain));

    /* 高度环默认增益（ID 2018~2020）：Control_Task 高度环启动值，
     * 上位机「全部读取」可直接看到，改后 100ms 内生效。 */
    s_pid_gain[18] = 0.35f;   /* P：高度误差 → 油门修正 */
    s_pid_gain[19] = 0.06f;   /* I：静差 */
    s_pid_gain[20] = 0.18f;   /* D：垂直速度阻尼 */
}

void Anonym_TxInit(void)
{
    Uart1_TxInit();
}

void Anonym_TxPump(void)
{
    Uart1_TxPump();
}

uint32_t AnonGetTxDropCount(void)
{
    return s_tx_drop_cnt;
}

/**
 * @brief  发送 0xA0 LOG 文本帧（上位机日志窗口显示）
 * @param  color : 0 黑 / 1 红 / 2 绿
 * @param  str   : ASCII 字符串，最长 39 字节
 * @retval 无
 * @note   任务上下文调用；非阻塞入队，TX 池满则丢弃。
 */
void AnonLog(uint8_t color, const char *str)
{
    if ((str == NULL) || (str[0] == '\0')) { return; }

    uint8_t d[40];
    uint8_t n = 0u;
    while ((n < 38u) && (str[n] != '\0')) {
        n++;
    }

    d[0] = color;
    (void)memcpy(&d[1], str, n);
    AnonSend(ANON_ADDR_BCAST, 0xA0u, d, (uint8_t)(n + 1u));
}

float Anon_GetPidGain(uint16_t id)
{
    if ((id < ANON_PID_FIRST) || (id > ANON_PID_LAST)) {
        return 0.0f;
    }
    return s_pid_gain[id - ANON_PID_FIRST];
}

void Anon_GetDbgCtrl(AnonDbgCtrl *out)
{
    if (out == NULL) { return; }
    out->mode                  = s_dbg_mode;
    out->throttle_us           = s_dbg_throttle;
    out->tgt_roll_centi_deg    = s_dbg_tgt_roll;
    out->tgt_pitch_centi_deg   = s_dbg_tgt_pitch;
    out->tgt_yaw_rate_centi_dps= s_dbg_tgt_yaw_r;
}


void Anon_SetTuneTargets(int16_t tgt_roll_centi_deg,
                         int16_t tgt_pitch_centi_deg,
                         int16_t tgt_yaw_rate_centi_dps,
                         uint8_t status_bits,
                         uint8_t mode)
{
    s_tw_sr = tgt_roll_centi_deg;
    s_tw_sp = tgt_pitch_centi_deg;
    s_tw_sy = tgt_yaw_rate_centi_dps;
    s_tw_status = status_bits;
    s_tw_mode   = mode;
}
