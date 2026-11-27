#include "calib.h"
#include "anon_fmt.h"
#include "stm32f4xx_hal.h"
#include "iwdg.h"
#include <string.h>
#include <stddef.h>
#include <math.h>

/* ==========================================================================
 * Flash 持久化：F401 512 KB。用 sector 7（128 KB @ 0x08060000）作参数页。
 * 擦除 ~1 s 阻塞 CommTask：只在组末帧（上位机一组参数下发完毕）触发，
 * 上位机 ACK 超时 5 s 覆盖。飞行路径上不存在任何落盘代码。
 * F401 单存储区：擦除挂起取指，所有任务（含喂狗）停摆，
 * CalibSave 内临时放宽 IWDG 重装载（4095 ≈ 4.1 s）后再落盘。
 * ========================================================================== */
#define CAL_FLASH_ADDR   0x08060000u
#define CAL_FLASH_SECTOR FLASH_SECTOR_7
#define CAL_MAGIC        0xCA11BA5Eu

/* 落盘期间临时放宽 IWDG：保持原分频，重装载拉到最大 4095。
 * LSI 32 kHz / 32 分频 × 4095 ≈ 4.1 s，覆盖最坏 ~2 s 的整扇区擦除。 */
#define CAL_IWDG_RELOAD_MAX  0xFFFu

#define CAL_FLAG_GYRO    (1u << 0)
#define CAL_FLAG_ACC     (1u << 1)
#define CAL_FLAG_MAG     (1u << 2)

/* ==========================================================================
 * PID 持久化记录：与校准共用 sector 7，放在 +1 KB 偏移处（0x08060400）。
 * 擦除以扇区为粒度 → ConfigSave 每次整页重写"校准 + PID"两条记录，
 * 写其一绝不会擦掉另一。两步式：0xE2 只改 RAM 缓存，
 * 0xE0/CID 0x45（CMD0=0xA5）才把 PID 缓存写入本记录并落盘。
 * ========================================================================== */
#define PID_FLASH_ADDR   (CAL_FLASH_ADDR + 0x400u)
#define PID_MAGIC        0x50494431u          /* "PID1" */
#define PID_BLOB_WORDS   (sizeof(PidBlob) / 4u)

typedef struct {
	uint32_t magic;
	uint32_t ver;
	float    pid[CAL_PID_COUNT];   /* ID 2000~2020，内部域 0~100 */
	uint32_t cks;
} PidBlob;

typedef char pid_blob_size_must_be_word_aligned[
    ((sizeof(PidBlob) % 4u) == 0u) ? 1 : -1];

/* 上次已落盘 PID 的 RAM 镜像；magic != PID_MAGIC 表示从未落盘，
 * 落盘时跳过该记录（保持擦除态 0xFF），上电加载即判无效 */
static PidBlob s_pid_saved;

typedef struct {
	uint32_t magic;
	uint32_t ver;
	float    gyro_bias[3];       /* 陀螺零偏 rad/s */
	float    acc_off[3];         /* 加速度零偏 g */
	float    acc_sens[3];        /* 加速度比例，默认 1 */
	float    mag_off[3];         /* 磁力计硬磁偏置 LSB */
	float    mag_si[9];   /* 磁力计软磁矩阵（对称） */
	uint32_t flags;
	uint32_t cks;
} CalibBlob;

#define CAL_BLOB_WORDS (sizeof(CalibBlob) / 4u)

typedef char calib_blob_size_must_be_word_aligned[
    ((sizeof(CalibBlob) % 4u) == 0u) ? 1 : -1];
typedef char calib_cks_must_be_last[
    ((offsetof(CalibBlob, cks) + 4u) == sizeof(CalibBlob)) ? 1 : -1];

static CalibBlob s_cal;

/* 校准原始值模式开关（CID 0x44）：开启时 Calib_Apply 空操作，
 * 0x01/0x02 上报原始数据供上位机采样；单核 M4 上 bool 读写天然原子 */
static volatile bool s_cal_raw_mode = false;

/**
 * @brief  计算校准数据块校验和
 * @param  b : 数据块指针
 * @retval 除末尾 cks 字段外所有 32 位字的累加和
 */
static uint32_t BlobCks(const CalibBlob *b)
{
    const uint32_t *p = (const uint32_t *)(const void *)b;
    const uint32_t n = offsetof(CalibBlob, cks) / 4u;   /* 不再依赖 -1 魔法 */
    uint32_t c = 0u;
    for (uint32_t i = 0u; i < n; ++i) c += p[i];
    return c;
}

/**
 * @brief  计算 PID 记录校验和（除末尾 cks 外所有 32 位字累加）
 */
static uint32_t PidBlobCks(const PidBlob *b)
{
    const uint32_t *p = (const uint32_t *)(const void *)b;
    const uint32_t n = offsetof(PidBlob, cks) / 4u;
    uint32_t c = 0u;
    for (uint32_t i = 0u; i < n; ++i) c += p[i];
    return c;
}

/**
 * @brief  恢复默认校准：零偏清零、比例与软磁矩阵为单位阵
 * @note   flags 清零后 Calib_Apply 等效于未校准。
 */
static void Defaults(void)
{
	memset(&s_cal, 0, sizeof(s_cal));
	s_cal.acc_sens[0] = 1.0f;
	s_cal.acc_sens[1] = 1.0f;
	s_cal.acc_sens[2] = 1.0f;

	/* 默认软磁矩阵 = 单位阵 */
    s_cal.mag_si[0]   = 1.0f;
    s_cal.mag_si[4]   = 1.0f;
    s_cal.mag_si[8]   = 1.0f;

	s_cal.flags = 0u;
}

/**
 * @brief  更新 IWDG 预分频/重装载并立即以新参数重载计数器
 * @param  pr     : PR 寄存器值（保持原分频即可）
 * @param  reload : RLR 寄存器值
 * @note   写 PR/RLR 只进缓冲，需等 SR 的 PVU/RVU 清零（LSI 同步，
 *         几十 µs）后再喂狗，否则 KR 重载用的仍是旧值；迭代上限
 *         防止 LSI 未启动时死等。不可用 HAL_GetTick 做等待基准：
 *         Flash 挂起期间 TIM4 tick 中断被合并，tick 不反映真实时间。
 */
static void IwdgApply(uint32_t pr, uint32_t reload)
{
	hiwdg.Instance->PR  = pr;
	hiwdg.Instance->RLR = reload;
	uint32_t guard = 200000u;
	while ((hiwdg.Instance->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0u) {
		if (--guard == 0u) { break; }
	}
	__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
}

/**
 * @brief  将校准 + PID 两条记录整页写入 Flash（共用 sector 7）
 * @retval true  擦写成功
 * @retval false 解锁或擦除/编程失败
 * @note   扇区擦除约 1 s 阻塞，调用方应避开关键时序。
 *         擦除期间 CPU 取指挂起、喂狗的 SafetyTask 同样停摆，
 *         故擦写前临时放宽 IWDG，结束后（无论成败）恢复原参数。
 *         两条记录总是同页重写：写 PID 不丢校准，写校准不丢 PID。
 *         从未落盘过 PID 时跳过 PID 记录（保持擦除态）。
 */
static bool ConfigSave(void)
{
	/* 先填 magic/ver，cks 清零后再求和，保证校验字段自身不参与累加 */
	s_cal.magic = CAL_MAGIC;
	s_cal.ver   = 1u;
	s_cal.cks   = 0u;
	s_cal.cks   = BlobCks(&s_cal);

	const bool pid_valid = (s_pid_saved.magic == PID_MAGIC);
	if (pid_valid) {
		s_pid_saved.ver = 1u;
		s_pid_saved.cks = 0u;
		s_pid_saved.cks = PidBlobCks(&s_pid_saved);
	}

	const uint32_t pr_save = hiwdg.Instance->PR;
	const uint32_t rl_save = hiwdg.Instance->RLR;
	IwdgApply(pr_save, CAL_IWDG_RELOAD_MAX);

	bool ok = false;
	if (HAL_FLASH_Unlock() == HAL_OK) {
		FLASH_EraseInitTypeDef er;
		uint32_t errsec = 0u;
		er.TypeErase    = FLASH_TYPEERASE_SECTORS;
		er.Sector       = CAL_FLASH_SECTOR;
		er.NbSectors    = 1u;
		er.VoltageRange = FLASH_VOLTAGE_RANGE_3;

		/* 整扇区擦除后逐字编程；任一字写失败即上锁退出 */
		if (HAL_FLASHEx_Erase(&er, &errsec) == HAL_OK) {
			const uint32_t *src = (const uint32_t *)(const void *)&s_cal;
			ok = true;
			for (uint32_t i = 0u; i < CAL_BLOB_WORDS; ++i) {
				if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
														CAL_FLASH_ADDR + (i * 4u), src[i]) != HAL_OK) {
					ok = false;
					break;
				}
			}
			if (ok && pid_valid) {
				const uint32_t *psrc = (const uint32_t *)(const void *)&s_pid_saved;
				for (uint32_t i = 0u; i < PID_BLOB_WORDS; ++i) {
					if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
															PID_FLASH_ADDR + (i * 4u), psrc[i]) != HAL_OK) {
						ok = false;
						break;
					}
				}
			}
		}
		(void)HAL_FLASH_Lock();
	}

	/* 无论成败都恢复原看门狗参数，避免长期放宽导致真实死机漏检 */
	IwdgApply(pr_save, rl_save);
	return ok;
}

/* ==========================================================================
 * 上电陀螺零偏自动校准：假设上电时机身静止，对一段时间内的陀螺读值取平均。
 * 结果只写 RAM（gyro_bias + CAL_FLAG_GYRO），需掉电保持时由上位机
 * 0xE0/CID 0x43 落盘。非静止样本直接丢弃；窗口超时则用已收集样本；
 * 有效样本太少则放弃（维持原 bias 不变）。
 * ========================================================================== */
#define GYRO_AC_SAMPLES  1000u    /* 目标静止样本数：1 kHz 采样约 1 s */
#define GYRO_AC_MIN       200u    /* 最少有效样本，不足判为环境不稳 */
#define GYRO_AC_MAX_TICKS 5000u   /* 窗口上限 5 s，超时用已有样本 */
#define GYRO_AC_STILL     0.02f   /* 单轴角速度静止阈值 rad/s（≈1.15 °/s），防慢速转动污染 */
#define GYRO_AC_RUN_MIN   30u     /* 起算门：先连续静止 30 拍才开始累计 */

static float    s_gac_sum[3];
static uint32_t s_gac_cnt;
static uint32_t s_gac_ticks;
static uint32_t s_gac_run;
static bool     s_gac_done;

/**
 * @brief  上电从 Flash 加载校准数据与已落盘 PID
 * @note   main.c USER CODE 2 调度器启动前调用，且必须先于 Anonym_Init
 *         （后者用 Calib_GetSavedPid 恢复 PID 缓存）；
 *         magic/ver/cks 三重校验任一失败则回退默认值。
 */
void Calib_Init(void)
{
	const CalibBlob *f = (const CalibBlob *)(const void *)CAL_FLASH_ADDR;

	if ((f->magic == CAL_MAGIC) && (f->ver == 1u) && (f->cks == BlobCks(f))) {
		s_cal = *f;
	} else {
		Defaults();   /* 首次上电/旧版本数据不兼容：单位阵 */
	}

	/* PID 记录独立校验：无效则镜像清零（落盘时跳过该记录） */
	const PidBlob *pf = (const PidBlob *)(const void *)PID_FLASH_ADDR;
	if ((pf->magic == PID_MAGIC) && (pf->ver == 1u) && (pf->cks == PidBlobCks(pf))) {
		s_pid_saved = *pf;
	} else {
		(void)memset(&s_pid_saved, 0, sizeof(s_pid_saved));
	}

	/* 每次上电重新做陀螺零偏自动校准 */
	(void)memset(s_gac_sum, 0, sizeof(s_gac_sum));
	s_gac_cnt   = 0u;
	s_gac_ticks = 0u;
	s_gac_run   = 0u;
	s_gac_done  = false;
}

/**
 * @brief  上电陀螺零偏自动校准单拍入口
 * @param  s : 本次采样（施加校准前的原始换算值）
 * @note   由 GY87_Sample 在 Calib_Apply 之前调用；校准完成后为空操作。
 */
void Calib_GyroAutoCal(SensorData *s)
{
	if (s_gac_done || (s == NULL) || (!s->valid)) { return; }

	s_gac_ticks++;

	/* 静止判据：加计模长接近 1 g 且三轴角速度都很小；
	 * 中途出现非静止样本即清空重算，保证 1 s 窗口取自连续静止段 */
	const float an2 = (s->ax_g * s->ax_g) + (s->ay_g * s->ay_g) + (s->az_g * s->az_g);
	const bool still = (an2 >= 0.81f) && (an2 <= 1.21f) &&
			(fabsf(s->gx_radps) < GYRO_AC_STILL) &&
			(fabsf(s->gy_radps) < GYRO_AC_STILL) &&
			(fabsf(s->gz_radps) < GYRO_AC_STILL);
	if (!still) {
		s_gac_run = 0u;
		s_gac_cnt = 0u;
		(void)memset(s_gac_sum, 0, sizeof(s_gac_sum));
	} else if (s_gac_run < GYRO_AC_RUN_MIN) {
		s_gac_run++;   /* 起算门未过，先确认连续静止 */
	} else {
		s_gac_sum[0] += s->gx_radps;
		s_gac_sum[1] += s->gy_radps;
		s_gac_sum[2] += s->gz_radps;
		s_gac_cnt++;
	}

	if ((s_gac_cnt < GYRO_AC_SAMPLES) && (s_gac_ticks < GYRO_AC_MAX_TICKS)) {
		return;
	}

	/* 窗口结束：样本足够则更新零偏并置位，否则维持原值 */
	s_gac_done = true;
	if (s_gac_cnt >= GYRO_AC_MIN) {
		const float inv = 1.0f / (float)s_gac_cnt;
		s_cal.gyro_bias[0] = s_gac_sum[0] * inv;
		s_cal.gyro_bias[1] = s_gac_sum[1] * inv;
		s_cal.gyro_bias[2] = s_gac_sum[2] * inv;
		s_cal.flags |= CAL_FLAG_GYRO;
	}
}

/**
 * @brief  对一次采样就地施加校准
 * @param  s : 传感器采样指针，为空或无效时直接返回
 * @note   gy87.c 出口调用；仅修正 flags 置位的分量，其余为恒等变换。
 *         校准原始值模式（上位机 CID 0x44 开启）下直接返回，
 *         使 0x01/0x02 上报未经校准的原始数据供上位机采样。
 */
void Calib_Apply(SensorData *s)
{
	if ((s == NULL) || (!s->valid)) {return;}
	if (s_cal_raw_mode) {return;}

	if (s_cal.flags & CAL_FLAG_GYRO) {   /* 陀螺：减零偏 */
		s->gx_radps -= s_cal.gyro_bias[0];
		s->gy_radps -= s_cal.gyro_bias[1];
		s->gz_radps -= s_cal.gyro_bias[2];
	}

	if (s_cal.flags & CAL_FLAG_ACC) {    /* 加速度：先减零偏再乘比例 */
		s->ax_g = (s->ax_g - s_cal.acc_off[0]) * s_cal.acc_sens[0];
		s->ay_g = (s->ay_g - s_cal.acc_off[1]) * s_cal.acc_sens[1];
		s->az_g = (s->az_g - s_cal.acc_off[2]) * s_cal.acc_sens[2];
	}

	if ((s_cal.flags & CAL_FLAG_MAG) && s->mag_ok) {
		/* 磁力计：Mag_cal = 软磁矩阵 × (原始值 - 硬磁偏置) */
		const float x = s->mx - s_cal.mag_off[0];
		const float y = s->my - s_cal.mag_off[1];
		const float z = s->mz - s_cal.mag_off[2];

		s->mx = (s_cal.mag_si[0] * x) + (s_cal.mag_si[1] * y) + (s_cal.mag_si[2] * z);
		s->my = (s_cal.mag_si[3] * x) + (s_cal.mag_si[4] * y) + (s_cal.mag_si[5] * z);
		s->mz = (s_cal.mag_si[6] * x) + (s_cal.mag_si[7] * y) + (s_cal.mag_si[8] * z);
	}
}

/**
 * @brief  读取校准参数
 * @param  id  : 参数 ID（CAL_ID_FIRST~CAL_ID_LAST，见 calib.h）
 * @param  val : 输出参数值，协议域 = 物理量 × 1e5
 * @retval true  读取成功
 * @retval false val 为空或 ID 越界
 */
bool Calib_GetParam(uint16_t id, int32_t *val)
{
    if (!val) return false;
    if (id >= CAL_ID_FIRST && id <= CAL_ID_LAST) {
        /* 各 ID 段连续且递增，级联比较即可定位对应数组元素 */
        const float *src = NULL;
        if (id <= CAL_ID_ACC_OFF_Z)      src = &s_cal.acc_off[id - CAL_ID_ACC_OFF_X];
        else if (id <= CAL_ID_ACC_SEN_Z) src = &s_cal.acc_sens[id - CAL_ID_ACC_SEN_X];
        else if (id <= CAL_ID_MAG_SI_22) src = &s_cal.mag_si[id - CAL_ID_MAG_SI_00];
        else if (id <= CAL_ID_MAG_OFF_Z) src = &s_cal.mag_off[id - CAL_ID_MAG_OFF_X];
        else                             src = &s_cal.gyro_bias[id - CAL_ID_GYRO_B_X];
        *val = F2I32(*src * 100000.0f);
        return true;
    }
    return false;
}

/**
 * @brief  把当前 RAM 校准数据落盘到 Flash
 * @note   两步式协议：0xE2 写参数只改 RAM；上位机再发 0xE0/CID 0x43
 *         （CMD0=0xA5 魔数）才调用本函数真正擦写 Flash。
 *         擦写时同步重写已落盘的 PID 记录，两者互不丢失。
 *         内部 ConfigSave 已临时放宽 IWDG，擦除阻塞期间不会复位。
 */
void Calib_SaveToFlash(void)
{
	(void)ConfigSave();
}

/**
 * @brief  把 PID 缓存写入持久化记录并整页落盘
 * @param  pid : CAL_PID_COUNT 个增益（内部域 0~100）
 * @retval true  擦写成功
 * @retval false 参数为空或 Flash 擦写失败
 * @note   先更新 RAM 镜像再落盘：失败时镜像与 Flash 一致（均为旧值），
 *         校准记录同页重写不会丢失。
 */
bool Calib_SavePidToFlash(const float *pid)
{
	if (pid == NULL) { return false; }

	s_pid_saved.magic = PID_MAGIC;
	for (uint32_t i = 0u; i < CAL_PID_COUNT; ++i) {
		s_pid_saved.pid[i] = pid[i];
	}
	return ConfigSave();
}

/**
 * @brief  读取上次落盘的 PID（上电恢复用）
 * @param  out : 输出数组，容量 CAL_PID_COUNT 个 float
 * @retval true  存在有效记录
 */
bool Calib_GetSavedPid(float *out)
{
	if ((out == NULL) || (s_pid_saved.magic != PID_MAGIC)) { return false; }
	(void)memcpy(out, s_pid_saved.pid, sizeof(s_pid_saved.pid));
	return true;
}

/**
 * @brief  写入校准参数
 * @param  id  : 参数 ID（CAL_ID_FIRST~CAL_ID_LAST）
 * @param  val : 参数值，协议域 = 物理量 × 1e5
 * @retval true  写入成功（仅更新 RAM；Flash 落盘由 0xE0/CID 0x43 触发）
 * @retval false ID 非法或越界值被护栏拒绝
 */
bool Calib_SetParam(uint16_t id, int32_t val)
{
	const float f = (float)val / 100000.0f;   /* 协议域 → 物理域 */

	if ((id >= CAL_ID_ACC_OFF_X) && (id <= CAL_ID_ACC_OFF_Z)) {
		s_cal.acc_off[id - CAL_ID_ACC_OFF_X] = f;
		s_cal.flags |= CAL_FLAG_ACC;
	} else if ((id >= CAL_ID_ACC_SEN_X) && (id <= CAL_ID_ACC_SEN_Z)) {
		if ((f < 0.5f) || (f > 2.0f)) { return false; }   /* 比例护栏 */
		s_cal.acc_sens[id - CAL_ID_ACC_SEN_X] = f;
		s_cal.flags |= CAL_FLAG_ACC;
	} else if ((id >= CAL_ID_MAG_SI_00) && (id <= CAL_ID_MAG_SI_22)) {
		if ((f < -2.0f) || (f > 2.0f)) { return false; }  /* 矩阵元护栏 */
		s_cal.mag_si[id - CAL_ID_MAG_SI_00] = f;
		s_cal.flags |= CAL_FLAG_MAG;
	} else if ((id >= CAL_ID_MAG_OFF_X) && (id <= CAL_ID_MAG_OFF_Z)) {
		if ((f < -10000.0f) || (f > 10000.0f)) { return false; }
		s_cal.mag_off[id - CAL_ID_MAG_OFF_X] = f;
		s_cal.flags |= CAL_FLAG_MAG;
	} else if ((id >= CAL_ID_GYRO_B_X) && (id <= CAL_ID_GYRO_B_Z)) {
		if ((f < -0.5f) || (f > 0.5f)) { return false; }  /* 零偏护栏 ≈ ±29°/s */
		s_cal.gyro_bias[id - CAL_ID_GYRO_B_X] = f;
		s_cal.flags |= CAL_FLAG_GYRO;
	} else {
		return false;
	}

	/* 两步式协议：写参数只改 RAM，不碰 Flash；
	 * 落盘由上位机显式发 0xE0/CID 0x43 触发 Calib_SaveToFlash。 */
	return true;
}

bool Calib_IsMagCalibrated(void)  { return (s_cal.flags & CAL_FLAG_MAG)  != 0u; }
bool Calib_IsAccCalibrated(void)  { return (s_cal.flags & CAL_FLAG_ACC)  != 0u; }
bool Calib_IsGyroCalibrated(void) { return (s_cal.flags & CAL_FLAG_GYRO) != 0u; }

void Calib_SetRawMode(bool on) { s_cal_raw_mode = on; }
bool Calib_IsRawMode(void)     { return s_cal_raw_mode; }
