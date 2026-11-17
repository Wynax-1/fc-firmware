#include "Madgwick.h"
#include <math.h>
#include "calib.h"

/* 梯度下降步长 beta（越大越信任加速度计/磁力计，越小越信任陀螺仪） */
#define BETA_DEF 0.041f
#define SQ(x) ((x)*(x))

/* 模长平方阈值，用于避免 sqrt 计算 */
#define MAG_FIELD_NORM2_MIN     10000.0f
#define MAG_FIELD_NORM2_MAX     400000000.0f

/* 模长偏离参考值 ±30% ，因为对 n2 比较，所以是比例的平方*/
#define MAG_REF_LOW_RATIO   0.49f
#define MAG_REF_HIGH_RATIO  1.69f

/* 动态范围连续拒绝达到该帧数（200 Hz 下约 10 s）判参考值被污染，重新学习 */
#define MAG_REF_RELEARN_FRAMES 2000u

/* 全局四元数 */
static Quaternion_t q = {1.0f, 0.0f, 0.0f, 0.0f};

static float s_mag_ref_norm2 = 0.0f;   /* 参考磁场模长平方（用于动态判断） */
static bool s_mag_ref_initialized = false;
static bool s_mag_used = false; 		/*磁力计是否使用*/
static uint32_t s_mag_reject_cnt = 0u; /* 动态范围连续拒绝计数（参考值污染检测） */

/* CH5 三档设定的传感器使用上限；ControlTask 写、本任务读，
 * 单核 M4 上 4 字节对齐 enum 读写天然原子 */
static volatile SensorMode s_mode_ceiling = SENS_MODE_MARG;

void Attitude_SetSensorMode(SensorMode m)
{
	s_mode_ceiling = m;
}

SensorMode Attitude_GetSensorMode(void)
{
	return s_mode_ceiling;
}

/* ==========================================================================
 * 磁力计可用性判断
 * ========================================================================== */
static bool IsMagUsable(const SensorData *s)
{
	if (!s->mag_ok)                  { return false; }   /* 硬件故障 */
	if (!Calib_IsMagCalibrated())    { return false; }   /* 未校准 */

	const float n2 = SQ(s->mx) + SQ(s->my) + SQ(s->mz);

	/* 静态兜底：明显无效的模长直接拒 */
	if (n2 < MAG_FIELD_NORM2_MIN)  { return false; }   /* 太小 */
	if (n2 > MAG_FIELD_NORM2_MAX)  { return false; }   /* 太大，疑似饱和 */

	/* 动态参考：首次学习，后续偏离 30% 判为干扰 */

	if (!s_mag_ref_initialized) {
		s_mag_ref_norm2 = n2;
		s_mag_ref_initialized = true;
		return true;
	}
	const float lo = s_mag_ref_norm2 * MAG_REF_LOW_RATIO;
	const float hi = s_mag_ref_norm2 * MAG_REF_HIGH_RATIO;
	if ((n2 < lo) || (n2 > hi)) {
		/* 参考值被污染（如首次上电时处于强磁环境）会让后续读数永久误拒：
		 * 连续拒绝约 10 s 后以当前读数为基准重新学习 */
		if (++s_mag_reject_cnt >= MAG_REF_RELEARN_FRAMES) {
			s_mag_ref_norm2  = n2;
			s_mag_reject_cnt = 0u;
		}
		return false;
	}
	s_mag_reject_cnt = 0u;

	/* 缓慢更新参考值，适应地磁变化 */
	s_mag_ref_norm2 = s_mag_ref_norm2 * 0.999f + n2 * 0.001f;
	return true;
}

void Attitude_Init(void)
{
  q.q0 = 1.0f;
  q.q1 = 0.0f;
  q.q2 = 0.0f;
  q.q3 = 0.0f;
	s_mag_ref_norm2 = 0.0f;
	s_mag_ref_initialized = false;
	s_mag_reject_cnt = 0u;
}

/* ==========================================================================
 * 纯陀螺积分（加计不可信时的降级路径）
 * ========================================================================== */
static void UpdateGyro(const SensorData *s, float dt)
{
	const float q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
	const float gx = s->gx_radps, gy = s->gy_radps, gz = s->gz_radps;

	q.q0 += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
	q.q1 += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * dt;
	q.q2 += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * dt;
	q.q3 += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * dt;

	const float inv = 1.0f / sqrtf(SQ(q.q0) + SQ(q.q1) + SQ(q.q2) + SQ(q.q3));
	q.q0 *= inv; q.q1 *= inv; q.q2 *= inv; q.q3 *= inv;
}

/* ==========================================================================
 * 6 轴 IMU（加计 + 陀螺，不用磁力计）
 * ========================================================================== */
static void UpdateIMU(const SensorData *s, float dt)
{
	float recipNorm;
	float ax = s->ax_g, ay = s->ay_g, az = s->az_g;

	/* 归一化加计 */
	recipNorm = 1.0f / sqrtf(SQ(ax) + SQ(ay) + SQ(az));
	ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

	const float q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;

	/* 预计算 */
	const float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;

	/* 目标函数 f（加计 3 项） */
	const float f0 = _2q1 * q3 - _2q0 * q2 - ax;
	const float f1 = _2q0 * q1 + _2q2 * q3 - ay;
	const float f2 = _2q0 * q0 + _2q3 * q3 - 1.0f - az;

	/* 梯度 s */
	float s0 = -_2q2 * f0 + _2q1 * f1;
	float s1 =  _2q3 * f0 + _2q0 * f1 - 4.0f * q1 * f2;
	float s2 = -_2q0 * f0 + _2q3 * f1 - 4.0f * q2 * f2;
	float s3 =  _2q1 * f0 + _2q2 * f1;

	recipNorm = 1.0f / sqrtf(SQ(s0) + SQ(s1) + SQ(s2) + SQ(s3));
	s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

	/* 陀螺导数 + 融合 */
	const float gx = s->gx_radps, gy = s->gy_radps, gz = s->gz_radps;
	q.q0 += (0.5f * (-q1 * gx - q2 * gy - q3 * gz) - BETA_DEF * s0) * dt;
	q.q1 += (0.5f * ( q0 * gx + q2 * gz - q3 * gy) - BETA_DEF * s1) * dt;
	q.q2 += (0.5f * ( q0 * gy - q1 * gz + q3 * gx) - BETA_DEF * s2) * dt;
	q.q3 += (0.5f * ( q0 * gz + q1 * gy - q2 * gx) - BETA_DEF * s3) * dt;

	recipNorm = 1.0f / sqrtf(SQ(q.q0) + SQ(q.q1) + SQ(q.q2) + SQ(q.q3));
	q.q0 *= recipNorm; q.q1 *= recipNorm; q.q2 *= recipNorm; q.q3 *= recipNorm;
}

/* ==========================================================================
 * 9 轴 MARG（现有逻辑搬过来，逻辑不变）
 * ========================================================================== */
static void UpdateMARG(const SensorData *s, float dt)
{
	float recipNorm;
	float q0=q.q0, q1=q.q1, q2=q.q2, q3=q.q3;
	float f0, f1, f2, f3, f4, f5;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0q0, _2q0q1, _2q0q2, _2q0q3, _2q1q1, _2q1q2, _2q1q3, _2q2q2, _2q2q3, _2q3q3, bx, bz, _2q0, _2q1, _2q2, _2q3;
	float _2q0bx, _2q0bz, _2q1bx, _2q1bz, _2q2bx, _2q2bz, _2q3bx, _2q3bz;
	
  float ax = s->ax_g, ay = s->ay_g, az = s->az_g;
  float mx = s->mx, my = s->my, mz = s->mz;
  float gx = s->gx_radps, gy = s->gy_radps, gz = s->gz_radps;


  /* 归一化加速度计 */
  recipNorm = 1.0f / sqrtf(SQ(ax) + SQ(ay) + SQ(az));
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  /* 归一化磁力计 */
  recipNorm = 1.0f / sqrtf(SQ(mx) + SQ(my) + SQ(mz));
  mx *= recipNorm;
  my *= recipNorm;
  mz *= recipNorm;

  /* 预计算公共项 */
	_2q0 = 2.0f * q0;
	_2q1 = 2.0f * q1;
	_2q2 = 2.0f * q2;
	_2q3 = 2.0f * q3;
  _2q0q0 = 2.0f * q0 * q0;
  _2q0q1 = 2.0f * q0 * q1;
  _2q0q2 = 2.0f * q0 * q2;
  _2q0q3 = 2.0f * q0 * q3;
  _2q1q1 = 2.0f * q1 * q1;
  _2q1q2 = 2.0f * q1 * q2;
  _2q1q3 = 2.0f * q1 * q3;
  _2q2q2 = 2.0f * q2 * q2;
  _2q2q3 = 2.0f * q2 * q3;
  _2q3q3 = 2.0f * q3 * q3;

  /* 计算参考磁场方向（强制水平分量指向 X 轴） */
  hx = mx * (_2q0q0 + _2q1q1 - 1.0f) + my * (_2q1q2 - _2q0q3) + mz * (_2q1q3 + _2q0q2);
  hy = mx * (_2q1q2+_2q0q3) + my * (_2q0q0 + _2q2q2 - 1.0f) + mz * (_2q2q3 - _2q0q1);
  bx = sqrtf(hx * hx + hy * hy);
  bz = mx * (_2q1q3 - _2q0q2) + my * (_2q0q1 + _2q2q3) + mz * (_2q0q0 + _2q3q3 - 1.0f);
	
	/* 误差向量（加速度计） */
	f0 = _2q1q3 - _2q0q2 - ax;
	f1 = _2q2q3 - _2q0q1 - ay;
	f2 = _2q0q0 + _2q3q3 - 1.0f - az;
	
	/* 误差向量（磁力计） */
	f3 = bx * (_2q0q0 + _2q1q1 - 1) + bz * (_2q1q3 - _2q0q2) - mx;
	f4 = bx * (_2q1q2 - _2q0q3) + bz * (_2q0q1 + _2q2q3) - my;
	f5 = bx * (_2q1q3 + _2q0q2) + bz * (_2q0q0 + _2q3q3 -1.0f) -mz;
	
	_2q0bx = _2q0 * bx;
	_2q0bz = _2q0 * bz;
	_2q1bx = _2q1 * bx;
	_2q1bz = _2q1 * bz;
	_2q2bx = _2q2 * bx;
	_2q2bz = _2q2 * bz;
	_2q3bx = _2q3 * bx;
	_2q3bz = _2q3 * bz;

  /* 梯度向量 s（闭合形式） */
  s0 = -_2q2*f0 + _2q1*f1 - _2q2bz*f3 + (_2q1bz - _2q3bx)*f4 + (_2q2bx)*f5;
  s1 = _2q3*f0 + _2q0*f1 - 4.0f*q1*f2 + _2q3bz*f3 + (_2q2bx + _2q0bz)*f4 + (_2q3bx - _2q1bz)*f5;
  s2 = -_2q0*f0 + _2q3*f1 - 4.0f*q2*f2 - (2.0f*_2q2bx + _2q0bz)*f3 + (_2q1bx+_2q3bx)*f4 + (_2q0bx - 2.0f*_2q2bz)*f5;
  s3 = _2q1*f0 + _2q2*f1 + (_2q2bz - 2.0f*_2q3bx)*f3 + (_2q2bz - _2q0bx)*f4 + _2q1bx*f5;

  /* 归一化梯度 */
  recipNorm = 1.0f / sqrtf(SQ(s0) + SQ(s1) + SQ(s2) + SQ(s3));
  s0 *= recipNorm;
  s1 *= recipNorm;
  s2 *= recipNorm;
  s3 *= recipNorm;

  /* 陀螺仪四元数导数 */
  qDot1 = 0.5f * (-q.q1 * gx - q.q2 * gy - q.q3 * gz);
  qDot2 = 0.5f * ( q.q0 * gx + q.q2 * gz - q.q3 * gy);
  qDot3 = 0.5f * ( q.q0 * gy - q.q1 * gz + q.q3 * gx);
  qDot4 = 0.5f * ( q.q0 * gz + q.q1 * gy - q.q2 * gx);

  /* 互补融合：梯度下降 + 陀螺仪积分 */
  q.q0 += (qDot1 - BETA_DEF * s0) * dt;
  q.q1 += (qDot2 - BETA_DEF * s1) * dt;
  q.q2 += (qDot3 - BETA_DEF * s2) * dt;
  q.q3 += (qDot4 - BETA_DEF * s3) * dt;

  /* 归一化四元数 */
  recipNorm = 1.0f / sqrtf(SQ(q.q0) + SQ(q.q1) + SQ(q.q2) + SQ(q.q3));
  q.q0 *= recipNorm;
  q.q1 *= recipNorm;
  q.q2 *= recipNorm;
  q.q3 *= recipNorm;
}

/**
 * @brief  用一帧传感器数据更新姿态
 * @param  s   : 本拍传感器快照（只用 ax/ay/az、gx/gy/gz、mag、mag_ok）
 * @param  dt  : 距上一拍的间隔，单位 s
 * @retval true  本次更新有效
 * @retval false dt 或数据异常，四元数保持不变（冻结）
 * @note   降级策略：
 *         加计不可信 + 磁力计不可信 → 纯陀螺（短时保姿态，长时会漂）
 *         加计可信 + 磁力计不可信   → 6 轴（yaw 会漂移，roll/pitch 稳定）
 *         加计可信 + 磁力计可信     → 9 轴（完整 MARG）
 *         dt 合法范围 [2 ms, 20 ms]，超出直接冻结四元数。
 *         上限取 20 ms：正常 400 kHz 总线 dt≈5 ms，放宽上限以容忍偶发的
 *         I2C 总线恢复/丢样本造成的间隔增大，避免误判超时把姿态冻结。
 */
/* ==========================================================================
 * 主入口：三档切换
 * 注意：本文件内部滤波是 x-ioh NWU 系（Z上 Y西，平放加计 Z 读 +1g，
 * 由 f2 = 2q0²+2q3²-1-az 可证）；而模块对外的 SensorData/四元数/欧拉角
 * 统一为 FRD 机体系 + NED 世界系。因此：
 *   入口：FRD→NWU（翻转 Y/Z，矢量变换）；
 *   出口：q_ned = (-w, -x, y, z)（已对纯 roll/pitch/yaw 逐轴验证）。
 * ========================================================================== */
static void FrdToNwu(const SensorData *in, SensorData *out)
{
    *out = *in;
    out->ay_g     = -in->ay_g;
    out->az_g     = -in->az_g;
    out->gy_radps = -in->gy_radps;
    out->gz_radps = -in->gz_radps;
    out->my       = -in->my;
    out->mz       = -in->mz;
}

bool Attitude_Update(const SensorData *s, float dt_s)
{
	if ((s == NULL) || (!s->valid))              { return false; }
	if ((dt_s < 0.002f) || (dt_s > 0.020f))      { return false; }

	SensorData nwu;
	FrdToNwu(s, &nwu);
	s = &nwu;

	s_mag_used = false;
	
	const float acc_norm2 = SQ(s->ax_g) + SQ(s->ay_g) + SQ(s->az_g);
	/* 上限（CH5）AND 可用性：档位只设上限，能否真用仍由判据决定 */
	const bool acc_usable = (s_mode_ceiling >= SENS_MODE_IMU)
	                        && (acc_norm2 >= 0.25f) && (acc_norm2 <= 2.25f);  /* [0.5, 1.5] g */
	const bool mag_usable = (s_mode_ceiling >= SENS_MODE_MARG) && IsMagUsable(s);

	if (!acc_usable && !mag_usable) {
		UpdateGyro(s, dt_s);
	} else if (!mag_usable) {
		UpdateIMU(s, dt_s);
	} else {
		UpdateMARG(s, dt_s);
		s_mag_used = mag_usable;
	}
	return true;
}

/**
 * @brief  读取当前欧拉角
 * @retval 欧拉角（rad）
 * @note   内部 q 为 NWU，先转 NED/FRD（q_ned = (-w, -x, y, z)）再提取。
 */
EulerAngles_t Attitude_GetEulerAngles(void)
{
	const float q0 = -q.q0, q1 = -q.q1, q2 = q.q2, q3 = q.q3;
	EulerAngles_t e;

	/* roll = atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²)) */
	const float sinr = 2.0f * (q0 * q1 + q2 * q3);
	const float cosr = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
	e.roll = atan2f(sinr, cosr);

	/* pitch = asin(2(q0q2 - q3q1))，asin 入参必须夹紧到 [-1,1] */
	float sinp = 2.0f * (q0 * q2 - q3 * q1);
	sinp = FC_SAT(sinp, -1.0f, 1.0f);
	e.pitch = asinf(sinp);

	/* yaw = atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²)) */
	const float siny = 2.0f * (q0 * q3 + q1 * q2);
	const float cosy = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
	e.yaw = atan2f(siny, cosy);

	return e;
}

/**
 * @brief  读取当前四元数（对外：NED 世界系 / FRD 机体系）
 * @note   内部 q 为 NWU，出口变换 q_ned = (-w, -x, y, z)。
 */
Quaternion_t Attitude_GetQuaternion(void)
{
	Quaternion_t n;
	n.q0 = -q.q0;
	n.q1 = -q.q1;
	n.q2 =  q.q2;
	n.q3 =  q.q3;
	return n;
}

bool Attitude_IsMagUsed(void)
{
	return s_mag_used;
}
