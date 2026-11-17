#ifndef __CONTROL_TYPES_H
#define __CONTROL_TYPES_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>

typedef uint32_t fc_usec_t;						//快照时间戳
typedef uint32_t fc_seq_t;						//快照序列号

//快照邮戳
typedef struct {
	fc_usec_t usec;
	fc_seq_t 	seq;
}FcStamp;


/** @brief 四元数（w, x, y, z），单位模长 */
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_t;

/** @brief 欧拉角，单位 rad */
typedef struct {
    float roll;   /**< 右滚为正 */
    float pitch;  /**< 抬头为正 */
    float yaw;    /**< 右偏为正 */
} EulerAngles_t;

/* ==========================================================================
 * 传感器快照：BusTask -> AttitudeTask
 * ========================================================================== */
/**
 * @brief  一次 IMU 采样结果
 * @note   由 BusTask 唯一发布；AttitudeTask 只读。
 *         角速度单位 rad/s，加速度单位 g，磁数据未归一化。
 */
typedef struct {
	FcStamp stamp;      /**< 采样邮戳*/

	float ax_g;         /**< X 轴加速度，单位 g */
	float ay_g;         /**< Y 轴加速度，单位 g */
	float az_g;         /**< Z 轴加速度，单位 g */

	float gx_radps;     /**< 绕 X 轴角速度，单位 rad/s（右滚为正） */
	float gy_radps;     /**< 绕 Y 轴角速度，单位 rad/s（抬头为正） */
	float gz_radps;     /**< 绕 Z 轴角速度，单位 rad/s（右偏航为正） */

	float mx;           /**< 磁力计 X 原始值 */
	float my;           /**< 磁力计 Y 原始值 */
	float mz;           /**< 磁力计 Z 原始值 */

	bool  mag_ok;       /**< true 表示本拍的磁数据可用 */
	bool  valid;        /**< true 表示 IMU 读取与换算成功 */
} SensorData;

/**
 * @brief  将数值饱和限制到闭区间 [lo, hi]
 * @param  v  待限制的值
 * @param  lo 下限
 * @param  hi 上限，必须满足 lo <= hi
 * @retval 限制后的值
 */
static inline float FC_SAT(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ==========================================================================
 * 遥控快照：RcTask -> ControlTask / SafetyTask
 * ========================================================================== */
typedef struct {
    FcStamp  stamp;      /**< 最近一帧有效 PPM 的接收时刻 */
    bool     valid;      /**< true = 当前接收机存活（未失联） */
    uint16_t ch[8];      /**< 8 通道原始脉宽 µs（无效通道为 0） */
} RcCommandSnapshot;

/* ==========================================================================
 * 姿态快照：AttitudeTask -> ControlTask / SafetyTask / CommTask
 * ========================================================================== */

/**
 * @brief  姿态解算输出
 * @note   由 AttitudeTask 在每次收到新传感器样本后发布（约 250 Hz）。
 *         仅含解算结果（四元数、欧拉角、有效性、是否用磁力计）；
 *         解算输入（陀螺、加速度）请直接读 g_chan_sensor。
 */
typedef struct {
    FcStamp stamp;      /**< 解算所用样本的时刻 + 序列 */
    float q0;           /**< 姿态四元数 w（0x04 帧数据源，发布前已归一化） */
    float q1;           /**< 姿态四元数 x */
    float q2;           /**< 姿态四元数 y */
    float q3;           /**< 姿态四元数 z */
    
    float roll_rad;     /**< 横滚角，单位 rad，右滚为正 */
    float pitch_rad;    /**< 俯仰角，单位 rad，抬头为正 */
    float yaw_rad;      /**< 偏航角，单位 rad，右偏为正（6 轴下会缓慢漂移） */

    bool  valid;        /**< true 表示姿态可信（dt 合法且解算收敛） */
    bool  mag_used;     /**< true 表示本次解算使用了磁力计 */
} AttitudeSnapshot;

/* ==========================================================================
 * 高度快照：BusTask -> ControlTask / CommTask / SafetyTask
 * ========================================================================== */
typedef enum {
    ALT_SRC_NONE = 0,
    ALT_SRC_US   = 1,   /* 超声波有效 */
    ALT_SRC_BARO = 2,   /* 气压计有效 */
} AltSource;

typedef struct {
    FcStamp stamp;
    float   height_m;   /**< 相对起飞点高度，米 */
    float   vz_mps;     /**< 垂直速度估计，m/s */
    uint8_t source;     /**< AltSource */
    bool    valid;
} AltitudeSnapshot;

/* ==========================================================================
 * 控制输出：ControlTask -> MixerTask(同任务) / CommTask
 * ========================================================================== */

/** @brief 定高/一键起飞辅助层中止原因 */
typedef enum {
    ALT_ABORT_NONE     = 0u,  /**< 未中止 */
    ALT_ABORT_ALT_LOSS = 1u,  /**< 高度数据失效，退回降级手动油门 */
    ALT_ABORT_RC_LOSS  = 2u,  /**< RC 丢失（Safety 同步复锁） */
    ALT_ABORT_ATT_LOSS = 3u,  /**< 姿态数据失效 */
    ALT_ABORT_PRECOND  = 4u,  /**< CH7 边沿解锁请求被 Safety 拒绝 */
    ALT_ABORT_CH7_LOW  = 5u,  /**< CH7 拨回 L 触发正常缓降（非故障） */
    ALT_ABORT_HOST     = 6u,  /**< 上位机 0x02 夺权锁定 */
    ALT_ABORT_BENCH    = 7u,  /**< DBG 模式接管，辅助层旁路 */
} AltAbortReason;

/**
 * @brief  归一化控制输出
 * @note   由 ControlTask 唯一发布，并在同一拍直接送混控。
 *         本结构不含脉宽/tick，单位无关，便于整定与移植。
 */
typedef struct {
    FcStamp stamp;          /**< 生成时刻 + 序列 */

    float motor_base_01;    /**< 基础油门，[0,1] */
    float roll_torque_nd;   /**< 横滚扭矩，[-1,1]，正=右滚 */
    float pitch_torque_nd;  /**< 俯仰扭矩，[-1,1]，正=抬头 */
    float yaw_torque_nd;    /**< 偏航扭矩，[-1,1]，正=右偏 */

    bool  assist_active;    /**< true 表示基础油门由辅助层（定高/一键起飞）给出 */
    float alt_target_m;     /**< 当前辅助层目标高度，单位 m；手动时为 0 */
    uint8_t abort_reason;   /**< AltAbortReason；0 表示未中止，粘滞到下一次 CH7 边沿 */

    bool  valid;            /**< true 表示本拍输出可信 */
} ControlOutput;

/* ==========================================================================
 * 气压快照（altitude.c 内部使用；不单独发布通道）
 * ========================================================================== */

typedef struct {
    FcStamp stamp;
    int32_t pressure_pa;      /**< 补偿后气压，单位 Pa */
    float   temperature_c;    /**< 补偿后温度，单位 °C */
    bool    valid;
} BaroSnapshot;

#endif //__CONTROL_TYPES_H
