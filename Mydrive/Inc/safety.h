#ifndef SAFETY_H
#define SAFETY_H

#include "control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 判据阈值
 * ========================================================================== */
/** RC 数据过期阈值：PPM 帧 20 ms，容忍连丢 2 帧（权威定义，rc.c 也使用） */
#define SAFE_RC_STALE_US       60000u
/** 姿态数据过期阈值：AT 200 Hz，正常 5 ms 一次 */
#define SAFE_ATT_STALE_US      50000u
/** 控制输出过期阈值：CT 500 Hz，正常 2 ms 一次；阈值 50 ms 容忍调度抖动 */
#define SAFE_CTRL_STALE_US     50000u
/** 高度数据过期阈值：只用于诊断，不参与授权 */
#define SAFE_ALT_STALE_US      300000u

/* ---- 状态机参数 ---- */
#define SAFE_ARM_CONFIRM_US    200000u    /* ARMING 持续 200 ms 才确认 ARMED */
/* 本机油门余量有限：倾角 45° 时维持高度需 1/cos45°≈1.41 倍悬停推力，
 * 已无法拉回，因此 45° 即判 FATAL，不再等到 60°。 */
#define SAFE_TILT_FATAL_RAD    (45.0f * 0.017453292f)
#define SAFE_TILT_FATAL_US     500000u    /* 倾斜持续 500 ms 才判 FATAL，防抖 */
#define SAFE_ARM_MAX_TILT_RAD  (15.0f * 0.017453292f)   /* 解锁时允许最大倾角 */

/** ARMED 下 RC 连续丢失超过此值进入 FAILSAFE（自动缓降着陆）。
 *  在 60 ms 的 PPM 老化判定基础上再容忍 250 ms，合计约 310 ms 内不动作 */
#define SAFE_RC_FAILSAFE_US      250000u
/** FAILSAFE 持续此值仍无 RC 恢复才最终失锁（拉低电机） */
#define SAFE_FAILSAFE_DISARM_US  2000000u
/** FATAL 自动恢复：坠地扶平 + 姿态恢复，连续健康多久才回 DISARMED */
#define SAFE_FATAL_CLEAR_US      1000000u

/* ==========================================================================
 * 故障位（日志/上报用）
 * ========================================================================== */
#define SAFE_FAULT_RC_STALE        (1u << 0)   /* 当前 RC 数据过期 */
#define SAFE_FAULT_ATT_STALE       (1u << 1)
#define SAFE_FAULT_CTRL_STALE      (1u << 2)
#define SAFE_FAULT_I2C_DEGRADED    (1u << 3)   /* 总线级诊断（含磁力计/气压计），不用于飞行决策 */
#define SAFE_FAULT_ARMED_LINK_LOSS (1u << 4)   /* ARMED 状态下曾丢失链路 */
#define SAFE_FAULT_ALT_STALE       (1u << 5)
#define SAFE_FAULT_IMU_DEGRADED    (1u << 6)   /* 关键 IMU（MPU6050）降级 */
#define SAFE_FAULT_TILT_FATAL      (1u << 7)   /* 倾角超限触发的锁存 */

#define SAFE_WORD_ALLOWED   (1u << 0)
#define SAFE_WORD_ARMED     (1u << 1)
#define SAFE_WORD_FATAL     (1u << 2)
#define SAFE_WORD_CHAIN_OK  (1u << 3)

typedef enum {
    SAFETY_DISARMED = 0,
    SAFETY_ARM_PENDING,
    SAFETY_ARMED,
    SAFETY_FAILSAFE,   /* RC 持续丢失：水平自稳 + 油门缓降着陆，等待链路恢复 */
    SAFETY_FATAL       /* 倾角超限的 crash-detection 锁存；自动恢复后回 DISARMED */
} SafetyState;

void Safety_Init(void);

/* 新增：rc 参数可为 NULL（视为 RC 不可用） */
void Safety_Step(const AttitudeSnapshot *att,
                 const ControlOutput *ctrl,
                 const RcCommandSnapshot *rc);

SafetyState Safety_GetState(void);
/* 事件历史（锁存）：自上次上电以来的安全事件，仅诊断，上电清零 */
uint32_t    Safety_GetFaultMask(void);
/* 当前实时故障：随条件消失自动清零 */
uint32_t    Safety_GetActiveFaultMask(void);
bool        Safety_IsAllowed(void);
/* 上位机源解锁请求（0x01/0x02） */
void        Safety_RequestArm(bool on);
/* RC 源解锁请求（CH7 电机锁 / 一键起飞）；与上位机源逻辑或，失锁路径各自撤销。
 * 两种编译配置都可用：一键起飞关闭时该开关就是纯电机锁 */
void        Safety_RequestArmRc(bool on);
bool        Safety_IsArmed(void);

/* ==========================================================================
 * 安全日志（SafetyTask 生产 / CommTask 消费）
 * ========================================================================== */
/* SafetyTask 侧：非阻塞写入（RTOS 临界区，无 RTOS 队列操作） */
void Safety_Log(uint8_t color, const char *str);

/* CommTask 侧：取出并清标志。有可发消息返回 true */
bool Safety_TakeLog(uint8_t *color_out, char *out, uint8_t cap);

/* 台架强制允许 */
#ifdef SAFE_BENCH_FORCE_ARM
void Safety_BenchForceAllowed(bool on);
#endif

#ifdef __cplusplus
}
#endif
#endif
