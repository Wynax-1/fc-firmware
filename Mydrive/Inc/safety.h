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
#define SAFE_TILT_FATAL_RAD    1.047f     /* 60°，ARMED 下倾斜超限判据 */
#define SAFE_TILT_FATAL_US     500000u    /* 倾斜持续 500 ms 才判 FATAL，防抖 */
#define SAFE_ARM_MAX_TILT_RAD  (15.0f * 0.017453292f)   /* 解锁时允许最大倾角 */

/** 上电宽限期：此期间无条件喂狗 */
#define SAFE_IWDG_GRACE_US     3000000u

/* ==========================================================================
 * 故障位（日志/上报用）
 * ========================================================================== */
#define SAFE_FAULT_RC_STALE        (1u << 0)   /* 当前 RC 数据过期 */
#define SAFE_FAULT_ATT_STALE       (1u << 1)
#define SAFE_FAULT_CTRL_STALE      (1u << 2)
#define SAFE_FAULT_I2C_DEGRADED    (1u << 3)
#define SAFE_FAULT_ARMED_LINK_LOSS (1u << 4)   /* ARMED 状态下曾丢失 RC 链路 */
#define SAFE_FAULT_ALT_STALE       (1u << 5)

#define SAFE_WORD_ALLOWED   (1u << 0)
#define SAFE_WORD_ARMED     (1u << 1)
#define SAFE_WORD_FATAL     (1u << 2)
#define SAFE_WORD_CHAIN_OK  (1u << 3)

typedef enum {
    SAFETY_DISARMED = 0,
    SAFETY_ARM_PENDING,
    SAFETY_ARMED,
    SAFETY_FATAL
} SafetyState;

void Safety_Init(void);

/* 新增：rc 参数可为 NULL（视为 RC 不可用） */
void Safety_Step(const AttitudeSnapshot *att,
                 const ControlOutput *ctrl,
                 const RcCommandSnapshot *rc);

SafetyState Safety_GetState(void);
uint32_t    Safety_GetFaultMask(void);
void        Safety_ClearFatal(void);
bool        Safety_IsAllowed(void);
/* 上位机源解锁请求（0x01/0x02） */
void        Safety_RequestArm(bool on);
/* RC 源解锁请求（CH7 一键起飞）；与上位机源逻辑或，失锁路径各自撤销 */
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
