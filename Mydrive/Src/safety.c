#include "safety.h"
#include "fc_time.h"
#include "i2c_adapter.h"
#include "gy87.h"          /* GY87_MPU_ADDR：关键设备判定 */
#include "iwdg.h"
#include "rc.h"            /* RC_THR 枚举、RC_THR_ARM_WIN_*_US 解锁窗口 */
#include "FreeRTOS.h"
#include "task.h"          /* taskENTER_CRITICAL */
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ==========================================================================
 * 内部状态
 * ========================================================================== */
static fc_usec_t s_arming_start_us = 0u;
static fc_usec_t s_tilt_start_us   = 0u;
static bool      s_tilt_active     = false;

static volatile uint32_t s_word = 0u;

#ifdef SAFE_BENCH_FORCE_ARM
static volatile bool s_bench_override = false;
#endif

static SafetyState s_state         = SAFETY_DISARMED;
static uint32_t    s_fault_live    = 0u;   /* 本拍实时条件 */
static uint32_t    s_fault_latched = 0u;   /* 事件历史（锁存，上电才清） */
static bool        s_inited        = false;

static volatile bool s_host_arm_req = false;
static volatile bool s_rc_arm_req   = false;   /* RC 源（CH7 电机锁/一键起飞）解锁请求 */

/* ---- RC 失联分级 ---- */
static fc_usec_t s_rc_loss_start_us  = 0u;
static bool      s_rc_loss_active    = false;
static fc_usec_t s_failsafe_start_us = 0u;

/* ---- FATAL 自动恢复 ---- */
static fc_usec_t s_fatal_clear_start_us = 0u;
static bool      s_fatal_clear_active   = false;

/* ---- 安全日志（SafetyTask 生产 / CommTask 消费） ---- */
static volatile char    s_log_msg[40];
static volatile uint8_t s_log_color   = 0u;
static volatile bool    s_log_pending = false;

static void PublishWord(bool allowed, bool armed, bool fatal, bool chain_ok)
{
    uint32_t w = 0u;
    if (allowed)  { w |= SAFE_WORD_ALLOWED;  }
    if (armed)    { w |= SAFE_WORD_ARMED;    }
    if (fatal)    { w |= SAFE_WORD_FATAL;    }
    if (chain_ok) { w |= SAFE_WORD_CHAIN_OK; }
    s_word = w;
}

static bool AttitudeLevel(const AttitudeSnapshot *att)
{
    if ((att == NULL) || (!att->valid)) { return false; }
    return (fabsf(att->roll_rad)  <= SAFE_ARM_MAX_TILT_RAD) &&
           (fabsf(att->pitch_rad) <= SAFE_ARM_MAX_TILT_RAD);
}

/* ==========================================================================
 * 安全日志：非阻塞，单 writer（SafetyTask），单 reader（CommTask）
 * ========================================================================== */
void Safety_Log(uint8_t color, const char *str)
{
    if ((str == NULL) || (str[0] == '\0')) { return; }

    taskENTER_CRITICAL();
    uint8_t n = 0u;
    while ((n < 38u) && (str[n] != '\0')) {
        s_log_msg[n] = str[n];
        n++;
    }
    s_log_msg[n] = '\0';
    s_log_color   = color;
    s_log_pending = true;
    taskEXIT_CRITICAL();
}

bool Safety_TakeLog(uint8_t *color_out, char *out, uint8_t cap)
{
    if ((out == NULL) || (cap == 0u)) { return false; }
    if (!s_log_pending) { return false; }   /* 快路径 */

    bool got = false;
    taskENTER_CRITICAL();
    if (s_log_pending) {
        if (color_out != NULL) { *color_out = s_log_color; }
        uint8_t n = 0u;
        while ((n + 1u < cap) && (s_log_msg[n] != '\0')) {
            out[n] = s_log_msg[n];
            n++;
        }
        out[n] = '\0';
        s_log_pending = false;
        got = true;
    }
    taskEXIT_CRITICAL();
    return got;
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */
void Safety_Init(void)
{
    s_word         = 0u;
    s_state        = SAFETY_DISARMED;
    s_fault_live   = 0u;
    s_fault_latched = 0u;
    s_host_arm_req = false;
    s_rc_arm_req   = false;
    s_tilt_active  = false;
    s_arming_start_us = 0u;
    s_tilt_start_us   = 0u;
    s_rc_loss_start_us  = 0u;
    s_rc_loss_active    = false;
    s_failsafe_start_us = 0u;
    s_fatal_clear_start_us = 0u;
    s_fatal_clear_active   = false;
    s_log_pending  = false;
    s_log_msg[0]   = '\0';
    s_inited       = true;
}

void Safety_Step(const AttitudeSnapshot *att,
                 const ControlOutput *ctrl,
                 const RcCommandSnapshot *rc)
{
    if (!s_inited) { Safety_Init(); }

    const fc_usec_t now = Fc_GetMicros();

    /* ---- RC 即时健康（从快照读取） ---- */
    const bool rc_alive = (rc != NULL) && rc->valid;
    /* 油门解锁判据：归一化域，随 rc.h 的 RC_CAL_* 标定，端点偏差不影响 */
    const float thr_n  = (rc != NULL) ? Rc_ThrottleNorm(rc->ch[RC_THR]) : 0.0f;
    const bool thr_low = rc_alive
                      && (thr_n >= RC_THR_ARM_WIN_LO_N)
                      && (thr_n <= RC_THR_ARM_WIN_HI_N);

    /* ---- 数据年龄与即时健康 ---- */
    const uint32_t att_age = ((att != NULL) && att->valid)
                             ? Fc_AgeUs(att->stamp.usec, now) : 0xFFFFFFFFu;
    const uint32_t ctl_age = (ctrl != NULL)
                             ? Fc_AgeUs(ctrl->stamp.usec, now) : 0xFFFFFFFFu;

    const bool att_ok   = (att_age <= SAFE_ATT_STALE_US);
    const bool ctl_ok   = (ctl_age <= SAFE_CTRL_STALE_US);
    const bool chain_ok = att_ok && ctl_ok;
    const bool level_ok = AttitudeLevel(att);

    /* ---- I2C 分级：只有 MPU 是关键设备，总线级仅诊断 ---- */
    const bool imu_bad = I2C_AuditDeviceDegraded(GY87_MPU_ADDR);
    const bool bus_bad = I2C_AuditDegraded();

    /* ---- 实时故障位：随条件消失自动清零 ---- */
    uint32_t live = 0u;
    if (!rc_alive) live |= SAFE_FAULT_RC_STALE;
    if (!att_ok)   live |= SAFE_FAULT_ATT_STALE;
    if (!ctl_ok)   live |= SAFE_FAULT_CTRL_STALE;
    if (bus_bad)   live |= SAFE_FAULT_I2C_DEGRADED;
    if (imu_bad)   live |= SAFE_FAULT_IMU_DEGRADED;
    s_fault_live = live;

    /* ---- 倾斜超限（仅 ARMED 下累计；FATAL 是 crash-detection） ---- */
    bool tilt_fatal = false;
    if ((s_state == SAFETY_ARMED) && (att != NULL) && att->valid) {
        const bool tilted =
            (fabsf(att->roll_rad)  > SAFE_TILT_FATAL_RAD) ||
            (fabsf(att->pitch_rad) > SAFE_TILT_FATAL_RAD);
        if (tilted) {
            if (!s_tilt_active) {
                s_tilt_active   = true;
                s_tilt_start_us = now;
            } else if (Fc_AgeUs(s_tilt_start_us, now) >= SAFE_TILT_FATAL_US) {
                tilt_fatal = true;
            }
        } else {
            s_tilt_active = false;
        }
    } else {
        s_tilt_active = false;
    }

    /* ---- 状态机 ---- */
    /* 解锁双源：上位机 0x01/0x02 与 CH7（电机锁/一键起飞）逻辑或；
     * 失锁路径各自撤销（见下） */
    const bool arm_req = s_host_arm_req || s_rc_arm_req;
    switch (s_state) {
    case SAFETY_DISARMED:
        if (arm_req && chain_ok && level_ok && !imu_bad
            && rc_alive && thr_low) {
            s_state           = SAFETY_ARM_PENDING;
            s_arming_start_us = now;
            Safety_Log(0u, "SAFETY: ARMING");
        }
        break;

    case SAFETY_ARM_PENDING:
        if (!arm_req || !chain_ok || !level_ok || imu_bad
            || !rc_alive || !thr_low) {
            s_state = SAFETY_DISARMED;
            Safety_Log(0u, "SAFETY: ARM ABORTED");
        } else if (Fc_AgeUs(s_arming_start_us, now) >= SAFE_ARM_CONFIRM_US) {
            s_state = SAFETY_ARMED;
            Safety_Log(2u, "SAFETY: ARMED");
        }
        break;

    case SAFETY_ARMED:
        if (!arm_req) {
            s_state = SAFETY_DISARMED;
            s_rc_loss_active = false;
            Safety_Log(0u, "SAFETY: DISARMED by host/rc");
        } else if (!att_ok) {
            /* 姿态链断（含 IMU 持续降级）：无法自稳，失锁 */
            if (imu_bad) { s_fault_latched |= SAFE_FAULT_IMU_DEGRADED; }
            s_fault_latched |= SAFE_FAULT_ARMED_LINK_LOSS;
            s_host_arm_req = false;
            s_rc_arm_req   = false;
            s_rc_loss_active = false;
            s_state = SAFETY_DISARMED;
            Safety_Log(1u, "SAFETY: DISARMED att lost");
        } else if (!ctl_ok) {
            /* ControlTask 卡死：兜底失锁 */
            if (imu_bad) { s_fault_latched |= SAFE_FAULT_IMU_DEGRADED; }
            s_fault_latched |= SAFE_FAULT_ARMED_LINK_LOSS;
            s_host_arm_req = false;
            s_rc_arm_req   = false;
            s_rc_loss_active = false;
            s_state = SAFETY_DISARMED;
            Safety_Log(1u, "SAFETY: DISARMED ctrl lost");
        } else if (!rc_alive) {
            /* RC 断但姿态/控制链健康：先保持 250 ms，再进 FAILSAFE 缓降 */
            if (!s_rc_loss_active) {
                s_rc_loss_active   = true;
                s_rc_loss_start_us = now;
            } else if (Fc_AgeUs(s_rc_loss_start_us, now) >= SAFE_RC_FAILSAFE_US) {
                s_rc_loss_active    = false;
                s_failsafe_start_us = now;
                s_state = SAFETY_FAILSAFE;
                Safety_Log(1u, "SAFETY: FAILSAFE rc lost");
            }
            /* 未超时：保持 ARMED，allowed 仍为 true，ControlTask 继续输出 */
        } else {
            s_rc_loss_active = false;   /* RC 恢复，清零计时 */
        }
        if ((s_state == SAFETY_ARMED) && tilt_fatal) {
            s_fault_latched |= SAFE_FAULT_TILT_FATAL;
            s_host_arm_req = false;
            s_rc_arm_req   = false;
            s_rc_loss_active = false;
            s_fatal_clear_active = false;
            s_state = SAFETY_FATAL;
            Safety_Log(1u, "SAFETY: FATAL tilt");
        }
        break;

    case SAFETY_FAILSAFE:
        /* 自动缓降着陆：电机仍在自稳输出 + 油门缓降，此状态不拉低电机 */
        if (!arm_req) {
            s_state = SAFETY_DISARMED;
            Safety_Log(0u, "SAFETY: DISARMED (failsafe, host/rc)");
        } else if (!att_ok || !ctl_ok) {
            if (imu_bad) { s_fault_latched |= SAFE_FAULT_IMU_DEGRADED; }
            s_host_arm_req = false;
            s_rc_arm_req   = false;
            s_state = SAFETY_DISARMED;
            Safety_Log(1u, "SAFETY: DISARMED chain lost");
        } else if (rc_alive) {
            /* RC 恢复，继续飞 */
            s_state = SAFETY_ARMED;
            Safety_Log(2u, "SAFETY: RC recovered");
        } else if (Fc_AgeUs(s_failsafe_start_us, now) >= SAFE_FAILSAFE_DISARM_US) {
            s_host_arm_req = false;
            s_rc_arm_req   = false;
            s_state = SAFETY_DISARMED;
            Safety_Log(1u, "SAFETY: DISARMED failsafe timeout");
        }
        break;

    case SAFETY_FATAL:
        /* FATAL 仅由倾角超限（tilt_fatal）触发：飞机已失控翻滚，拉低电机是
         * crash-detection 式的保护，飞机必然坠地。因此退出后不存在“在空中
         * 抢时间重新解锁”的问题——飞机已经在地面，操作者捡回后重新解锁/上电即可。
         * 唯一出口是自动恢复：坠地扶平（倾角回中）+ 姿态链恢复，持续健康后回 DISARMED。 */
        s_host_arm_req = false;
        s_rc_arm_req   = false;

        {
            /* 自动恢复：姿态链健康（隐含 IMU 未降级）+ 倾角回中（飞机已扶平）。
             * 不要求 RC 在线、不要求总线级全健康（磁力计/气压计坏了不阻止退出） */
            const bool fatal_healthy = att_ok && level_ok;
            if (!fatal_healthy) {
                s_fatal_clear_active = false;
            } else if (!s_fatal_clear_active) {
                s_fatal_clear_active   = true;
                s_fatal_clear_start_us = now;
            } else if (Fc_AgeUs(s_fatal_clear_start_us, now) >= SAFE_FATAL_CLEAR_US) {
                s_fatal_clear_active = false;
                s_state = SAFETY_DISARMED;
                Safety_Log(2u, "SAFETY: FATAL cleared");
            }
        }
        break;

    default:
        s_state = SAFETY_DISARMED;
        break;
    }

    /* ---- 授权字 ---- */
    const bool armed   = (s_state == SAFETY_ARMED) || (s_state == SAFETY_FAILSAFE);
    const bool fatal   = (s_state == SAFETY_FATAL);
    const bool allowed = armed && chain_ok;   /* FAILSAFE 下 chain_ok 健康 → 允许自稳输出 */
    PublishWord(allowed, armed, fatal, chain_ok);

    /* ---- 看门狗：只检测 SafetyTask 是否仍在运行，不承担故障动作 ----
     * 电机输出由 Safety_IsAllowed()/Motor 门控决定；任何状态都喂狗。 */
    (void)HAL_IWDG_Refresh(&hiwdg);
}

SafetyState Safety_GetState(void)     { return s_state; }
uint32_t    Safety_GetFaultMask(void) { return s_fault_latched; }
uint32_t    Safety_GetActiveFaultMask(void) { return s_fault_live; }
void        Safety_RequestArm(bool on){ s_host_arm_req = on; }   /* 上位机源 */
void        Safety_RequestArmRc(bool on){ s_rc_arm_req = on; }    /* RC 源（CH7 电机锁/一键起飞） */
bool        Safety_IsArmed(void)      { return (s_word & SAFE_WORD_ARMED) != 0u; }

bool Safety_IsAllowed(void)
{
#ifdef SAFE_BENCH_FORCE_ARM
    if (s_bench_override) { return true; }
#endif
    return (s_word & SAFE_WORD_ALLOWED) != 0u;
}

#ifdef SAFE_BENCH_FORCE_ARM
void Safety_BenchForceAllowed(bool on) { s_bench_override = on; }
#endif
