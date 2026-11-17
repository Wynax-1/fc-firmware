#include "safety.h"
#include "fc_time.h"
#include "i2c_adapter.h"
#include "iwdg.h"
#include "rc.h"            /* RC_THR 枚举、RC_THR_ARM_WIN_*_US 回中窗口 */
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

static SafetyState s_state   = SAFETY_DISARMED;
static fc_usec_t   s_boot_us = 0u;
static uint32_t    s_fault   = 0u;
static bool        s_inited  = false;

static volatile bool s_host_arm_req = false;
static volatile bool s_rc_arm_req   = false;   /* RC 源（CH7 一键起飞）解锁请求 */

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
    s_boot_us      = Fc_GetMicros();
    s_fault        = 0u;
    s_host_arm_req = false;
    s_rc_arm_req   = false;
    s_tilt_active  = false;
    s_arming_start_us = 0u;
    s_tilt_start_us   = 0u;
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
    /* 回中油门：解锁判据 = 杆处于回中窗口（静止 1500µs），而非低于下限 */
    const bool thr_low  = rc_alive
                       && (rc->ch[RC_THR] >= RC_THR_ARM_WIN_LO_US)
                       && (rc->ch[RC_THR] <= RC_THR_ARM_WIN_HI_US);
    if (!rc_alive) { s_fault |= SAFE_FAULT_RC_STALE; }

    /* ---- 数据年龄与即时健康 ---- */
    const uint32_t att_age = ((att != NULL) && att->valid)
                             ? Fc_AgeUs(att->stamp.usec, now) : 0xFFFFFFFFu;
    const uint32_t ctl_age = (ctrl != NULL)
                             ? Fc_AgeUs(ctrl->stamp.usec, now) : 0xFFFFFFFFu;

    const bool att_ok   = (att_age <= SAFE_ATT_STALE_US);
    const bool ctl_ok   = (ctl_age <= SAFE_CTRL_STALE_US);
    const bool chain_ok = att_ok && ctl_ok;
    const bool i2c_bad  = I2C_AuditDegraded();
    const bool level_ok = AttitudeLevel(att);

    if (!att_ok)  { s_fault |= SAFE_FAULT_ATT_STALE;  }
    if (!ctl_ok)  { s_fault |= SAFE_FAULT_CTRL_STALE; }
    if (i2c_bad)  { s_fault |= SAFE_FAULT_I2C_DEGRADED; }

    /* ---- 倾斜超限 ---- */
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
    /* 解锁双源：上位机 0x01/0x02 与 CH7 逻辑或；失锁路径各自撤销（见下） */
    const bool arm_req = s_host_arm_req || s_rc_arm_req;
    switch (s_state) {
    case SAFETY_DISARMED:
        if (arm_req && chain_ok && level_ok && !i2c_bad
            && rc_alive && thr_low) {
            s_state           = SAFETY_ARM_PENDING;
            s_arming_start_us = now;
            Safety_Log(0u, "SAFETY: ARMING");
        }
        break;

    case SAFETY_ARM_PENDING:
        if (!arm_req || !chain_ok || !level_ok || i2c_bad
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
            Safety_Log(0u, "SAFETY: DISARMED by host/rc");
        } else if (!rc_alive) {
            /* 记录"ARMED 状态下丢失 RC"这一历史严重事件 */
            s_fault |= SAFE_FAULT_ARMED_LINK_LOSS;
            s_host_arm_req = false;   /* 需上位机重新解锁，防链路恢复后自动起转 */
            s_rc_arm_req   = false;
            s_state  = SAFETY_DISARMED;
            Safety_Log(1u, "SAFETY: DISARMED rc lost");
        } else if (!chain_ok) {
            /* 数据链短暂中断只关油门，不升级 FATAL：
             * 50 ms 抖动就让飞控复位的代价远大于收益 */
            s_host_arm_req = false;
            s_rc_arm_req   = false;
            s_state = SAFETY_DISARMED;
            Safety_Log(1u, "SAFETY: DISARMED link lost");
        } else if (i2c_bad) {
            s_state = SAFETY_FATAL;
            Safety_Log(1u, "SAFETY: FATAL i2c degraded");
        } else if (tilt_fatal) {
            s_state = SAFETY_FATAL;
            Safety_Log(1u, "SAFETY: FATAL tilt");
        }
        break;

    case SAFETY_FATAL:
        break;

    default:
        s_state = SAFETY_DISARMED;
        break;
    }

    /* ---- 授权字 ---- */
    const bool armed   = (s_state == SAFETY_ARMED);
    const bool fatal   = (s_state == SAFETY_FATAL);
    const bool allowed = armed && chain_ok;
    PublishWord(allowed, armed, fatal, chain_ok);

    /* ---- 看门狗门控 ---- */
    const bool in_grace = (Fc_AgeUs(s_boot_us, now) < SAFE_IWDG_GRACE_US);
    const bool feed = in_grace
                   || (s_state == SAFETY_DISARMED)
                   || (s_state == SAFETY_ARM_PENDING)
                   || (s_state == SAFETY_ARMED);
    if (feed) { (void)HAL_IWDG_Refresh(&hiwdg); }
}

SafetyState Safety_GetState(void)     { return s_state; }
uint32_t    Safety_GetFaultMask(void) { return s_fault; }
void        Safety_RequestArm(bool on){ s_host_arm_req = on; }   /* 上位机源 */
void        Safety_RequestArmRc(bool on){ s_rc_arm_req = on; }    /* RC 源（CH7） */
bool        Safety_IsArmed(void)      { return (s_word & SAFE_WORD_ARMED) != 0u; }

void Safety_ClearFatal(void)
{
    if (s_state == SAFETY_FATAL) { s_state = SAFETY_DISARMED; }
    s_fault            = 0u;
    s_host_arm_req     = false;
    s_rc_arm_req       = false;
    s_tilt_active      = false;
    s_arming_start_us  = 0u;
    s_tilt_start_us    = 0u;
    PublishWord(false, false, false, false);
}

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
