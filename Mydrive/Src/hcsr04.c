/* hcsr04.c */
#include "hcsr04.h"
#include "tim.h"
#include "main.h"        /* HC_SR04_TRIG_Pin / HC_SR04_TRIG_GPIO_Port */
#include "fc_time.h"
#include <string.h>

/* TRIG 引脚没定义则编译期报错，提醒用户补 CubeMX 配置 */
#ifndef HC_SR04_TRIG_Pin
#error "HC_SR04_TRIG_Pin not defined — 请在 CubeMX 里给 HC-SR04 的 TRIG 分配一个 GPIO 输出"
#endif
#ifndef HC_SR04_ECHO_Pin
#error "HC_SR04_ECHO_Pin not defined"
#endif

#define HCSR04_TRIG_US        15u      /* TRIG 高电平时长，>= 10µs */
#define HCSR04_PERIOD_US      100000u  /* 100ms 触发一次，给足 40ms 转换时间 */
#define HCSR04_ECHO_TIMEOUT_US 30000u  /* 30ms 无回波判失败 */
#define HCSR04_DIST_MIN_CM    2.0f
#define HCSR04_DIST_MAX_CM    400.0f

typedef enum {
    HS_IDLE = 0,
    HS_TRIG_HIGH,
    HS_WAIT_ECHO,      /* 已发 TRIG，等 ECHO 上升沿 */
    HS_WAIT_FALL,      /* ECHO 上升沿已到，等下降沿 */
} Hcsr04State;

static Hcsr04State s_state       = HS_IDLE;
static fc_usec_t   s_next_trig   = 0u;
static fc_usec_t   s_trig_start  = 0u;
static fc_usec_t   s_echo_rise   = 0u;   /* 上升沿的 Fc_GetMicros 时刻 */
static uint32_t    s_echo_rise_cnt = 0u; /* 上升沿的 TIM2 CNT（用于算宽度） */
static bool        s_have_rise_cnt = false;

static uint32_t    s_dist_width_us = 0u;   /* 原始回波脉宽（µs），浮点换算在任务侧 */
static fc_usec_t   s_dist_ts     = 0u;
static bool        s_dist_ready  = false;
static uint32_t    s_err_cnt     = 0u;
static uint32_t    s_ok_cnt      = 0u;

void Hcsr04_Init(void)
{
    s_state       = HS_IDLE;
    s_next_trig   = 0u;
    s_dist_ready  = false;
    s_err_cnt     = 0u;
    s_ok_cnt      = 0u;
    s_have_rise_cnt = false;

    HAL_GPIO_WritePin(HC_SR04_TRIG_GPIO_Port, HC_SR04_TRIG_Pin, GPIO_PIN_RESET);
    (void)HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}

void Hcsr04_OnCapture(uint32_t cap_cnt, bool is_rise)
{
    if (s_state != HS_WAIT_ECHO && s_state != HS_WAIT_FALL) { return; }

    if (is_rise && s_state == HS_WAIT_ECHO) {
        s_echo_rise     = Fc_GetMicros();
        s_echo_rise_cnt = cap_cnt;
        s_have_rise_cnt = true;
        s_state         = HS_WAIT_FALL;
    } else if (!is_rise && s_state == HS_WAIT_FALL && s_have_rise_cnt) {
        /* 用 TIM2 CNT 差值算宽度（同一时钟源，最准确） */
        const uint32_t width_us = cap_cnt - s_echo_rise_cnt;
        s_have_rise_cnt = false;
        s_state = HS_IDLE;

        /* ISR 只存原始脉宽+时间戳+就绪标志；浮点换算与距离范围判定移到任务侧
         * Hcsr04_GetDistance。此处仅做整数脉宽合理性筛（100~25000µs）。 */
        if (width_us >= 100u && width_us <= 25000u) {
            s_dist_width_us = width_us;
            s_dist_ts       = s_echo_rise;
            s_dist_ready    = true;
        } else {
            s_err_cnt++;
        }
    }
}

void Hcsr04_Tick(fc_usec_t now)
{
    switch (s_state) {
    case HS_IDLE:
        if ((s_next_trig == 0u) || Fc_Reached(s_next_trig, now)) {
            HAL_GPIO_WritePin(HC_SR04_TRIG_GPIO_Port, HC_SR04_TRIG_Pin, GPIO_PIN_SET);
            s_trig_start = now;
            s_next_trig  = now + HCSR04_PERIOD_US;
            s_state      = HS_TRIG_HIGH;
        }
        break;

    case HS_TRIG_HIGH:
        if (Fc_Reached(s_trig_start + HCSR04_TRIG_US, now)) {
            HAL_GPIO_WritePin(HC_SR04_TRIG_GPIO_Port, HC_SR04_TRIG_Pin, GPIO_PIN_RESET);
            s_state = HS_WAIT_ECHO;
        }
        break;

    case HS_WAIT_ECHO:
        /* ECHO 超时：距离太远或没接 */
        if (Fc_Reached(s_trig_start + HCSR04_ECHO_TIMEOUT_US, now)) {
            s_err_cnt++;
            s_state = HS_IDLE;
        }
        break;

    case HS_WAIT_FALL:
        /* 下降沿超时：用已收到的上升沿宽度算个上限，或直接放弃 */
        if (Fc_Reached(s_echo_rise + HCSR04_ECHO_TIMEOUT_US, now)) {
            s_have_rise_cnt = false;
            s_err_cnt++;
            s_state = HS_IDLE;
        }
        break;

    default:
        s_state = HS_IDLE;
        break;
    }
}

bool Hcsr04_GetDistance(float *out_cm, fc_usec_t *out_ts)
{
    /* PRIMASK 保护：与 TIM2 捕获中断互斥，原子完成"读脉宽+读时刻+清就绪" */
    uint32_t  width_us;
    fc_usec_t ts;
    bool      ready;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ready    = s_dist_ready;
    width_us = s_dist_width_us;
    ts       = s_dist_ts;
    s_dist_ready = false;
    __set_PRIMASK(primask);

    if (!ready) { return false; }

    /* 浮点换算与距离范围判定在任务侧完成（width_us = 距离(cm) × 58） */
    const float cm = (float)width_us / 58.0f;
    if ((cm >= HCSR04_DIST_MIN_CM) && (cm <= HCSR04_DIST_MAX_CM)) {
        s_ok_cnt++;
        if (out_cm != NULL) { *out_cm = cm; }
        if (out_ts != NULL) { *out_ts = ts; }
        return true;
    }
    s_err_cnt++;
    return false;
}

uint32_t Hcsr04_GetOkCount(void)  { return s_ok_cnt; }
uint32_t Hcsr04_GetErrCount(void) { return s_err_cnt; }
