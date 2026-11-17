#include "ppm.h"
#include "hcsr04.h"
#include "tim.h"
#include "main.h"       /* PPM_Pin / PPM_GPIO_Port */
#include "fc_time.h"
#include <string.h>

/* 脉宽有效性窗口：标准 1000~2000 µs，放宽到 [300, 2500] */
#define PPM_PULSE_MIN_US   300u
#define PPM_PULSE_MAX_US   2500u
/* 帧尾同步低电平：R8EF 标准 2.5 ms 起，上限放到 15 ms 防长时间失联 */
#define PPM_SYNC_MIN_US    2500u
#define PPM_SYNC_MAX_US    15000u
/* 至少 N 个通道才认一帧（R8EF 8 通道，允许丢一半） */
#define PPM_CH_MIN_OK      4u

/* CNT 1 MHz，ARR = 0xFFFFFFFF，所有时刻用 CNT 原值 */
static uint32_t s_rise_cnt      = 0u;
static uint32_t s_last_fall_cnt = 0u;
static bool     s_have_fall     = false;
static uint8_t  s_ch            = 0u;
static uint16_t s_buf[PPM_CH_COUNT];

static uint16_t          s_frame[PPM_CH_COUNT];
static volatile bool     s_ready    = false;
static volatile fc_usec_t s_frame_us = 0u;
static volatile uint32_t          s_frame_cnt = 0u;
static volatile uint32_t          s_err_cnt   = 0u;

/**
 * @brief BOTHEDGE 捕获回调核心：读 GPIO 电平判断上升/下降沿
 * @note  tim.c 中 TIM2_CH1 配为 BOTHEDGE，无需切换极性；
 *        读到高电平 = 刚捕获上升沿，低电平 = 刚捕获下降沿。
 */
static void PpmOnCapture(void)
{
    const uint32_t cap = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
    const bool is_rise = (HAL_GPIO_ReadPin(PPM_GPIO_Port, PPM_Pin) == GPIO_PIN_SET);

    if (is_rise) {
        /* 上升沿 = 前一个通道脉冲结束，检查到上次下降沿的间隔 */
        if (s_have_fall) {
            const uint32_t gap = cap - s_last_fall_cnt;
            if ((gap >= PPM_SYNC_MIN_US) && (gap <= PPM_SYNC_MAX_US)) {
                /* 帧尾同步脉冲：提交上一帧 */
                if ((s_ch >= PPM_CH_MIN_OK) && (s_ch <= PPM_CH_COUNT)) {
                    for (uint8_t i = 0u; i < PPM_CH_COUNT; ++i) {
                        s_frame[i] = (i < s_ch) ? s_buf[i] : 0u;
                    }
                    s_frame_us = Fc_GetMicros();
                    s_frame_cnt++;
                    s_ready = true;
                } else {
                    s_err_cnt++;   /* 通道数不合理 */
                }
                s_ch = 0u;
            } else if (gap < PPM_PULSE_MIN_US) {
                s_err_cnt++;       /* 极短抖动，丢弃 */
            } else if (gap > PPM_SYNC_MAX_US) {
							/* 长时间失联后恢复：丢弃半帧，重新同步 */
							s_err_cnt++;
							s_ch = 0u;
						}
            /* 否则 gap 在 [PULSE_MIN, SYNC_MIN) 之间：可能是帧内，
             * 也可能是失联后的过长时间；继续按通道收，等下一个同步 */
        }
        s_rise_cnt  = cap;
        s_have_fall = false;
    } else {
        /* 下降沿：算脉宽 */
        if (s_have_fall) {
            /* 连续两个下降沿之间没有上升沿：异常，重置 */
            s_err_cnt++;
            s_ch = 0u;
        } else {
            const uint32_t width = cap - s_rise_cnt;
            if ((width >= PPM_PULSE_MIN_US) && (width <= PPM_PULSE_MAX_US)) {
                if (s_ch < PPM_CH_COUNT) {
                    s_buf[s_ch++] = (uint16_t)width;
                } else {
                    s_err_cnt++;
                    s_ch = 0u;   /* 溢出：重新同步 */
                }
            } else {
                s_err_cnt++;
            }
        }
        s_last_fall_cnt = cap;
        s_have_fall     = true;
    }
}

void PPM_Init(void)
{
    s_rise_cnt      = 0u;
    s_last_fall_cnt = 0u;
    s_have_fall     = false;
    s_ch            = 0u;
    s_ready         = false;
    s_frame_us      = 0u;
    s_frame_cnt     = 0u;
    s_err_cnt       = 0u;
    memset(s_buf,   0, sizeof(s_buf));
    memset(s_frame, 0, sizeof(s_frame));

    (void)HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

bool PPM_GetFrame(uint16_t out[PPM_CH_COUNT], fc_usec_t *frame_us)
{
    if (!s_ready) { return false; }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(out, (const void *)s_frame, sizeof(s_frame));
    if (frame_us != NULL) { *frame_us = s_frame_us; }
    s_ready = false;
    __set_PRIMASK(primask);
    return true;
}

void PPM_Stats(uint32_t *frame_cnt, uint32_t *err_cnt)
{
    if (frame_cnt != NULL) { *frame_cnt = s_frame_cnt; }
    if (err_cnt   != NULL) { *err_cnt   = s_err_cnt;   }
}

/* ---- HAL 回调入口：确认全工程无同名定义 ---- */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) { return; }

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        /* PPM */
        PpmOnCapture();
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        /* HC-SR04 ECHO */
        const uint32_t cap = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
        const bool is_rise = (HAL_GPIO_ReadPin(HC_SR04_ECHO_GPIO_Port,
                                                HC_SR04_ECHO_Pin) == GPIO_PIN_SET);
        Hcsr04_OnCapture(cap, is_rise);
    }
}
