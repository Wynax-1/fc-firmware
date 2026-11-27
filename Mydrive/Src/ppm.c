#include "ppm.h"
#include "hcsr04.h"
#include "tim.h"
#include "main.h"
#include "fc_time.h"
#include <string.h>

/* ==========================================================================
 * PPM 解析（TIM2_CH1 上升沿捕获）
 * --------------------------------------------------------------------------
 * PPM 是"脉冲位置调制"：通道值由相邻两个上升沿的时间差（位置）决定，
 * 高电平脉宽本身不携带信息，无需测量，也不做任何脉宽换算：
 *   channel_value = t(rise[n+1]) - t(rise[n])
 * 帧同步：每帧除通道间隔外还有一个超长同步脉冲/间隔，其对应的时间差
 *   明显偏大，超过 PPM_SYNC_MIN_US 即判定一帧结束。
 *
 * 标称通道值 1000~2000µs；本模块只做"是否落在合理范围"的粗筛，
 * 回中/端点偏差交由 rc.c 的标定宏与死区处理（见 rc.h）。
 *
 * 帧结构：N 个通道脉宽之外，每帧还有一个超长的同步脉冲/间隔。上升沿间隔
 *   因此比通道数多一个（8 通道 = 8 个通道间隔 + 1 个同步间隔）；同步间隔被
 *   丢弃、通道间隔全部保留，所以每帧可完整解出 N 个通道（8 通道接收机 8 路齐全）。
 * ========================================================================== */

/* 通道值有效性窗口（上升沿间隔即通道值，标准 1000~2000，留余量） */
#define PPM_CH_MIN_US       800u
#define PPM_CH_MAX_US       2200u
/* 帧同步长间隔（标准 >2.5ms，远大于任何合法通道值） */
#define PPM_SYNC_MIN_US     2500u
#define PPM_SYNC_MAX_US     15000u
/* 满帧才认：R8EF 8 通道，上升沿解码可完整得到 8 路；
 * 少一路说明该帧损坏，整帧丢弃，避免 CH7 等关键通道被误清零 */
#define PPM_CH_MIN_OK       8u

/* CNT 1 MHz，ARR = 0xFFFFFFFF，所有时刻用 CNT 原值 */
static uint32_t s_prev_cnt  = 0u;    /* 上一个上升沿的捕获值 */
static bool     s_have_prev = false; /* 是否已有前一沿（首沿仅建基准） */
static uint8_t  s_ch        = 0u;    /* 当前帧已解码的通道计数 */
static uint16_t s_buf[PPM_CH_COUNT]; /* 当前帧各通道脉宽暂存 */

static uint16_t          s_frame[PPM_CH_COUNT]; /* 最近一帧完整通道值（供外部读取） */
static volatile bool     s_ready    = false;    /* 新一帧已就绪标志 */
static volatile fc_usec_t s_frame_us = 0u;      /* 最近一帧的时间戳 */
static volatile uint32_t          s_frame_cnt = 0u; /* 累计有效帧数 */
static volatile uint32_t          s_err_cnt   = 0u; /* 累计错误帧数 */

/**
 * @brief 上升沿捕获回调核心：用相邻上升沿时间差解码 PPM
 * @note  TIM2_CH1 已在 CubeMX 中配置为仅上升沿捕获（滤波 1），
 *        故本函数每次进入都对应一个通道的起始沿。
 */
static void PpmOnCapture(void)
{
    const uint32_t cap = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);

    /* 首个上升沿只建立时间基准，不产生通道 */
    if (!s_have_prev) {
        s_prev_cnt  = cap;
        s_have_prev = true;
        return;
    }

    /* 无符号差：CNT 自由运行回绕安全 */
    const uint32_t interval = cap - s_prev_cnt;
    s_prev_cnt = cap;

    /* 1) 长间隔 = 帧同步：提交上一帧 */
    if ((interval >= PPM_SYNC_MIN_US) && (interval <= PPM_SYNC_MAX_US)) {
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
        return;
    }

    /* 2) 超长间隔：长时间失联后恢复，丢弃半帧重新同步 */
    if (interval > PPM_SYNC_MAX_US) {
        s_err_cnt++;
        s_ch = 0u;
        return;
    }

    /* 3) 过短/过长间隔或灰区：伪沿破坏了对齐，整帧丢弃重来 */
    if ((interval < PPM_CH_MIN_US) || (interval > PPM_CH_MAX_US)) {
        s_err_cnt++;
        s_ch = 0u;
        return;
    }

    /* 4) 正常通道：上升沿间隔即通道值，直接存入（不扣任何脉宽/间隙） */
    if (s_ch < PPM_CH_COUNT) {
        s_buf[s_ch++] = (uint16_t)interval;
    } else {
        s_err_cnt++;
        s_ch = 0u;   /* 溢出：重新同步 */
    }
}

void PPM_Init(void)
{
    s_prev_cnt      = 0u;
    s_have_prev     = false;
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
        /* PPM（仅上升沿） */
        PpmOnCapture();
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        /* HC-SR04 ECHO（双沿） */
        const uint32_t cap = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
        const bool is_rise = (HAL_GPIO_ReadPin(HC_SR04_ECHO_GPIO_Port,
                                                HC_SR04_ECHO_Pin) == GPIO_PIN_SET);
        Hcsr04_OnCapture(cap, is_rise);
    }
}
