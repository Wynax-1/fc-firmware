#include "fc_time.h"
#include "stm32f4xx_hal.h"

/* 每微秒对应的内核周期数：84 MHz / 1 MHz = 84。
 * 若将来改主频，只需改 SystemCoreClock，本宏自动跟随。 */
#define FC_CYCLES_PER_US (SystemCoreClock / 1000000u)

/** 上一次读取到的 CYCCNT 值 */
static uint32_t s_last_cyc = 0u;
/** 尚未折算成微秒的剩余周期数（0 .. FC_CYCLES_PER_US-1） */
static uint32_t s_rem_cyc  = 0u;
/** 累加得到的微秒计数，71.6 分钟回卷一次 */
static uint32_t s_accum_us = 0u;

/**
 * @brief  初始化 DWT 周期计数器
 * @note   必须在调度器启动前调用一次；重复调用是安全的（会清零计数）。
 */
void Fc_TimeInit(void)
{
    /* 打开调试异常与监视控制寄存器的跟踪位，否则 DWT 不计数 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    s_last_cyc = 0u;
    s_rem_cyc  = 0u;
    s_accum_us = 0u;
}

/**
 * @brief  读取当前微秒时间戳
 * @retval 自 Fc_TimeInit 以来的微秒数；71.6 分钟后回卷
 * @note   可在任务与中断中调用；内部用 PRIMASK 保护累加状态。
 *
 *         回卷安全性：调用者若用 "now - event" 的无符号差分计算年龄，
 *         只要两次调用的间隔小于 71.6 分钟，结果就永远正确。
 */
fc_usec_t Fc_GetMicros(void)
{
    /* 用 PRIMASK 保护：任务与 TIM2 捕获中断都会调用本函数 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t now   = DWT->CYCCNT;
    const uint32_t delta = now - s_last_cyc;  /* 无符号差分，天然处理 32 位回卷 */
    s_last_cyc = now;

    /* 把上次的余数一起折算，避免每次截断导致的系统性漂移 */
    const uint32_t total = delta + s_rem_cyc;
    const uint32_t us    = total / FC_CYCLES_PER_US;
    s_rem_cyc  = total % FC_CYCLES_PER_US;
    s_accum_us += us;

    const fc_usec_t out = s_accum_us;

    __set_PRIMASK(primask);
    return out;
}

/**
 * @brief  计算有界微秒年龄
 * @param  event : 事件发生时的微秒时间戳
 * @param  now   : 当前微秒时间戳
 * @retval 年龄，单位 µs；若结果超过 0x7FFFFFFF（约 35.8 分钟）则饱和到该值
 * @note   饱和是为了让 "从未发生"（event=0）不至于因为无符号回绕变成"很新"。
 */
uint32_t Fc_AgeUs(fc_usec_t event, fc_usec_t now)
{
    /* 无符号减法：即使 now 已经回卷、event 在回卷前，结果依然正确 */
    const uint32_t d = (uint32_t)(now - event);
    /* 若事件"在未来"（例如 event 从未被赋值而保持 0），饱和到一个大值，
     * 让调用者判定为"已过期"，而不是误判为"刚发生"。 */
    return (d > 0x7FFFFFFFu) ? 0x7FFFFFFFu : d;
}

/**
 * @brief  计算有界毫秒年龄
 * @param  event : 事件发生时的微秒时间戳
 * @param  now   : 当前微秒时间戳
 * @retval 年龄，单位 ms（向下取整）
 */
uint32_t Fc_AgeMs(fc_usec_t event, fc_usec_t now)
{
    return Fc_AgeUs(event, now) / 1000u;
}
