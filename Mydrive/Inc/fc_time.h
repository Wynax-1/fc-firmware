#ifndef FC_TIME_H
#define FC_TIME_H

#include "control_types.h"   /* 提供 fc_usec_t */

void Fc_TimeInit(void);

fc_usec_t Fc_GetMicros(void);

uint32_t Fc_AgeUs(fc_usec_t event, fc_usec_t now);

uint32_t Fc_AgeMs(fc_usec_t event, fc_usec_t now);

/**
 * @brief  判断当前时刻是否已到达（或越过）指定截止时刻
 * @param  deadline : 绝对截止时刻
 * @param  now      : 当前时刻
 * @retval true  已到达
 * @retval false 未到达
 * @note   用 int32_t 有符号差分处理 32 位回卷；
 *         deadline 与 now 的时间跨度需小于 2^31 µs（约 35.8 分钟）。
 */
static inline bool Fc_Reached(fc_usec_t deadline, fc_usec_t now)
{
    return ((int32_t)(now - deadline)) >= 0;
}

#endif /* FC_TIME_H */
