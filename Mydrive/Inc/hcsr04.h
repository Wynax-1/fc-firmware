#ifndef HCSR04_H
#define HCSR04_H

#include <stdint.h>
#include <stdbool.h>
#include "control_types.h"

/* 启动 TIM2_CH2 输入捕获；PPM 的 CH1 由 Rc_Init 独立启动 */
void Hcsr04_Init(void);

/* BusTask 每 1ms 调用一次，非阻塞推进状态机 */
void Hcsr04_Tick(fc_usec_t now);

/* 取最新一次有效距离（cm）与时刻。无新数据返回 false。 */
bool Hcsr04_GetDistance(float *out_cm, fc_usec_t *out_ts);

/* 供 ppm.c 的 HAL_TIM_IC_CaptureCallback 转发 CH2 中断 */
void Hcsr04_OnCapture(uint32_t cap_cnt, bool is_rise);

uint32_t Hcsr04_GetOkCount(void);
uint32_t Hcsr04_GetErrCount(void);

#endif
