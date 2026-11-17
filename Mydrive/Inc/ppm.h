#ifndef PPM_H
#define PPM_H

#include <stdint.h>
#include <stdbool.h>
#include "control_types.h"

#define PPM_CH_COUNT          8u

void PPM_Init(void);

/* 取一帧已解好的 PPM：成功返回 true 并清 ready 标志；无新帧返回 false。 */
bool PPM_GetFrame(uint16_t out[PPM_CH_COUNT], fc_usec_t *frame_us);

void PPM_Stats(uint32_t *frame_cnt, uint32_t *err_cnt);

#endif
