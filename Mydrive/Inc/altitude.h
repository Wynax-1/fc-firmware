#ifndef ALTITUDE_H
#define ALTITUDE_H

#include "control_types.h"

void Altitude_Init(void);

/* BusTask 每 1ms 调用，非阻塞推进 BMP180/HC-SR04 状态机 + 融合发布 */
void Altitude_Tick(fc_usec_t now);

/* 取当前高度快照（内部会读 g_chan_altitude） */
bool Altitude_GetSnapshot(AltitudeSnapshot *out);

/* 取超声测高（cm，相对地面标定点）；返回 false 表示超声当前无效 */
bool Altitude_GetUsHeightCm(float *cm);

/* 触发地面标定：以当前超声/气压读数为零点，重置高度为 0 */
void Altitude_CalibrateGround(void);

/* 状态查询（调试） */
uint32_t Altitude_GetUsOkCount(void);
uint32_t Altitude_GetUsErrCount(void);

#endif
