#ifndef __MADGWICK_H
#define __MADGWICK_H

#include "stm32f4xx_hal.h"
#include "control_types.h"
#include <math.h>

/* 传感器使用上限：CH5 三档开关选择，实际使用仍受可用性判据约束 */
typedef enum {
    SENS_MODE_GYRO = 0,   /**< 仅陀螺积分（姿态短时保持，长时漂移） */
    SENS_MODE_IMU  = 1,   /**< 陀螺 + 加速度计（6 轴，yaw 漂移） */
    SENS_MODE_MARG = 2,   /**< 九轴（磁力计仍需 IsMagUsable 动态判定） */
} SensorMode;

/* 函数声明 */
void Attitude_Init(void);
bool Attitude_Update(const SensorData *s, float dt_s);
Quaternion_t   Attitude_GetQuaternion(void);
EulerAngles_t  Attitude_GetEulerAngles(void);
/* 最近一次解算是否使用了磁力计 */
bool Attitude_IsMagUsed(void);
void       Attitude_SetSensorMode(SensorMode m);
SensorMode Attitude_GetSensorMode(void);
	
#endif
