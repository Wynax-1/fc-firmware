#ifndef CALIB_H
#define CALIB_H

#include "control_types.h"

/* ==========================================================================
 * 传感器校准（匿名协议 V7 §功能指令 CID 0x01/0x30 + 校准参数 ID 1000~1020）
 * --------------------------------------------------------------------------
 * 工作流：上位机完成校准计算，按组连续下发 0xE2 参数帧：
 *   加速度 1000~1005（零偏 3 + 比例 3）
 *   磁力   1006~1017（软磁矩阵 9 + 硬磁零偏 3）
 *   陀螺   1018~1020（零偏 3）
 * 飞控端 0xE2 写参数只改 RAM；上位机再发 0xE0/CID 0x43 才擦写 Flash。
 *
 * 安全红线：ARMED / FATAL 状态下拒绝一切校准请求（手册 2.19 授权链不动）。
 * 执行上下文：校准计算由上位机完成，飞控端只做参数接收、存储、应用。
 *             结果写 Flash（F401 sector 7 @ 0x08060000），掉电保持。
 *             两步式：0xE2 只写 RAM，0xE0/CID 0x43 才落盘 Flash。
 * ========================================================================== */

/* calib.h —— 参数 ID 唯一权威定义
 * ⚠️ 本项目偏离匿名协议 V7 原文：
 * 1000~1002 加速度零偏 X/Y/Z（g）
 * 1003~1005 加速度比例 X/Y/Z
 * 1006~1017 = 软磁矩阵 9 + 硬磁零偏 3
 * 1018~1020 = 陀螺零偏。
 */
enum {
    CAL_ID_ACC_OFF_X  = 1000, CAL_ID_ACC_OFF_Y, CAL_ID_ACC_OFF_Z,   /* g,    ×1e5 */
    CAL_ID_ACC_SEN_X  = 1003, CAL_ID_ACC_SEN_Y, CAL_ID_ACC_SEN_Z,   /* 1,    ×1e5 */
    CAL_ID_MAG_SI_00  = 1006, /* ... 到 1014：mag_si[0..8] 行优先 */  CAL_ID_MAG_SI_22 = 1014,
    CAL_ID_MAG_OFF_X  = 1015, CAL_ID_MAG_OFF_Y, CAL_ID_MAG_OFF_Z,   /* LSB,  ×1e5 */
    CAL_ID_GYRO_B_X   = 1018, CAL_ID_GYRO_B_Y,  CAL_ID_GYRO_B_Z,    /* rad/s,×1e5 */

    CAL_ID_FIRST = CAL_ID_ACC_OFF_X,
    CAL_ID_LAST  = CAL_ID_GYRO_B_Z,
};

void Calib_Init(void);
void Calib_GyroAutoCal(SensorData *s);
void Calib_Apply(SensorData *s);
bool Calib_GetParam(uint16_t id, int32_t *val);
bool Calib_SetParam(uint16_t id, int32_t val);
void Calib_SaveToFlash(void);

/* 校准原始值模式（上位机采样期间开启，CID 0x44）：
 * 开启后 Calib_Apply 直接返回，0x01/0x02 上报未经校准的原始数据，
 * 避免校准参数在旧参数上"叠加"。参数写入(0xE2)与落盘(0x43)不受影响。 */
void Calib_SetRawMode(bool on);
bool Calib_IsRawMode(void);

bool Calib_IsMagCalibrated(void);
bool Calib_IsAccCalibrated(void);
bool Calib_IsGyroCalibrated(void);

#endif /* CALIB_H */
