#ifndef GY87_H
#define GY87_H

#include "control_types.h"
#include "i2c_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- 设备地址（HAL 要求左移后的 8 位地址） ---------- */
#define GY87_MPU_ADDR   0xD0u   /* 7 位 0x68 */
#define GY87_HMC_ADDR   0x3Cu   /* 7 位 0x1E */

/* ---------- MPU6050 寄存器 ---------- */
#define MPU_REG_SMPLRT_DIV    0x19u
#define MPU_REG_CONFIG        0x1Au
#define MPU_REG_GYRO_CONFIG   0x1Bu
#define MPU_REG_ACCEL_CONFIG  0x1Cu
#define MPU_REG_INT_PIN_CFG   0x37u
#define MPU_REG_USER_CTRL     0x6Au
#define MPU_REG_PWR_MGMT_1    0x6Bu
#define MPU_REG_ACCEL_XOUT_H  0x3Bu
#define MPU_REG_WHO_AM_I      0x75u

/* ---------- HMC5883L 寄存器 ---------- */
#define HMC_REG_CRA     0x00u
#define HMC_REG_CRB     0x01u
#define HMC_REG_MODE    0x02u
#define HMC_REG_DATAX   0x03u

I2cResult GY87_Init(void);

I2cResult GY87_Sample(SensorData *out, fc_usec_t now);

#ifdef __cplusplus
}
#endif
#endif /* GY87_H */
