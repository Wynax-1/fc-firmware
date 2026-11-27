#ifndef I2C_ADAPTER_H
#define I2C_ADAPTER_H

#include "i2c.h"
#include "control_types.h"

/**
 * @brief  I²C 事务结果
 * @note   不把未知总线错误伪装成 NACK：两者对应的恢复策略不同。
 */
typedef enum {
    I2C_OK = 0,      /**< 成功 */
    I2C_BUSY,        /**< 队列满或外设忙 */
    I2C_TIMEOUT,     /**< 等待完成超时 */
    I2C_NACK,        /**< 从设备无应答 */
    I2C_BUS_FAULT    /**< 其他总线错误（BERR/ARLO/OVR 等） */
} I2cResult;

I2cResult I2C_BusRead(uint16_t dev8, uint8_t reg, uint8_t *buf,
                      uint16_t len, uint32_t timeout_ms);

I2cResult I2C_BusWrite(uint16_t dev8, uint8_t reg, const uint8_t *val,
                       uint16_t len, uint32_t timeout_ms);

I2cResult I2C_QueueSubmit(uint16_t dev8, uint8_t reg, uint8_t val);

void I2C_QueueDrain(void);

bool I2C_AuditDegraded(void);

/**
 * @brief  查询某个从设备是否处于降级状态
 * @param  dev8 左移后的 8 位设备地址（MPU6050 = 0xD0）
 * @retval true 该设备连续失败达到阈值，当前降级
 * @note   与 I2C_AuditDegraded()（总线级，任一设备降级即 true）不同，
 *         本函数用于安全决策：只有关键设备才应触发锁存。
 *         只读，任意任务可调用，最坏读到上一拍旧值。
 */
bool I2C_AuditDeviceDegraded(uint16_t dev8);

void I2C_BusRecover(void);

#endif /* I2C_ADAPTER_H */
