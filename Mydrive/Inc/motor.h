#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_CH_COUNT   4u

/* 通道编号约定：0=CH1(PC6) 1=CH2(PC7) 2=CH3(PC8) 3=CH4(PC9)
 * 物理接线按此顺序对应 M1~M4；若接线不同，在 Motor_Init 前改映射即可 */
typedef enum {
    MOTOR_CH1 = 0,
    MOTOR_CH2 = 1,
    MOTOR_CH3 = 2,
    MOTOR_CH4 = 3,
} MotorChannel;

/* 输出范围（µs），超出会被钳制 */
#define MOTOR_US_MIN   1000u
#define MOTOR_US_MAX   2000u

void     Motor_Init(void);
void     Motor_DisarmAll(void);

/* 门控写入：Safety_IsAllowed() 为 false 时强制 1000µs 并返回 false。
 * 返回 true 表示本次请求被接受（us 已钳制到 [1000,2000]）。 */
bool     Motor_SetThrottle(uint8_t ch, uint16_t us);

/* 只读当前实际写入值（调试/上报用） */
uint16_t Motor_GetThrottle(uint8_t ch);

/* 被门控拒绝的累计次数（调试用） */
uint32_t Motor_GetGatedCount(void);

/* 混控：输入为归一化控制量，各轴范围约 [-1,1]。
 * thr 单位 µs（1000~2000），roll/pitch/yaw 为 PID 输出（已限幅）。
 * 内部对每路结果钳制到 [MOTOR_US_MIN, MOTOR_US_MAX] 后走门控写入。 */
void Motor_Mix(uint16_t thr_us, float roll, float pitch, float yaw);

#endif /* MOTOR_H */
