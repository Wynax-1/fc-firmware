#include "motor.h"
#include "tim.h"        /* 提供 htim3，需 CubeMX 已生成 TIM3 句柄 */
#include "safety.h"
#include <string.h>


/* 混控系数：四轴位移量（µs）最大值。PID 输出 ±1 对应 ±MIX_GAIN_US。
 * 调参期先用小值（如 200µs），避免地面测试时电机猛冲。 */
#define MIX_GAIN_US  200.0f

static uint16_t s_throttle[MOTOR_CH_COUNT];
static uint32_t s_gated_cnt = 0u;

/**
 * @brief 写单通道比较值（内部函数，不查门控）
 * @note  TIM3 配置为 1MHz 计数时，CCR 数值 == 脉宽 µs。
 */
static void WriteChannel(uint8_t ch, uint16_t us)
{
    if (us < MOTOR_US_MIN) { us = MOTOR_US_MIN; }
    if (us > MOTOR_US_MAX) { us = MOTOR_US_MAX; }

    switch (ch) {
    case MOTOR_CH1: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us); break;
    case MOTOR_CH2: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, us); break;
    case MOTOR_CH3: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, us); break;
    case MOTOR_CH4: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, us); break;
    default: return;
    }
    s_throttle[ch] = us;
}

/**
 * @brief 初始化：先把四路 CCR 写成 1000µs，再启动 PWM 输出
 * @note  顺序很重要：先写 CCR 再 Start，保证电调看到的第一个有效脉宽
 *        就是 1000µs（最小油门），而不是 AF 切换瞬间的不确定电平。
 *        必须在调度器启动前、Safety_Init() 之后调用。
 */
void Motor_Init(void)
{
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; ++i) {
        WriteChannel(i, MOTOR_US_MIN);
    }
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

/**
 * @brief 全部拉最低油门（1000µs）
 * @note  不受门控影响：这是"关"，永远允许。
 *        也是 SafetyTask 每拍的兜底动作，防止 ControlTask 卡死时电机保持高转。
 */
void Motor_DisarmAll(void)
{
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; ++i) {
        WriteChannel(i, MOTOR_US_MIN);
    }
}

/**
 * @brief 门控写入单通道油门
 * @retval true  请求被接受（已写入，us 钳制到 [1000,2000]）
 * @retval false 被门控拒绝（ch 非法，或 Safety_IsAllowed()==false），
 *               该通道已被强制拉回 1000µs
 * @note  这一层检查刻意放在驱动里，而不是让调用方自觉：
 *        ControlTask 的任何一个 bug 都不应能让桨在不允许的状态下转。
 */
bool Motor_SetThrottle(uint8_t ch, uint16_t us)
{
    if (ch >= MOTOR_CH_COUNT) {
        return false;
    }
    if (!Safety_IsAllowed()) {
        s_gated_cnt++;
        WriteChannel(ch, MOTOR_US_MIN);
        return false;
    }
    WriteChannel(ch, us);
    return true;
}

uint16_t Motor_GetThrottle(uint8_t ch)
{
    return (ch < MOTOR_CH_COUNT) ? s_throttle[ch] : 0u;
}

uint32_t Motor_GetGatedCount(void)
{
    return s_gated_cnt;
}

void Motor_Mix(uint16_t thr_us, float roll, float pitch, float yaw)
{
	/* 标准 X 型（俯视，箭头朝前）：
	 *   M1(前左, CW)   M2(前右, CCW)
	 *   M3(后左, CCW)  M4(后右, CW)
	 *
	 * Roll + : 左侧加力、右侧减力（右滚为正）
	 * Pitch+: 机头加力、机尾减力（抬头为正）
	 * Yaw + : CCW（M2/M3）加力，CW（M1/M4）减力
	 *
	 * 若你的接线/桨型不同，改下面四行的符号即可 */
	float m1 = (float)thr_us + ( roll + pitch - yaw) * MIX_GAIN_US;
	if (m1 < 0.0f) {m1 = 0.0f;}
	float m2 = (float)thr_us + (-roll + pitch + yaw) * MIX_GAIN_US;
	if (m2 < 0.0f) {m2 = 0.0f;}
	float m3 = (float)thr_us + ( roll - pitch + yaw) * MIX_GAIN_US;
	if (m3 < 0.0f) {m3 = 0.0f;}
	float m4 = (float)thr_us + (-roll - pitch - yaw) * MIX_GAIN_US;
	if (m4 < 0.0f) {m4 = 0.0f;}
	
	(void)Motor_SetThrottle(MOTOR_CH1, (uint16_t)m1);
	(void)Motor_SetThrottle(MOTOR_CH2, (uint16_t)m2);
	(void)Motor_SetThrottle(MOTOR_CH3, (uint16_t)m3);
	(void)Motor_SetThrottle(MOTOR_CH4, (uint16_t)m4);
}
