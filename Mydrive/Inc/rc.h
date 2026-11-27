#ifndef RC_H
#define RC_H

#include <stdbool.h>
#include "control_types.h"

typedef enum {
    RC_AIL  = 0, RC_ELE  = 1, RC_THR  = 2, RC_RUD  = 3,
    RC_AUX1 = 4, RC_AUX2 = 5, RC_AUX3 = 6, RC_AUX4 = 7,
} RcChannelIdx;

/* 油门解锁判据（归一化域，随标定宏一起生效，不受端点偏差影响） */
#define RC_THROTTLE_ARM_MAX     0.10f   /* 标准油门：最低位 10% 行程内 */

#ifdef FEATURE_ALT_ASSIST
/* 回中油门：辅助层以"回中=定高/怠速"工作，解锁 = 归一化油门处于回中窗口 */
#define RC_THR_ARM_WIN_LO_N     0.40f
#define RC_THR_ARM_WIN_HI_N     0.60f
#else
/* 标准油门：解锁 = 归一化油门处于最低位 */
#define RC_THR_ARM_WIN_LO_N     0.00f
#define RC_THR_ARM_WIN_HI_N     RC_THROTTLE_ARM_MAX
#endif

/* ---- 通道标定（参考 ArduPilot RCn_MIN/TRIM/MAX/DZ、Betaflight midrc/rxrange）----
 * 标称端点只用于归一化：超出部分由限幅吸收，回中偏差由死区吸收。
 * 若实测回中不是 1500、或端点不是 1000/2000，按实测值改这三个宏即可。 */
#define RC_CAL_MIN_US   1000u
#define RC_CAL_MID_US   1500u
#define RC_CAL_MAX_US   2000u
#define RC_STICK_DZ_ND  0.05f   /* 摇杆回中死区（归一化，满行程的 5%） */

/** 油门归一化 [0,1]（与 rc.c NormThr 同算法，供安全层做 µs 无关判据） */
static inline float Rc_ThrottleNorm(uint16_t us)
{
    float v = (float)((int32_t)us - (int32_t)RC_CAL_MIN_US)
            / (float)((int32_t)RC_CAL_MAX_US - (int32_t)RC_CAL_MIN_US);
    if (v < 0.0f) { v = 0.0f; }
    if (v > 1.0f) { v = 1.0f; }
    return v;
}

/**
 * @brief  遥控归一化快照（摇杆为无量纲域，辅助通道为原始脉宽域）
 * @note   字段名按四轴控制轴，通道→轴的映射在 Rc_Normalize 一次完成：
 *         roll←CH1(AIL)　pitch←CH2(ELE)　thr←CH3(THR)　yaw←CH4(RUD)。
 *         摇杆换算见 rc.c NormStick / NormThr，端点取 RC_CAL_MIN/MID/MAX：
 *         摇杆通道按中点分段线性映射到 [-1,1]，越界限幅，回中死区清零；
 *         油门通道 RC_CAL_MIN~MAX → [0,1]，越界限幅；
 *         辅助通道不归一化：开关/旋钮在 1000~2000µs 原始域直接判档位。
 */
typedef struct {
    float roll;   /**< 横滚杆（CH1）：右杆为正 → 右倾横滚目标 */
    float pitch;  /**< 俯仰杆（CH2）：前推为正；ControlTask 取负作俯仰目标（前推=低头） */
    float thr;    /**< 油门（CH3）：[0,1]；定高辅助开时仅上半程（0.5~1.0）有效（回中=怠速），
                       关闭时全行程 1000~2000µs 直通 */
    float yaw;    /**< 偏航杆（CH4）：右舵为正 → 右偏航速率指令 */
    uint16_t aux_us[4]; /**< AUX1~4（=CH5~CH8）原始脉宽 µs：[0]=CH5 传感器三档（GYRO/IMU/MARG），
                             [1]=CH6 起飞高度旋钮（仅 FEATURE_ALT_ASSIST），
                             [2]=CH7 电机锁/一键起飞两档（两种配置都用），[3] 未分配 */
} RcNorm;

void Rc_Init(void);
void Rc_TaskStep(void);

/* 生产路径：从快照归一化。raw==NULL / raw->valid==false / out==NULL 返回 false。 */
bool Rc_Normalize(const RcCommandSnapshot *raw, RcNorm *out);

/* 以下 getter 保留给调试 / 快速查询用（生产路径不再依赖） */
bool Rc_GetNorm(RcNorm *out);
bool Rc_IsAlive(void);
bool Rc_ThrottleLow(void);

#endif
