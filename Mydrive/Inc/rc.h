#ifndef RC_H
#define RC_H

#include <stdbool.h>
#include "control_types.h"

typedef enum {
    RC_AIL  = 0, RC_ELE  = 1, RC_THR  = 2, RC_RUD  = 3,
    RC_AUX1 = 4, RC_AUX2 = 5, RC_AUX3 = 6, RC_AUX4 = 7,
} RcChannelIdx;

/* 油门解锁上限（归一化）：0.10 => 1100µs。两个宏保持一致 */
#define RC_THROTTLE_ARM_MAX     0.10f
#define RC_THROTTLE_ARM_MAX_US  1100u   /* = 1000 + RC_THROTTLE_ARM_MAX * 1000 */

/* 回中油门解锁窗口：油门杆为回中摇杆，静止在 1500µs，
 * 解锁判据改为"处于回中窗口内"而非"低于下限" */
#define RC_THR_ARM_WIN_LO_US    1400u
#define RC_THR_ARM_WIN_HI_US    1600u
#define RC_THR_ARM_WIN_LO_N     0.40f   /* 归一化域等价判据（调试 getter 用） */
#define RC_THR_ARM_WIN_HI_N     0.60f

typedef struct {
    float ail;    /* [-1, 1] */
    float ele;    /* [-1, 1] */
    float thr;    /* [0, 1]  */
    float rud;    /* [-1, 1] */
    float aux[4]; /* [-1, 1] */
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
