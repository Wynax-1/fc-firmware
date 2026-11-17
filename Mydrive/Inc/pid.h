#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float i_lim;        /* 积分项绝对值限幅 */
    float out_lim;      /* 输出绝对值限幅 */
    float d_lpf_alpha;  /* D 低通系数 (0,1]；1=不滤波 */

    /* 内部状态 */
    float i_term;
    float d_state;
    float prev_meas;
    bool  inited;
} Pid;

void  Pid_Init(Pid *p, float kp, float ki, float kd,
               float i_lim, float out_lim, float d_alpha);
void  Pid_Reset(Pid *p);
void  Pid_SetGains(Pid *p, float kp, float ki, float kd);
float Pid_Update(Pid *p, float sp, float meas, float dt);

#endif
