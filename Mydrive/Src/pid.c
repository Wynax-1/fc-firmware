#include "pid.h"

void Pid_Init(Pid *p, float kp, float ki, float kd,
              float i_lim, float out_lim, float d_alpha)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->i_lim = i_lim;
    p->out_lim = out_lim;
    p->d_lpf_alpha = (d_alpha <= 0.0f) ? 1.0f
                   : (d_alpha > 1.0f) ? 1.0f : d_alpha;
    p->i_term = 0.0f;
    p->d_state = 0.0f;
    p->prev_meas = 0.0f;
    p->inited = false;
}

void Pid_Reset(Pid *p)
{
    p->i_term = 0.0f;
    p->d_state = 0.0f;
    p->prev_meas = 0.0f;
    p->inited = false;
}

void Pid_SetGains(Pid *p, float kp, float ki, float kd)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
}

float Pid_Update(Pid *p, float sp, float meas, float dt)
{
    if (dt <= 0.0f) { return 0.0f; }
    if (!p->inited) { p->prev_meas = meas; p->inited = true; }

    const float err = sp - meas;

    /* 微分作用于测量值，再低通 */
    const float d_meas = (meas - p->prev_meas) / dt;
    p->prev_meas = meas;
    p->d_state += p->d_lpf_alpha * (d_meas - p->d_state);

    /* 候选积分项先限幅，再条件积分：只有"未饱和"或"误差反向（帮助退出饱和）"
     * 时才提交，饱和同向时冻结 i_term，避免回退减掉已被 i_lim 钳位的旧值 */
    float i_new = p->i_term + p->ki * err * dt;
    if (i_new >  p->i_lim) { i_new =  p->i_lim; }
    if (i_new < -p->i_lim) { i_new = -p->i_lim; }

    float out = p->kp * err + i_new - p->kd * p->d_state;

    if (out > p->out_lim) {
        if (err <= 0.0f) { p->i_term = i_new; }
        out = p->out_lim;
    } else if (out < -p->out_lim) {
        if (err >= 0.0f) { p->i_term = i_new; }
        out = -p->out_lim;
    } else {
        p->i_term = i_new;
    }
    return out;
}
