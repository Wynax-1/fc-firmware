#include "rc.h"
#include "ppm.h"
#include "safety.h"   /* SAFE_RC_STALE_US —— 超时判据的权威定义 */
#include "fc_time.h"
#include "atomic_publish.h"
#include <math.h>
#include <string.h>

static fc_usec_t s_last_ok_us = 0u;
static bool      s_alive      = false;
static uint16_t  s_raw[8];
static RcNorm    s_norm;

static bool      s_pub_inited     = false;
static bool      s_pub_last_valid = false;

static float NormStick(uint16_t us)
{
    /* 以标定中点为界分段线性：中点到两端各占满半程，
     * 端点偏差靠限幅吸收，回中偏差靠死区吸收 */
    float v;
    if ((int32_t)us >= (int32_t)RC_CAL_MID_US) {
        v = (float)((int32_t)us - (int32_t)RC_CAL_MID_US)
          / (float)((int32_t)RC_CAL_MAX_US - (int32_t)RC_CAL_MID_US);
    } else {
        v = (float)((int32_t)us - (int32_t)RC_CAL_MID_US)
          / (float)((int32_t)RC_CAL_MID_US - (int32_t)RC_CAL_MIN_US);
    }
    if (v >  1.0f) { v =  1.0f; }
    if (v < -1.0f) { v = -1.0f; }
    if (fabsf(v) < RC_STICK_DZ_ND) { v = 0.0f; }
    return v;
}

static float NormThr(uint16_t us)
{
    /* 与安全层共用同一归一化（rc.h Rc_ThrottleNorm） */
    return Rc_ThrottleNorm(us);
}

/* ---- 从快照归一化（生产路径）---- */
bool Rc_Normalize(const RcCommandSnapshot *raw, RcNorm *out)
{
    if ((raw == NULL) || (out == NULL) || (!raw->valid)) { return false; }

    out->roll   = NormStick(raw->ch[RC_AIL]);
    out->pitch  = NormStick(raw->ch[RC_ELE]);
    out->thr    = NormThr  (raw->ch[RC_THR]);
    out->yaw    = NormStick(raw->ch[RC_RUD]);
    /* 辅助通道保留原始 µs：开关/旋钮阈值在 1000~2000 域直接判定 */
    out->aux_us[0] = raw->ch[RC_AUX1];
    out->aux_us[1] = raw->ch[RC_AUX2];
    out->aux_us[2] = raw->ch[RC_AUX3];
    out->aux_us[3] = raw->ch[RC_AUX4];
    return true;
}

void Rc_Init(void)
{
    PPM_Init();
    s_alive          = false;
    s_last_ok_us     = 0u;
    s_pub_inited     = false;
    s_pub_last_valid = false;
    memset(s_raw,  0, sizeof(s_raw));
    memset(&s_norm, 0, sizeof(s_norm));
}

void Rc_TaskStep(void)
{
    uint16_t  raw[PPM_CH_COUNT];
    fc_usec_t ts;

    const bool got_new = PPM_GetFrame(raw, &ts);

    if (got_new) {
        s_last_ok_us = ts;
        s_alive      = true;

        for (uint8_t i = 0u; i < 8u; ++i) { s_raw[i] = raw[i]; }

        /* 用局部变量组装完一次赋值，缩短撕裂窗口（虽然生产路径已改用快照，
         * 但调试用 getter 仍读 s_norm，保持一致性） */
        RcNorm tmp;
        tmp.roll   = NormStick(raw[RC_AIL]);
        tmp.pitch  = NormStick(raw[RC_ELE]);
        tmp.thr    = NormThr  (raw[RC_THR]);
        tmp.yaw    = NormStick(raw[RC_RUD]);
        tmp.aux_us[0] = raw[RC_AUX1];
        tmp.aux_us[1] = raw[RC_AUX2];
        tmp.aux_us[2] = raw[RC_AUX3];
        tmp.aux_us[3] = raw[RC_AUX4];
        s_norm = tmp;
    }

    const fc_usec_t now = Fc_GetMicros();
    if (s_alive && (Fc_AgeUs(s_last_ok_us, now) > SAFE_RC_STALE_US)) {
        s_alive = false;
        memset(&s_norm, 0, sizeof(s_norm));
    }

    const bool need_pub = got_new
                       || (s_alive != s_pub_last_valid)
                       || !s_pub_inited;
    if (!need_pub) { return; }

    RcCommandSnapshot snap;
    memset(&snap, 0, sizeof(snap));
    snap.valid = s_alive;
    if (s_alive) {
        for (uint8_t i = 0u; i < 8u; ++i) { snap.ch[i] = s_raw[i]; }
    }
    FcChan_Publish(&g_chan_rc, &snap, now);

    s_pub_last_valid = s_alive;
    s_pub_inited     = true;
}

/* ---- 以下 getter 仅调试用 ---- */
bool Rc_GetNorm(RcNorm *out)
{
    if (!s_alive) { return false; }
    if (out != NULL) { *out = s_norm; }
    return true;
}

bool Rc_IsAlive(void)    { return s_alive; }
/* 油门解锁位判据（与 safety.c 的 µs 域判据等价；窗口随 FEATURE_ALT_ASSIST 切换） */
bool Rc_ThrottleLow(void){ return s_alive
                            && (s_norm.thr >= RC_THR_ARM_WIN_LO_N)
                            && (s_norm.thr <= RC_THR_ARM_WIN_HI_N); }
