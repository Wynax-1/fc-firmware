/* altitude.c */
#include "altitude.h"
#include "bmp180.h"
#include "hcsr04.h"
#include "fc_time.h"
#include "atomic_publish.h"
#include <math.h>
#include <string.h>

/* ---------- 阈值 ---------- */
#define ALT_US_MAX_CM        400.0f     /* 超声有效上限 */
#define ALT_US_STALE_US      300000u    /* 300ms 无新超声，判失效 */
#define ALT_BARO_STALE_US    1500000u   /* 1.5s 无新气压，判失效 */
#define ALT_PUBLISH_HZ       50u        /* 发布频率 */
#define ALT_LPF_ALPHA        0.15f      /* 高度一阶低通系数（越大越快，噪声越大）*/
#define ALT_VZ_ALPHA         0.30f      /* 速度一阶低通 */

/* ---------- 内部状态 ---------- */
static BmpHandle        s_bmp;
static BaroSnapshot     s_baro;
static fc_usec_t        s_last_fuse_us  = 0u;

static float            s_us_cm         = 0.0f;
static fc_usec_t        s_us_ts         = 0u;
static bool             s_us_valid      = false;

/* 地面标定 */
static bool             s_ground_calib_done = false;
static float            s_ground_us_cm      = 0.0f;   /* 起飞点超声读数（补偿地面悬停）*/
static float            s_baro_ref_alt_m    = 0.0f;   /* 地面标定时基准海拔（气压计算出）*/

/* 融合输出 */
static float            s_alt_m          = 0.0f;
static float            s_vz_mps         = 0.0f;
static float            s_last_fused_m   = 0.0f;
static uint8_t          s_last_source    = ALT_SRC_NONE;

static fc_usec_t        s_next_pub_us    = 0u;
static bool             s_inited         = false;

/* 标准大气压→高度（近似，仅用于标定平移，不含绝对精度要求） */
static float BaroToAltM(int32_t press_pa)
{
    if (press_pa <= 0) { return 0.0f; }
    /* 国际标准大气压：h = 44330 * (1 - (P/P0)^(1/5.255)) */
    const float ratio = (float)press_pa / 101325.0f;
    return 44330.0f * (1.0f - powf(ratio, 0.190295f));
}

void Altitude_Init(void)
{
    memset(&s_bmp,     0, sizeof(s_bmp));
    memset(&s_baro,    0, sizeof(s_baro));
    s_bmp.oss    = 1;                 /* 过采样 1：7.5ms 转换，平衡速度/精度 */
    s_bmp.state  = BMP_ST_CALIB;

    s_last_fuse_us     = 0u;
    s_us_cm            = 0.0f;
    s_us_ts            = 0u;
    s_us_valid         = false;
    s_ground_calib_done = false;
    s_ground_us_cm      = 0.0f;
    s_baro_ref_alt_m    = 0.0f;
    s_alt_m             = 0.0f;
    s_vz_mps            = 0.0f;
    s_last_fused_m      = 0.0f;
    s_last_source       = ALT_SRC_NONE;
    s_next_pub_us       = 0u;

    Hcsr04_Init();
    s_inited = true;
}

void Altitude_CalibrateGround(void)
{
    /* 用当前超声读数作为地面基准（如果超声不可用则用气压计基准） */
    if (s_us_valid && (Fc_AgeUs(s_us_ts, Fc_GetMicros()) < ALT_US_STALE_US)) {
        s_ground_us_cm = s_us_cm;
    } else {
        s_ground_us_cm = 0.0f;
    }

    if (s_baro.valid) {
        s_baro_ref_alt_m = BaroToAltM(s_baro.pressure_pa);
    }

    s_alt_m          = 0.0f;
    s_vz_mps         = 0.0f;
    s_last_fused_m   = 0.0f;
    s_last_source    = ALT_SRC_NONE;
    s_ground_calib_done = true;
}

void Altitude_Tick(fc_usec_t now)
{
    if (!s_inited) { Altitude_Init(); }

    /* ---- 1. 推进两个传感器状态机 ---- */
    Bmp_Tick(&s_bmp, &s_baro, now);   /* 新样本时刻由 bmp180.c 写入 s_baro.stamp.usec */

    Hcsr04_Tick(now);

    float cm;
    fc_usec_t us_ts;
    if (Hcsr04_GetDistance(&cm, &us_ts)) {
        s_us_cm    = cm;
        s_us_ts    = us_ts;
        s_us_valid = true;
    }

    /* ---- 2. 地面标定（首次上电 3 秒后自动执行一次）---- */
    if (!s_ground_calib_done && (Fc_AgeUs(0u, now) > 3000000u)) {
        Altitude_CalibrateGround();
    }

    /* ---- 3. 传感器有效性 ---- */
    const bool baro_ok = s_baro.valid
                      && (Fc_AgeUs(s_baro.stamp.usec, now) <= ALT_BARO_STALE_US);
    const bool us_ok   = s_us_valid
                      && (Fc_AgeUs(s_us_ts, now) <= ALT_US_STALE_US)
                      && (s_us_cm <= ALT_US_MAX_CM);

    /* ---- 4. 融合 ---- */
    float new_alt = s_alt_m;
    uint8_t new_src = ALT_SRC_NONE;

    if (us_ok) {
        /* 超声：直接读米 */
        new_alt = (s_us_cm - s_ground_us_cm) / 100.0f;
        new_src = ALT_SRC_US;
    } else if (baro_ok) {
        /* 气压计：相对起飞点的海拔差 */
        const float cur_alt = BaroToAltM(s_baro.pressure_pa);
        new_alt = cur_alt - s_baro_ref_alt_m;
        new_src = ALT_SRC_BARO;
    }

    /* 切换源时保持连续（避免超声↔气压计切换导致高度跳变） */
    if ((new_src != s_last_source) && (s_last_source != ALT_SRC_NONE)
        && (new_src != ALT_SRC_NONE)) {
        /* 用当前输出作为新源的起点 */
        s_last_fused_m = s_alt_m;
        new_alt        = s_alt_m;
    }

    /* 一阶低通 */
    s_alt_m += ALT_LPF_ALPHA * (new_alt - s_alt_m);

    /* ---- 5. 垂直速度（对高度差分）---- */
    if (s_last_fuse_us != 0u) {
        const float dt_s = (float)Fc_AgeUs(s_last_fuse_us, now) * 1e-6f;
        if (dt_s > 0.001f && dt_s < 0.2f) {
            const float vz_raw = (s_alt_m - s_last_fused_m) / dt_s;
            s_vz_mps += ALT_VZ_ALPHA * (vz_raw - s_vz_mps);
        }
    }
    s_last_fuse_us = now;
    s_last_fused_m = s_alt_m;
    s_last_source  = new_src;

    /* ---- 6. 发布 ---- */
    if ((s_next_pub_us == 0u) || Fc_Reached(s_next_pub_us, now)) {
        AltitudeSnapshot snap;
        memset(&snap, 0, sizeof(snap));
        snap.height_m = s_alt_m;
        snap.vz_mps   = s_vz_mps;
        snap.source   = new_src;
        snap.valid    = (new_src != ALT_SRC_NONE);
        FcChan_Publish(&g_chan_altitude, &snap, now);
        s_next_pub_us = now + (1000000u / ALT_PUBLISH_HZ);
    }
}

bool Altitude_GetSnapshot(AltitudeSnapshot *out)
{
    if (out == NULL) { return false; }
    return FcChan_Read(&g_chan_altitude, out);
}

/**
 * @brief  取超声测高（相对地面标定点）
 * @param  cm : 输出高度，单位 cm
 * @retval true  超声当前有效
 * @retval false 超声无新样本/超量程/cm 为空
 * @note   与融合使用的判据一致（ALT_US_STALE_US、ALT_US_MAX_CM）。
 */
bool Altitude_GetUsHeightCm(float *cm)
{
    if (cm == NULL) { return false; }

    const fc_usec_t now = Fc_GetMicros();
    const bool ok = s_us_valid
                 && (Fc_AgeUs(s_us_ts, now) <= ALT_US_STALE_US)
                 && (s_us_cm <= ALT_US_MAX_CM);
    if (!ok) { return false; }

    *cm = s_us_cm - s_ground_us_cm;
    return true;
}

uint32_t Altitude_GetUsOkCount(void)  { return Hcsr04_GetOkCount(); }
uint32_t Altitude_GetUsErrCount(void) { return Hcsr04_GetErrCount(); }
