#include "fc_time.h"
#include "i2c_adapter.h"
#include "bmp180.h"
#include <string.h>

#define BMP180_I2C_ADDR   0xEEu   /* 0x77 << 1 */
#define BMP180_REG_CALIB  0xAAu
#define BMP180_REG_CTRL   0xF4u
#define BMP180_REG_RESULT 0xF6u


static int16_t Be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/**
 * @brief 检查全部 11 个 BMP180 校准字是否有效
 * @param  c : 校准字集合
 * @retval true  校准值未表现出全 0 或全 0xFFFF 的异常
 * @retval false 校准值可疑
 * @note   有符号与无符号校准字按器件定义分别读取。
 */
static bool CalibOk(const BmpCalib *c)
{
    const int16_t *s = (const int16_t *)c;
    const uint16_t *u = (const uint16_t *)c;

    if (c == NULL) return false;

    /* 前 8 个有符号校准字：AC1..B2 */
    for (uint_fast8_t i = 0u; i < 8u; ++i) {
        if ((s[i] == 0) || (s[i] == (int16_t)0xFFFF)) return false;
    }
    /* 后 3 个有符号校准字：MB、MC、MD */
    if ((s[8] == 0) || (s[8] == (int16_t)0xFFFF)) return false;
    if ((s[9] == 0) || (s[9] == (int16_t)0xFFFF)) return false;
    if ((s[10] == 0) || (s[10] == (int16_t)0xFFFF)) return false;

    /* 无符号校准字 AC4/AC5/AC6 不得作为有符号值解释。 */
    for (uint_fast8_t i = 0u; i < 3u; ++i) {
        if ((u[3u + i] == 0u) || (u[3u + i] == 0xFFFFu)) return false;
    }
    return true;
}

/**
 * @brief 执行温度与压力补偿，输出 Pa
 * @param  calib    : 校准字集合
 * @param  ut       : 未补偿温度原始值
 * @param  up       : 未补偿压力原始值
 * @param  oss      : 过采样设置
 * @param  press_pa : 补偿后压力输出
 * @retval 0        成功
 * @retval -1       参数无效或数值越界
 */
int32_t Bmp_Compensate(const BmpCalib *calib,
                       int32_t ut,
                       int32_t up,
                       uint8_t oss,
                       int32_t *press_pa)
{
    int64_t X1, X2, B5, B6, X3, B3, B7;
    uint64_t UP_shifted, B4, denominator;
    int64_t p;
    uint32_t oss_shift;
    const int64_t B7_LIMIT = (int64_t)0x80000000;

    if ((calib == NULL) || (press_pa == NULL)) return -1;
    /* 校准字有效性由调用方句柄的 calib_ok 门控，BmpCalib 本身不携带该标志 */
    if (oss > 3u) return -1;

    /* 温度补偿；避免 MD=0 和 X1+MD=0 造成未定义除法。 */
    X1 = ((int64_t)ut - (int64_t)calib->AC6) * (int64_t)calib->AC5;
    if (calib->MD == 0) return -1;
    if ((X1 + (int64_t)calib->MD) == 0) return -1;
    X2 = ((int64_t)calib->MC << 11) / (X1 + (int64_t)calib->MD);
    B5 = X1 + X2;

    /* 压力补偿；B7 使用 int64，正确处理 UP<B3 的负值。 */
    B6 = B5 - 4000;
    X1 = ((int64_t)calib->B2 * (B6 * B6 / 4096)) / 2048;
    X2 = ((int64_t)calib->AC2 * B6) / 2048;
    X3 = X1 + X2;
    if (calib->AC1 == 0) return -1;
    B3 = (((((int64_t)calib->AC1 * 4) + X3) << oss) + 2) / 4;

    X1 = ((int64_t)calib->AC3 * B6) / 8192;
    X2 = ((int64_t)calib->B1 * ((B6 * B6 / 4096))) / 65536;
    X3 = ((X1 + X2) + 2) / 4;
    B4 = ((uint64_t)calib->AC4 * (uint64_t)(X3 + 32768)) / 32768;
    if (B4 == 0u) return -1;

    oss_shift = (uint32_t)(8u - oss);
    if (oss_shift >= 32u) return -1;
    UP_shifted = (uint64_t)up << oss_shift;

    if (UP_shifted < (uint64_t)B3) {
        B7 = -(int64_t)((uint64_t)B3 - UP_shifted);
    } else {
        B7 = (int64_t)(UP_shifted - (uint64_t)B3);
    }
    if (B7 >= B7_LIMIT) return -1;

    denominator = (uint64_t)B4 * 2u;
    if (denominator == 0u) return -1;
    B7 = (B7 * 50000) / (int64_t)denominator;

    p = (int64_t)(B7 / 256) + 5000;
    X1 = (p / 256) * (p / 256);
    X1 = (X1 * 3038) / 65536;
    X2 = (-7357 * p) / 65536;
    p = p + (X1 + X2 + 3791) / 16;

    if ((p < 300L) || (p > 115500L)) return -1;
    *press_pa = (int32_t)p;
    return 0;
}

/**
 * @brief 执行非阻塞 BMP180 温度、压力状态机
 * @param  h   : 驱动句柄
 * @param  out : 气压快照输出
 * @param  now : 当前时间戳
 * @retval 无
 * @note   校准长度为 22 字节；截止判断使用有界时间差。
 */
void Bmp_Tick(BmpHandle *h, BaroSnapshot *out, fc_usec_t now)
{
    static const uint16_t oss_delay_us[4] = { 4500u, 7500u, 13500u, 25500u };

    if ((h == NULL) || (out == NULL)) { return; }

    switch (h->state) {
    case BMP_ST_CALIB: {
        uint8_t buf[22];
        if (I2C_BusRead(BMP180_I2C_ADDR, BMP180_REG_CALIB, buf, 22, 5) != I2C_OK) {
            h->state = BMP_ST_ERROR;
            break;
        }
        h->calib.AC1 = Be16(&buf[0]);
        h->calib.AC2 = Be16(&buf[2]);
        h->calib.AC3 = Be16(&buf[4]);
        h->calib.AC4 = Be16(&buf[6]);
        h->calib.AC5 = Be16(&buf[8]);
        h->calib.AC6 = Be16(&buf[10]);
        h->calib.B1  = Be16(&buf[12]);
        h->calib.B2  = Be16(&buf[14]);
        h->calib.MB  = Be16(&buf[16]);
        h->calib.MC  = Be16(&buf[18]);
        h->calib.MD  = Be16(&buf[20]);
        h->calib_ok  = CalibOk(&h->calib);
        h->state     = h->calib_ok ? BMP_ST_TRIG_T : BMP_ST_ERROR;
        break;
    }

    case BMP_ST_TRIG_T: {
        const uint8_t cmd = 0x2Eu;   /* 温度转换 */
        if (I2C_BusWrite(BMP180_I2C_ADDR, BMP180_REG_CTRL, &cmd, 1, 5) != I2C_OK) {
            h->state = BMP_ST_ERROR;
            break;
        }
        h->deadline = now + 5000u;
        h->state    = BMP_ST_WAIT_T;
        break;
    }

    case BMP_ST_WAIT_T:
        if (!Fc_Reached(h->deadline, now)) { return; }
        h->state = BMP_ST_READ_T;
        break;

    case BMP_ST_READ_T: {
        uint8_t buf[2];
        if (I2C_BusRead(BMP180_I2C_ADDR, BMP180_REG_RESULT, buf, 2, 5) != I2C_OK) {
            h->state = BMP_ST_ERROR;
            break;
        }
        h->UT    = Be16(buf);
        h->state = BMP_ST_TRIG_P;
        break;
    }

    case BMP_ST_TRIG_P: {
        const uint8_t cmd = (uint8_t)(0x34u | (uint8_t)(h->oss << 6));
        if (I2C_BusWrite(BMP180_I2C_ADDR, BMP180_REG_CTRL, &cmd, 1, 5) != I2C_OK) {
            h->state = BMP_ST_ERROR;
            break;
        }
        h->deadline = now + oss_delay_us[h->oss & 3u];
        h->state    = BMP_ST_WAIT_P;
        break;
    }

    case BMP_ST_WAIT_P:
        if (!Fc_Reached(h->deadline, now)) { return; }
        h->state = BMP_ST_READ_P;
        break;

    case BMP_ST_READ_P: {
        uint8_t buf[3];
        if (I2C_BusRead(BMP180_I2C_ADDR, BMP180_REG_RESULT, buf, 3, 5) != I2C_OK) {
            h->state = BMP_ST_ERROR;
            break;
        }
        const int32_t up_raw = ((int32_t)buf[0] << 16)
                             | ((int32_t)buf[1] << 8)
                             |  (int32_t)buf[2];
        h->UP = up_raw >> (8 - (h->oss & 3u));

        int32_t press_pa;
        if (Bmp_Compensate(&h->calib, h->UT, h->UP, h->oss, &press_pa) == 0) {
            out->pressure_pa = press_pa;
            out->stamp.usec  = now;
            out->stamp.seq  += 1u;
            /* 温度 = B5 / 10^1，需要 Bmp_Compensate 暴露或简化 */
            out->temperature_c = 25.0f;   /* 简化：温度不参与高度计算 */
            out->valid = true;
            h->retry = 0;
        }
        h->state = BMP_ST_TRIG_T;   /* 循环：T → P → T ... */
        break;
    }

    case BMP_ST_ERROR:
        if (++h->retry > 5u) { h->retry = 0; h->state = BMP_ST_CALIB; }
        else                 { h->state = BMP_ST_TRIG_T; }
        I2C_BusRecover();
        break;

    default:
        h->state = BMP_ST_CALIB;
        break;
    }
}
