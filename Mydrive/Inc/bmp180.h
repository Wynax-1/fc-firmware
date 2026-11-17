#ifndef BMP180_H
#define BMP180_H

#include "control_types.h"


/**
 * @brief BMP180 驱动状态
 * @note   所有状态迁移均由 Bmp_Tick() 管理；
 *         补偿器不保存跨调用的 B5 状态。
 */
typedef enum {
    BMP_ST_CALIB = 0, /**< 读取 22 字节校准数据 */
    BMP_ST_TRIG_T,    /**< 启动温度转换 */
    BMP_ST_WAIT_T,    /**< 等待温度转换完成 */
    BMP_ST_READ_T,    /**< 读取温度原始值 */
    BMP_ST_TRIG_P,    /**< 启动压力转换 */
    BMP_ST_WAIT_P,    /**< 等待压力转换完成 */
    BMP_ST_READ_P,    /**< 读取压力原始值 */
    BMP_ST_ERROR      /**< 错误恢复状态 */
} BmpState;

/**
 * @brief BMP180 有符号校准字
 * @note   字段顺序和类型遵循器件校准存储定义。
 */
typedef struct {
    int16_t AC1; /**< 校准字 AC1 */
    int16_t AC2; /**< 校准字 AC2 */
    int16_t AC3; /**< 校准字 AC3 */
    int16_t AC4; /**< 校准字 AC4，使用无符号语义读取 */
    int16_t AC5; /**< 校准字 AC5，使用无符号语义读取 */
    int16_t AC6; /**< 校准字 AC6，使用无符号语义读取 */
    int16_t B1;  /**< 校准字 B1 */
    int16_t B2;  /**< 校准字 B2 */
    int16_t MB;  /**< 校准字 MB */
    int16_t MC;  /**< 校准字 MC */
    int16_t MD;  /**< 校准字 MD */
} BmpCalib;

/**
 * @brief BMP180 驱动句柄
 * @note   UT/UP 是原始转换值；补偿中间状态不由句柄保存。
 */
typedef struct {
    BmpCalib calib;      /**< 校准字 */
    int32_t  UT;         /**< 未补偿温度原始值 */
    int32_t  UP;         /**< 未补偿压力原始值 */
    uint8_t  oss;        /**< 压力过采样设置，取值范围受器件约束 */
    BmpState state;      /**< 当前驱动状态 */
    uint32_t deadline;   /**< 有界截止时间 */
    uint32_t retry;      /**< 错误重试计数 */
    bool     calib_ok;   /**< true 表示校准字有效 */
} BmpHandle;

/**
 * @brief 执行 BMP180 非阻塞状态机
 * @param  h    : 驱动句柄，输入输出参数
 * @param  out  : 气压快照输出，输入输出参数
 * @param  now  : 当前时间戳
 * @pre    out 必须可写；时间戳由安全时间服务产生
 * @retval 无
 * @note   本函数单次调用最多迁移一个状态；
 *         不承诺固定微秒周期，也不在控制关键路径忙等。
 */
void Bmp_Tick(BmpHandle *h, BaroSnapshot *out, fc_usec_t now);

/**
 * @brief 使用原始温度和压力值执行 BMP180 补偿
 * @param  calib : 校准字，输入参数
 * @param  ut    : 未补偿温度原始值
 * @param  up    : 未补偿压力原始值
 * @param  oss   : 过采样设置
 * @param  press_pa : 补偿后气压输出，单位 Pa
 * @retval 0      补偿成功
 * @retval -1     校准无效、参数越界或数值溢出
 * @note   使用 int64 中间变量，正确处理 UP<B3 与移位量级。
 */
int32_t Bmp_Compensate(const BmpCalib *calib,
                       int32_t ut,
                       int32_t up,
                       uint8_t oss,
                       int32_t *press_pa);

#endif /* BMP180_H */
