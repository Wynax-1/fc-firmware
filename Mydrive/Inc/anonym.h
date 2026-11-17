#ifndef __ANONYM_H
#define __ANONYM_H

#include "control_types.h"

typedef struct {
    int32_t  mode;
    int32_t  throttle_us;
    int32_t  tgt_roll_centi_deg;    /* 0.01° */
    int32_t  tgt_pitch_centi_deg;
    int32_t  tgt_yaw_rate_centi_dps;
} AnonDbgCtrl;



/**
 * @brief  初始化匿名通信模块
 * @note   在 osKernelStart() 之前调用一次。
 *         清零内部状态与 PID 参数缓存（如需非零默认 PID，请在本函数内填写）。
 */
void Anonym_Init(void);

/**
 * @brief  接收字节流注入
 * @param  data : 本次从 UART 环形缓冲取出的数据
 * @param  len  : 字节数
 * @note   只在 CommTask 中调用（协议规定 CommTask 每 10 ms 排空 DMA 环形缓冲）。
 *         本函数内部维护帧解析状态机，遇到非法 LEN 会自动回退到帧头重新同步。
 */
void Anonym_RxPush(const uint8_t *data, uint16_t len);

/**
 * @brief  10 ms 节拍调度
 * @note   只在 CommTask 中调用，周期 10 ms。
 *         每拍发 0x04 + 0xF1；10 拍轮转发
 *         0x03(欧拉角) / 0x01(惯导) / 0x02(磁力计)。
 *         地面阶段不发遥控/高度/模式/电机帧。
 */
void Anonym_Step(void);

/**
 * @brief  发送一帧匿名协议上行数据
 * @param  daddr : 目标地址（0xAF 主机 / 0xFF 广播）
 * @param  id    : 帧 ID
 * @param  data  : 数据区指针
 * @param  len   : 数据长度，不得超过 ANON_MAX_DATA
 * @note   非阻塞：拼帧后入 TX 块池队列，由 DMA 异步发出；
 *         池满时本帧丢弃并累加丢帧计数（AnonGetTxDropCount），不重试。
 */
void AnonSend(uint8_t daddr, uint8_t id, const uint8_t *data, uint8_t len);

/* TX 池初始化（在 MX_FREERTOS_Init 里调用）与发送泵 */
void Anonym_TxInit(void);
void Anonym_TxPump(void);

uint32_t AnonGetTxDropCount(void);

/**
 * @brief  发送 0xA0 LOG 文本帧（上位机日志窗口显示）
 * @param  color : 0 黑 / 1 红 / 2 绿
 * @param  str   : ASCII 字符串，最长 39 字节
 */
void AnonLog(uint8_t color, const char *str);

/* 读取 PID 增益（ID 2000~2020 内部域 0~100）。越界返回 0。 */
float Anon_GetPidGain(uint16_t id);

void Anon_GetDbgCtrl(AnonDbgCtrl *out);
/* ControlTask 每拍发布：目标姿态 + 状态位。CommTask 每 10ms 读取打到 0xF1 帧。 */
void Anon_SetTuneTargets(int16_t tgt_roll_centi_deg,
                         int16_t tgt_pitch_centi_deg,
                         int16_t tgt_yaw_rate_centi_dps,
                         uint8_t status_bits,
                         uint8_t mode);

#endif /* __ANONYM_H */
