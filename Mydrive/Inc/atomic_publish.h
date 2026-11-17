#ifndef ATOMIC_PUBLISH_H
#define ATOMIC_PUBLISH_H

#include "control_types.h"

/**
 * @brief  双缓冲快照通道
 * @note   初始化后由唯一写者调用 FcChan_Publish()，
 *         任意多个读者调用 FcChan_Read()。
 *         结构体成员由本模块维护，调用者不得直接访问。
 */
typedef struct {
    uint8_t  *slot[2];      /**< 两个载荷槽 */
    uint32_t  size;         /**< 载荷字节数 */
    volatile uint32_t seq;  /**< 奇数=写入中，偶数=已发布 */
    volatile uint8_t  active; /**< 读者应读取的槽索引 */
} FcChannel;

void FcChan_Init(FcChannel *ch, void *slot_a, void *slot_b, uint32_t size);

void FcChan_Publish(FcChannel *ch, const void *src, fc_usec_t usec);

bool FcChan_Read(const FcChannel *ch, void *dst);

/* ---- 全局通道（定义在 atomic_publish.c，存储也由该文件私有） ---- */

extern FcChannel g_chan_sensor;   /**< BusTask   -> AttitudeTask               */
extern FcChannel g_chan_attitude; /**< AttitudeTask -> ControlTask/SafetyTask  */
extern FcChannel g_chan_altitude; /**< BusTask   -> ControlTask/SafetyTask/Comm */
extern FcChannel g_chan_rc;       /**< RcTask    -> ControlTask/SafetyTask      */
extern FcChannel g_chan_ctrl;     /**< ControlTask -> CommTask                  */

void Fc_Interlock_Init(void);

#endif /* ATOMIC_PUBLISH_H */
