#include "atomic_publish.h"
#include <string.h>
#include "stm32f4xx_hal.h"   /* 提供 __DMB() */

/* ==========================================================================
 * 私有存储：5 条通道 × 2 槽
 * 放在 BSS 里，不占用 FreeRTOS heap，也不会因 malloc 失败而启动失败。
 * ========================================================================== */
static SensorData        s_sensor_slot[2];
static AttitudeSnapshot  s_att_slot[2];
static AltitudeSnapshot  s_alt_slot[2];
static RcCommandSnapshot s_rc_slot[2];
static ControlOutput     s_ctrl_slot[2];

FcChannel g_chan_sensor;
FcChannel g_chan_attitude;
FcChannel g_chan_altitude;
FcChannel g_chan_rc;
FcChannel g_chan_ctrl;

/**
 * @brief  初始化一条快照通道
 * @param  ch     : 通道对象
 * @param  slot_a : 槽 A 的存储（必须 >= size 字节且生命周期覆盖全程）
 * @param  slot_b : 槽 B 的存储
 * @param  size   : 载荷字节数，必须 >= sizeof(FcStamp)
 * @retval 无
 * @note   必须在创建任何写者/读者任务之前调用。
 */
void FcChan_Init(FcChannel *ch, void *slot_a, void *slot_b, uint32_t size)
{
    if ((ch == NULL) || (slot_a == NULL) || (slot_b == NULL)) {
        return;
    }
    /* 载荷必须至少能容纳 FcStamp，否则回填时间戳会越界写 */
    if (size < (uint32_t)sizeof(FcStamp)) {
        return;
    }

    ch->slot[0] = (uint8_t *)slot_a;
    ch->slot[1] = (uint8_t *)slot_b;
    ch->size    = size;
    ch->seq     = 0u;
    ch->active  = 0u;

    /* 槽必须先清零：读者可能在第一次发布之前就来读（读到 seq==0 会拒绝，
     * 但清零能保证任何意外路径读到的都是确定值而不是随机 BSS 残留）。 */
    (void)memset(slot_a, 0, size);
    (void)memset(slot_b, 0, size);
}

/**
 * @brief  发布一份快照
 * @param  ch   : 通道对象
 * @param  src  : 载荷源（其首字段必须是 FcStamp，本函数会回填 usec/seq）
 * @param  usec : 本次采样/生成时刻，单位 µs
 * @retval 无
 * @pre    调用者必须是该通道的唯一写者。
 * @note   不能在中断中使用本函数以外的同通道写操作；
 *         本函数本身可在中断中调用（不涉及 RTOS API）。
 */
void FcChan_Publish(FcChannel *ch, const void *src, fc_usec_t usec)
{
    if ((ch == NULL) || (src == NULL)) {
        return;
    }

    /* 写非活动槽，读者此时仍在读活动槽 */
    const uint8_t next = (uint8_t)(ch->active ^ 1u);

    /* 1) seq 先变成奇数：告知读者"正在写"；
     *    回绕时跳过 0，避免读者把 seq==0 误判为"从未发布" */
    uint32_t seq_odd = ch->seq + 1u;
    if (seq_odd == 0u) { seq_odd = 1u; }
    ch->seq = seq_odd;
    __DMB();

    /* 2) 拷贝载荷 */
    (void)memcpy(ch->slot[next], src, ch->size);

    /* 3) 回填时间戳与最终序列号（偶数） */
    {
        FcStamp *st = (FcStamp *)(void *)ch->slot[next];
        st->usec = usec;
        st->seq  = seq_odd + 1u;
    }
    __DMB();

    /* 4) 翻转活动槽，再发布偶数 seq。
     *    顺序至关重要：active 必须先于 seq 生效，
     *    这样读者只要看到 seq 未变，就一定能读到与 seq 对应的那个槽。 */
    ch->active = next;
    __DMB();
    ch->seq = seq_odd + 1u;
}

/**
 * @brief  读取一份一致快照
 * @param  ch  : 通道对象
 * @param  dst : 输出缓冲，大小必须等于初始化时的 size
 * @retval true  读取成功，dst 内容一致
 * @retval false 通道尚未发布过，或连续 4 次校验失败
 */
bool FcChan_Read(const FcChannel *ch, void *dst)
{
    if ((ch == NULL) || (dst == NULL)) {
        return false;
    }

    for (uint32_t attempt = 0u; attempt < 4u; ++attempt) {
        const uint32_t s1 = ch->seq;

        /* 奇数 = 写者正在写入；0 = 从未发布过 */
        if (((s1 & 1u) != 0u) || (s1 == 0u)) {
            continue;
        }
        __DMB();

        (void)memcpy(dst, ch->slot[ch->active], ch->size);

        __DMB();
        if (ch->seq == s1) {
            return true;   /* 拷贝期间没有发生新的发布，数据一致 */
        }
        /* 否则说明写者在拷贝期间发布了一次，重试 */
    }
    return false;
}

/**
 * @brief  初始化全部全局通道
 * @note   必须在 osKernelStart() 之前调用一次。
 */
void Fc_Interlock_Init(void)
{
    FcChan_Init(&g_chan_sensor,
                &s_sensor_slot[0], &s_sensor_slot[1],
                (uint32_t)sizeof(SensorData));
    FcChan_Init(&g_chan_attitude,
                &s_att_slot[0], &s_att_slot[1],
                (uint32_t)sizeof(AttitudeSnapshot));
    FcChan_Init(&g_chan_altitude,
                &s_alt_slot[0], &s_alt_slot[1],
                (uint32_t)sizeof(AltitudeSnapshot));
    FcChan_Init(&g_chan_rc,
                &s_rc_slot[0], &s_rc_slot[1],
                (uint32_t)sizeof(RcCommandSnapshot));
    FcChan_Init(&g_chan_ctrl,
                &s_ctrl_slot[0], &s_ctrl_slot[1],
                (uint32_t)sizeof(ControlOutput));
}
