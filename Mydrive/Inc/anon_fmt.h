/* anon_fmt.h */
#ifndef ANON_FMT_H
#define ANON_FMT_H
#include <stdint.h>

/* ==========================================================================
 * 小端写入 / 读取、浮点定标
 * ========================================================================== */

/**
 * @brief  将 16 位无符号整数按小端写入缓冲区
 * @param  p : 目标缓冲区指针，至少 2 字节
 * @param  v : 待写入的值
 * @retval 无
 */
static inline void PutU16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

/**
 * @brief  将 16 位有符号整数按小端写入缓冲区
 * @param  p : 目标缓冲区指针，至少 2 字节
 * @param  v : 待写入的值
 * @retval 无
 */
static inline void PutI16(uint8_t *p, int16_t v)
{
	PutU16(p, (uint16_t)v);
}

/**
 * @brief  将 32 位有符号整数按小端写入缓冲区
 * @param  p : 目标缓冲区指针，至少 4 字节
 * @param  v : 待写入的值
 * @retval 无
 */
static inline void PutI32(uint8_t *p, int32_t v)
{
	p[0] = (uint8_t)((uint32_t)v & 0xFFu);
	p[1] = (uint8_t)(((uint32_t)v >> 8) & 0xFFu);
	p[2] = (uint8_t)(((uint32_t)v >> 16) & 0xFFu);
	p[3] = (uint8_t)(((uint32_t)v >> 24) & 0xFFu);
}

/**
 * @brief  从缓冲区按小端读取 16 位无符号整数
 * @param  p : 源缓冲区指针，至少 2 字节
 * @retval 读取到的 16 位无符号整数
 */
static inline uint16_t GetU16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/**
 * @brief  从缓冲区按小端读取 32 位有符号整数
 * @param  p : 源缓冲区指针，至少 4 字节
 * @retval 读取到的 32 位有符号整数
 */
static inline int32_t GetI32(const uint8_t *p)
{
	return (int32_t)((uint32_t)p[0]
								 | ((uint32_t)p[1] << 8)
								 | ((uint32_t)p[2] << 16)
								 | ((uint32_t)p[3] << 24));
}

/**
 * @brief  浮点数四舍五入转换为 16 位有符号整数（带饱和）
 * @param  v : 输入浮点数
 * @retval 转换后的 int16_t，越界时饱和到边界值
 */
static inline int16_t F2I16(float v)
{
	const float r = (v >= 0.0f) ? (v + 0.5f) : (v - 0.5f);
	if (r > 32767.0f)  { return 32767; }
	if (r < -32768.0f) { return -32768; }
	return (int16_t)r;
}

/**
 * @brief  浮点数四舍五入转换为 32 位有符号整数（带饱和）
 * @param  v : 输入浮点数
 * @retval 转换后的 int32_t，越界时饱和到边界值
 */
static inline int32_t F2I32(float v)
{
	const float r = (v >= 0.0f) ? (v + 0.5f) : (v - 0.5f);
	if (r > 2147483520.0f)  { return 2147483647; }
	if (r < -2147483648.0f) { return (-2147483647 - 1); }
	return (int32_t)r;
}

#endif
