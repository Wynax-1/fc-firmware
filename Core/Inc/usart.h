/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include <stdbool.h>

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

/* 驱动层资源约束：一次入队的帧长度上限（含协议头尾开销）。
 * 这是 TX 块池的块大小，与任何具体协议无关；上层自行保证不超。 */
#define UART1_TX_BLOCK_SIZE   64u
#define UART1_TX_POOL_SIZE    8u

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/**
 * @brief  启动 USART1 IDLE+DMA 接收
 * @note   在 MX_USART1_UART_Init 之后、osKernelStart 之前调用一次。
 */
void Uart1_Rx_Start(void);

/**
 * @brief  从 USART1 接收环形缓冲取数据
 * @param  dst : 输出缓冲
 * @param  cap : 最多取多少字节
 * @retval 实际取出字节数（0 表示暂无数据）
 */
uint16_t Uart_RxRing_Pop(uint8_t *dst, uint16_t cap);

/**
 * @brief  查询并清除"接收链路中断"标志
 * @retval true  接收曾中断，调用方应调 Uart1_Rx_Start() 恢复
 */
bool Uart1_Rx_TakeRestartReq(void);

void     Uart1_TxInit(void);
bool     Uart1_TxEnqueue(const uint8_t *buf, uint16_t len);
void     Uart1_TxPump(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

