/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = BT_TX_Pin|BT_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, BT_TX_Pin|BT_RX_Pin);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <string.h>

/* ==========================================================================
 * USART1 RX：循环 DMA + 单调写计数 + 零拷贝
 * ========================================================================== */
#define UART1_RX_BUF_SIZE   512u    /* 必须是 2 的幂，且 ≥ 2×最大帧长 */

static uint8_t           s_rx_buf[UART1_RX_BUF_SIZE];
static volatile uint32_t s_rx_evt_wr  = 0u;
static volatile uint32_t s_rx_rd      = 0u;
static volatile bool     s_rx_restart_req = false;

/**
 * @brief  计算 RX 环形缓冲的单调写计数
 * @retval 当前 DMA 写指针对应的绝对写入位置（单调递增，不回绕）
 * @note   以事件计数为基准，结合 DMA 剩余计数修正回绕；结果不小于快照值。
 */
static uint32_t rx_write_count(void)
{
	/* 先快照事件计数、再读 DMA 指针，缩小与中断侧竞态窗口 */
	const uint32_t wr_snapshot = s_rx_evt_wr;
	/* DMA 剩余计数是倒数的，用整圈长度相减得到圈内写位置 */
	const uint32_t pos =
		(uint32_t)UART1_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
	const uint32_t pos_old = wr_snapshot & (UART1_RX_BUF_SIZE - 1u);
	const uint32_t base    = wr_snapshot & ~(uint32_t)(UART1_RX_BUF_SIZE - 1u);

	/* DMA 位置回退说明事件后已跨过一圈，补一个圈长 */
	uint32_t candidate;
	if (pos >= pos_old) {
		candidate = base + pos;
	} else {
		candidate = base + UART1_RX_BUF_SIZE + pos;
	}
	/* 事件尚未处理时 DMA 读数可能落后于快照，取快照兜底保证单调不减 */
	return (candidate > wr_snapshot) ? candidate : wr_snapshot;
}

/**
 * @brief  从 RX 环形缓冲读出数据（拷贝到调用方缓冲区）
 * @param  dst : 目标缓冲区指针
 * @param  cap : 目标缓冲区容量（字节）
 * @retval 实际读出的字节数，0 表示当前无数据可读
 */
uint16_t Uart_RxRing_Pop(uint8_t *dst, uint16_t cap)
{
    if ((dst == NULL) || (cap == 0u)) { return 0u; }

    const uint32_t rd    = s_rx_rd;
    const uint32_t wr    = rx_write_count();
    /* 两侧计数均单调递增，无符号减法天然处理写侧跨圈的情况 */
    const uint32_t avail = wr - rd;
    const uint16_t n = (avail < (uint32_t)cap) ? (uint16_t)avail : cap;

    for (uint16_t i = 0u; i < n; ++i) {
        dst[i] = s_rx_buf[(rd + i) & (UART1_RX_BUF_SIZE - 1u)];
    }
    /* 屏障保证数据拷贝完成后才发布新读指针，中断侧不会读到半更新状态 */
    __DMB();
    s_rx_rd = rd + (uint32_t)n;
    return n;
}

/**
 * @brief  启动/重启 USART1 循环 DMA 空闲接收
 * @note   先中止旧接收并清错误标志，再重新挂载整圈 DMA；
 *         启动失败时置 s_rx_restart_req，由上层稍后重试。
 */
void Uart1_Rx_Start(void)
{
    /* 先中止旧接收，释放 DMA 通道与外设状态，保证能重新挂载 */
    (void)HAL_UART_AbortReceive(&huart1);

    /* 清掉上次会话残留的错误标志，否则重启后第一帧立刻触发错误回调 */
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);

    /* 整圈挂载循环接收；关半传输中断，只保留 Idle/TC 两类事件 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx_buf,
                                      UART1_RX_BUF_SIZE) != HAL_OK) {
        s_rx_restart_req = true;   /* DMA 忙等异常：交还上层稍后重试 */
        return;
    }
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    /* 启动成功后才复位计数，避免读侧看到"边收边归零"的中间状态 */
    s_rx_evt_wr = 0u;
    s_rx_rd     = 0u;
    s_rx_restart_req = false;
}

/**
 * @brief  UART 接收事件回调（HT / TC / Idle）
 * @param  huart : UART 句柄，仅处理 USART1
 * @param  Size  : 本次已接收字节数（TC 事件可能传整圈长度或 0）
 * @note   中断上下文调用，仅推进单调写计数 s_rx_evt_wr。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance != USART1) { return; }

	/* TC 事件：整圈完成。部分 F4 HAL 版本会传 Size==RxXferSize，
	 * 也有版本传 0。统一按"完整一圈"处理，避免漏计。 */
	if (Size == UART1_RX_BUF_SIZE) {
		s_rx_evt_wr = (s_rx_evt_wr & ~(uint32_t)(UART1_RX_BUF_SIZE - 1u))
									+ UART1_RX_BUF_SIZE;
		__DMB();
		return;
	}

	/* Idle/HT 事件：Size 为圈内位置，小于上次位置即已跨过一圈 */
	const uint32_t pos     = (uint32_t)Size;
	const uint32_t pos_old = s_rx_evt_wr & (UART1_RX_BUF_SIZE - 1u);
	const uint32_t base    = s_rx_evt_wr & ~(uint32_t)(UART1_RX_BUF_SIZE - 1u);

	if (pos >= pos_old) {
		s_rx_evt_wr = base + pos;
	} else {
		s_rx_evt_wr = base + UART1_RX_BUF_SIZE + pos;   /* 补一圈 */
	}
	__DMB();   /* 保证计数更新对任务侧可见 */
}

/**
 * @brief  取出并清除 RX 重启请求标志
 * @retval true  存在重启请求，调用方应执行 Uart1_Rx_Start
 * @retval false 无重启请求
 * @note   临界区内原子完成"读 + 清"。
 */
bool Uart1_Rx_TakeRestartReq(void)
{
    /* 保存并恢复 PRIMASK：调用时中断可能本来就关着，不能无条件重开 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool req = s_rx_restart_req;
    s_rx_restart_req = false;
    __set_PRIMASK(primask);
    return req;
}

/* ==========================================================================
 * USART1 TX：静态块池 + 队列 + DMA（与协议完全解耦）
 *   上层只提交"一串已经拼好的字节"，驱动层负责排队和 DMA 发出。
 * ========================================================================== */
typedef struct {
    uint8_t  data[UART1_TX_BLOCK_SIZE];
    uint16_t len;
} TxBlock;

static TxBlock           s_tx_pool[UART1_TX_POOL_SIZE];
static TxBlock          *s_tx_inflight = NULL;
static osMessageQueueId_t s_tx_free_q  = NULL;
static osMessageQueueId_t s_tx_ready_q = NULL;

/**
 * @brief  初始化 TX 块池与 free/ready 两个队列
 * @note   需在调度器启动前调用，把全部块挂入 free 队列。
 */
void Uart1_TxInit(void)
{
    /* 队列元素只是块指针，块本体常驻静态池，运行期零堆分配 */
    s_tx_free_q   = osMessageQueueNew(UART1_TX_POOL_SIZE, sizeof(TxBlock *), NULL);
    s_tx_ready_q  = osMessageQueueNew(UART1_TX_POOL_SIZE, sizeof(TxBlock *), NULL);
    s_tx_inflight = NULL;

    /* 初始状态：全部块都在 free 队列 */
    for (uint32_t i = 0u; i < UART1_TX_POOL_SIZE; ++i) {
        TxBlock *p = &s_tx_pool[i];
        (void)osMessageQueuePut(s_tx_free_q, &p, 0u, 0u);
    }
}

/**
 * @brief  入队一帧已拼好的字节
 * @param  buf : 数据指针
 * @param  len : 数据长度，不得超过 UART1_TX_BLOCK_SIZE
 * @retval true  入队成功
 * @retval false 参数非法或块池/队列已满，本次由上层决定是否丢帧
 */
bool Uart1_TxEnqueue(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0u) || (len > UART1_TX_BLOCK_SIZE)) {
        return false;
    }
    if ((s_tx_free_q == NULL) || (s_tx_ready_q == NULL)) { return false; }

    TxBlock *blk = NULL;
    /* timeout 0：池空立即返回，调用方（任务/中断）都不会被阻塞 */
    if (osMessageQueueGet(s_tx_free_q, &blk, NULL, 0u) != osOK || blk == NULL) {
        return false;
    }

    /* 拷入块内后 buf 即可复用，块的生命周期移交驱动层 */
    (void)memcpy(blk->data, buf, len);
    blk->len = len;

    if (osMessageQueuePut(s_tx_ready_q, &blk, 0u, 0u) != osOK) {
        (void)osMessageQueuePut(s_tx_free_q, &blk, 0u, 0u);   /* 回滚借出的块 */
        return false;
    }
    return true;
}

/**
 * @brief  从 ready 队列取一帧启动 DMA 发送
 * @note   队列未初始化、已有在途块、UART 忙或 ready 队列空时直接返回；
 *         启动失败时回滚在途标记并把块归还 free 队列。
 */
void Uart1_TxPump(void)
{
	if ((s_tx_ready_q == NULL) || (s_tx_free_q == NULL)) { return; }

	/* 临界区与 TC 回调的链式启动互斥，防止同一块被双发 */
	taskENTER_CRITICAL();
	if (s_tx_inflight != NULL || huart1.gState != HAL_UART_STATE_READY) {
		taskEXIT_CRITICAL();
		return;
	}

	TxBlock *blk = NULL;
	if (osMessageQueueGet(s_tx_ready_q, &blk, NULL, 0u) != osOK || blk == NULL) {
		taskEXIT_CRITICAL();
		return;
	}
	s_tx_inflight = blk;
	taskEXIT_CRITICAL();   /* 先认领块再放锁，缩短关中断时间 */

	/* 启动失败：回滚在途标记并把块还给 free 队列 */
	if (HAL_UART_Transmit_DMA(&huart1, blk->data, blk->len) != HAL_OK) {
		taskENTER_CRITICAL();
		s_tx_inflight = NULL;
		taskEXIT_CRITICAL();
		(void)osMessageQueuePut(s_tx_free_q, &blk, 0u, 0u);
	}
}

/* ==========================================================================
 * HAL 回调
 * ========================================================================== */
/**
 * @brief  UART 发送完成回调
 * @param  huart : UART 句柄，仅处理 USART1
 * @note   TC 中断上下文：归还刚发完的块，并链式启动下一帧。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != USART1) { return; }

	/* 归还刚发完的块 */
	if (s_tx_inflight != NULL) {
		TxBlock *blk = s_tx_inflight;
		s_tx_inflight = NULL;
		(void)osMessageQueuePut(s_tx_free_q, &blk, 0u, 0u);
	}

	/* 链式启动下一帧：TC 中断中 gState 已回到 READY */
	if ((s_tx_ready_q != NULL) && (huart1.gState == HAL_UART_STATE_READY)) {
		TxBlock *next = NULL;
		if (osMessageQueueGet(s_tx_ready_q, &next, NULL, 0u) == osOK &&
			next != NULL) {
			s_tx_inflight = next;
			if (HAL_UART_Transmit_DMA(&huart1, next->data, next->len) != HAL_OK) {
				/* 启动失败：归还块，积压帧等下次 Uart1_TxPump 重试 */
				s_tx_inflight = NULL;
				(void)osMessageQueuePut(s_tx_free_q, &next, 0u, 0u);
			}
		}
	}
}

/**
 * @brief  UART 错误回调
 * @param  huart : UART 句柄，仅处理 USART1
 * @note   发送被中断时归还在途块；RX 只置重启标志，
 *         由 CommTask 统一恢复，避免与 Uart1_Rx_Start 并发。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != USART1) { return; }

	/* gState 已回 READY 说明 TX 被错误中止，在途块不会再触发 TxCplt，需手动归还 */
	if ((s_tx_inflight != NULL) && (huart->gState == HAL_UART_STATE_READY)) {
		TxBlock *blk = s_tx_inflight;
		s_tx_inflight = NULL;
		(void)osMessageQueuePut(s_tx_free_q, &blk, 0u, 0u);
	}

	/* 只置标志，由 CommTask 统一恢复，避免与 CommTask 中的重启并发 */
	s_rx_restart_req = true;
}


/* USER CODE END 1 */
