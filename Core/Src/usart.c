/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"

/* The 4G DTU only needs a bounded AT bootstrap during power-up.
 * After that, USART1 returns to raw TCP payload transport and task08 keeps
 * owning the business publish path exactly as before.
 *
 * Current publish plain-text topics are still defined in publish.c:
 *   fast_telemetry / bms_summary / vehicle_state / bms_detail / thermal_summary
 * The bootstrap below does not rewrite payload encoding or inspect topics.
 */
#define Y100M_AT_REPLY_BUFFER_SIZE        256U
#define Y100M_AT_LOG_BUFFER_SIZE          192U
#define Y100M_AT_GUARD_TIME_MS           1000U
#define Y100M_AT_CMD_TIMEOUT_MS          2000U
#define Y100M_AT_RESET_PULSE_MS          1000U
#define Y100M_AT_BOOT_WAIT_MS            5000U
#define Y100M_AT_MAX_HANDSHAKE_ATTEMPTS     2U
#define Y100M_AT_EXIT_CMD               "AT+ENTM"
#define Y100M_UART3_LOG_TIMEOUT_MS      1000U

typedef struct
{
  const char *name;
  const char *command;
  const char *expected;
  uint32_t timeout_ms;
} Y100M_AtStep_t;

static uint8_t g_y100mBootstrapDone = 0U;
static uint8_t g_y100mBootstrapOk = 0U;
static char g_y100mAtReplyBuffer[Y100M_AT_REPLY_BUFFER_SIZE];
static char g_y100mLogBuffer[Y100M_AT_LOG_BUFFER_SIZE];

/* ── USART1 RX DMA for AT response reception ── */
#define Y100M_AT_RX_DMA_BUFFER_SIZE       256U
static uint8_t  g_y100mUsart1RxDmaBuffer[Y100M_AT_RX_DMA_BUFFER_SIZE];
static volatile uint8_t  g_y100mUsart1RxDmaDone;
static volatile uint16_t g_y100mUsart1RxDmaSize;

/* Called from HAL_UARTEx_RxEventCallback when USART1 idle is detected. */
void Y100M_OnUsart1RxIdle(uint16_t size)
{
    g_y100mUsart1RxDmaSize = size;
    g_y100mUsart1RxDmaDone = 1U;
}

/* USART IRQ handlers — HAL DMA TX completion enables TC interrupt
 * to flush the shift register; without these, gState stays BUSY. */
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&huart3); }

/* Keep all DTU power-up AT steps local to usart.c so later command tweaks stay
 * isolated from FreeRTOS tasks and publish/protobuf logic.
 *
 * Replace the TCP-specific placeholders below with the exact sequence from:
 *   https://yinerda.yuque.com/yt1fh6/4gdtu/rq1k2ege1avsqga8
 *
 * The currently enabled steps are intentionally conservative: they only verify
 * that the module can talk, that the SIM is ready, and that the network state
 * is queryable. Unsupported commands should not be guessed here.
 */
static const Y100M_AtStep_t g_y100mBootstrapSteps[] =
{
  { "Disable echo",          "ATE0",      "OK",      1000U },
  { "Check SIM ready",       "AT+CPIN?",  "READY",   2000U },
  { "Check signal quality",  "AT+CSQ",    "+CSQ:",   2000U },
  { "Check network status",  "AT+CREG?",  "+CREG:",  3000U },
  /* Example placeholders for the final TCP client configuration:
   * { "Set work mode",      "AT+XXXX",   "OK",      2000U },
   * { "Set remote server",  "AT+XXXX",   "OK",      2000U },
   * { "Save parameters",    "AT+XXXX",   "OK",      2000U },
   */
};

static HAL_StatusTypeDef Y100M_DebugUart3DmaBlocking(const uint8_t *data, uint16_t length);
static void Y100M_DebugLog(const char *text);
static void Y100M_Uart1DrainRx(void);
static HAL_StatusTypeDef Y100M_WaitReply(const char *expected, uint32_t timeout_ms, char *reply_buffer, uint16_t reply_capacity);
static HAL_StatusTypeDef Y100M_SendAtExpect(const char *command, const char *expected, uint32_t timeout_ms);
static void Y100M_EnterAtMode(void);
static HAL_StatusTypeDef Y100M_ExitAtMode(void);
static void Y100M_ResetPulse(void);
static HAL_StatusTypeDef Y100M_RunBootstrapSteps(void);
void Y100M_BootstrapOnce(void);

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_tx;

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
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

  /* USER CODE BEGIN USART1_MspInit 1 */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
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
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

  /* USER CODE BEGIN USART2_MspInit 1 */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA1_Stream3;
    hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

  /* USER CODE BEGIN USART3_MspInit 1 */
  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE END USART3_MspInit 1 */
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
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);
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
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
  /* USER CODE BEGIN USART2_MspDeInit 1 */
    HAL_NVIC_DisableIRQ(USART2_IRQn);

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* ── DMA-based USART1 send for AT command sequences ── */
static HAL_StatusTypeDef Y100M_Uart1DmaSend(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
  uint32_t start_tick;
  HAL_StatusTypeDef status;

  if ((data == NULL) || (length == 0U) || (huart1.Instance == NULL))
  {
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  while ((HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) ||
         (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX))
  {
    if ((HAL_GetTick() - start_tick) >= timeout_ms)
    {
      return HAL_TIMEOUT;
    }
    osDelay(1);
  }

  status = HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, length);
  if (status != HAL_OK)
  {
    return status;
  }

  start_tick = HAL_GetTick();
  while ((HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) ||
         (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX))
  {
    if ((HAL_GetTick() - start_tick) >= timeout_ms)
    {
      (void)HAL_UART_Abort(&huart1);
      return HAL_TIMEOUT;
    }
    osDelay(1);
  }

  return HAL_OK;
}

/* ── RTOS-aware millisecond delay ── */
static void Y100M_DelayMs(uint32_t delay_ms)
{
  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(delay_ms);
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}

/* Send a debug line through USART3 using DMA and wait until the transfer finishes. */
static HAL_StatusTypeDef Y100M_DebugUart3DmaBlocking(const uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef status;
  uint32_t start_tick;

  if ((data == NULL) || (length == 0U) || (huart3.Instance == NULL))
  {
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  while ((HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX) ||
         (HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX_RX))
  {
    if ((HAL_GetTick() - start_tick) >= Y100M_UART3_LOG_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }

  status = HAL_UART_Transmit_DMA(&huart3, (uint8_t *)data, length);
  if (status != HAL_OK)
  {
    return status;
  }

  start_tick = HAL_GetTick();
  while ((HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX) ||
         (HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX_RX))
  {
    if ((HAL_GetTick() - start_tick) >= Y100M_UART3_LOG_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

/* Forward a text log to the USART3 debug output path. */
static void Y100M_DebugLog(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  (void)Y100M_DebugUart3DmaBlocking((const uint8_t *)text, (uint16_t)strlen(text));
}

/* Remove any stale bytes from USART1 before the next AT command is sent. */
static void Y100M_Uart1DrainRx(void)
{
  uint32_t start_tick = HAL_GetTick();
  uint8_t byte;

  while ((HAL_GetTick() - start_tick) < 50U)
  {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    {
      byte = (uint8_t)(huart1.Instance->DR & 0xFFU);
      (void)byte;
      start_tick = HAL_GetTick();
    }
    else
    {
      osDelay(1);
    }
  }
}

/* Wait for a reply that contains the expected text, or fail on timeout.
 * Uses USART1 RX DMA with idle detection instead of byte-by-byte polling. */
static HAL_StatusTypeDef Y100M_WaitReply(const char *expected, uint32_t timeout_ms, char *reply_buffer, uint16_t reply_capacity)
{
  uint32_t start_tick = HAL_GetTick();
  HAL_StatusTypeDef status;

  if ((reply_buffer == NULL) || (reply_capacity == 0U))
  {
    return HAL_ERROR;
  }

  reply_buffer[0] = '\0';

  g_y100mUsart1RxDmaDone = 0U;
  g_y100mUsart1RxDmaSize = 0U;
  (void)HAL_UART_DMAStop(&huart1);
  __HAL_UART_CLEAR_OREFLAG(&huart1);

  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                         g_y100mUsart1RxDmaBuffer,
                                         Y100M_AT_RX_DMA_BUFFER_SIZE);
  if (status != HAL_OK)
  {
    return status;
  }

  while ((HAL_GetTick() - start_tick) < timeout_ms)
  {
    if (g_y100mUsart1RxDmaDone != 0U)
    {
      uint16_t received = g_y100mUsart1RxDmaSize;
      if (received > 0U && received <= Y100M_AT_RX_DMA_BUFFER_SIZE)
      {
        uint16_t copy_len = received;
        if (copy_len > (uint16_t)(reply_capacity - 1U))
        {
          copy_len = (uint16_t)(reply_capacity - 1U);
        }
        (void)memcpy(reply_buffer, g_y100mUsart1RxDmaBuffer, copy_len);
        reply_buffer[copy_len] = '\0';

        if ((expected != NULL) && (expected[0] != '\0') &&
            (strstr(reply_buffer, expected) != NULL))
        {
          return HAL_OK;
        }

        if ((strstr(reply_buffer, "ERROR") != NULL) &&
            ((expected == NULL) || (strstr(expected, "ERROR") == NULL)))
        {
          return HAL_ERROR;
        }
      }

      g_y100mUsart1RxDmaDone = 0U;
      g_y100mUsart1RxDmaSize = 0U;
      status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                             g_y100mUsart1RxDmaBuffer,
                                             Y100M_AT_RX_DMA_BUFFER_SIZE);
      if (status != HAL_OK)
      {
        return status;
      }
    }
    else
    {
      osDelay(1);
    }
  }

  return HAL_TIMEOUT;
}

/* Send one AT command and confirm that the expected response arrives. */
static HAL_StatusTypeDef Y100M_SendAtExpect(const char *command, const char *expected, uint32_t timeout_ms)
{
  HAL_StatusTypeDef tx_status;
  HAL_StatusTypeDef rx_status;
  uint16_t log_length;

  if ((command == NULL) || (command[0] == '\0'))
  {
    return HAL_OK;
  }

  Y100M_Uart1DrainRx();

  log_length = (uint16_t)snprintf(g_y100mLogBuffer,
                                  sizeof(g_y100mLogBuffer),
                                  "[4G][TX] %s\r\n",
                                  command);
  if ((log_length > 0U) && (log_length < sizeof(g_y100mLogBuffer)))
  {
    Y100M_DebugLog(g_y100mLogBuffer);
  }

  tx_status = Y100M_Uart1DmaSend((const uint8_t *)command, (uint16_t)strlen(command), timeout_ms);
  if (tx_status != HAL_OK)
  {
    return tx_status;
  }

  tx_status = Y100M_Uart1DmaSend((const uint8_t *)"\r\n", 2U, timeout_ms);
  if (tx_status != HAL_OK)
  {
    return tx_status;
  }

  rx_status = Y100M_WaitReply(expected,
                              timeout_ms,
                              g_y100mAtReplyBuffer,
                              (uint16_t)sizeof(g_y100mAtReplyBuffer));

  log_length = (uint16_t)snprintf(g_y100mLogBuffer,
                                  sizeof(g_y100mLogBuffer),
                                  "[4G][RX][%s] %s\r\n",
                                  (rx_status == HAL_OK) ? "OK" :
                                  ((rx_status == HAL_TIMEOUT) ? "TIMEOUT" : "FAIL"),
                                  g_y100mAtReplyBuffer);
  if ((log_length > 0U) && (log_length < sizeof(g_y100mLogBuffer)))
  {
    Y100M_DebugLog(g_y100mLogBuffer);
  }

  return rx_status;
}

/* Enter AT command mode after the required guard time. */
static void Y100M_EnterAtMode(void)
{
  /* "+++" is treated as the lightest escape from transparent transfer back to
   * AT mode. If the module is already in AT mode, this sequence is harmless.
   */
  (void)Y100M_Uart1DmaSend((const uint8_t *)"+++", 3U, Y100M_AT_CMD_TIMEOUT_MS);
  Y100M_DelayMs(Y100M_AT_GUARD_TIME_MS);
  Y100M_Uart1DrainRx();
}

/* Leave AT command mode and return the module to data mode. */
static HAL_StatusTypeDef Y100M_ExitAtMode(void)
{
  if (Y100M_AT_EXIT_CMD[0] == '\0')
  {
    return HAL_OK;
  }

  return Y100M_SendAtExpect(Y100M_AT_EXIT_CMD, "OK", Y100M_AT_CMD_TIMEOUT_MS);
}

/* Pulse the external reset pin to force the 4G module to reboot. */
static void Y100M_ResetPulse(void)
{
  Y100M_DebugLog("[4G] AT bootstrap failed, pulse RST_4G high for 1s\r\n");
  HAL_GPIO_WritePin(RST_4G_GPIO_Port, RST_4G_Pin, GPIO_PIN_SET);
  Y100M_DelayMs(Y100M_AT_RESET_PULSE_MS);
  HAL_GPIO_WritePin(RST_4G_GPIO_Port, RST_4G_Pin, GPIO_PIN_RESET);
  Y100M_DelayMs(Y100M_AT_BOOT_WAIT_MS);
  Y100M_Uart1DrainRx();
}

/* Run the bounded one-shot AT bootstrap sequence for the 4G module. */
static HAL_StatusTypeDef Y100M_RunBootstrapSteps(void)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t index = 0U;
  uint8_t attempt = 0U;

  Y100M_EnterAtMode();

  for (attempt = 0U; attempt < Y100M_AT_MAX_HANDSHAKE_ATTEMPTS; attempt++)
  {
    status = Y100M_SendAtExpect("AT", "OK", Y100M_AT_CMD_TIMEOUT_MS);
    if (status == HAL_OK)
    {
      break;
    }

    /* If the AT probe fails, leave the module in business mode first, then
     * try to re-enter AT mode once more before escalating to hardware reset.
     */
    (void)Y100M_ExitAtMode();
    Y100M_EnterAtMode();
  }

  if (status != HAL_OK)
  {
    Y100M_ResetPulse();
    Y100M_EnterAtMode();
    status = Y100M_SendAtExpect("AT", "OK", Y100M_AT_CMD_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      (void)Y100M_ExitAtMode();
      return status;
    }
  }

  for (index = 0U; index < (sizeof(g_y100mBootstrapSteps) / sizeof(g_y100mBootstrapSteps[0])); index++)
  {
    const Y100M_AtStep_t *step = &g_y100mBootstrapSteps[index];

    if ((step->command == NULL) || (step->command[0] == '\0'))
    {
      continue;
    }

    status = Y100M_SendAtExpect(step->command, step->expected, step->timeout_ms);
    if (status != HAL_OK)
    {
      (void)snprintf(g_y100mLogBuffer,
                     sizeof(g_y100mLogBuffer),
                     "[4G] bootstrap step failed: %s\r\n",
                     step->name);
      Y100M_DebugLog(g_y100mLogBuffer);
      (void)Y100M_ExitAtMode();
      return status;
    }
  }

  return Y100M_ExitAtMode();
}

/* Execute the one-shot bootstrap only once during startup. */
void Y100M_BootstrapOnce(void)
{
  HAL_StatusTypeDef status;

  if (g_y100mBootstrapDone != 0U)
  {
    return;
  }

  g_y100mBootstrapDone = 1U;
  Y100M_DebugLog("[4G] start one-shot AT bootstrap\r\n");

  status = Y100M_RunBootstrapSteps();
  if (status == HAL_OK)
  {
    g_y100mBootstrapOk = 1U;
    Y100M_DebugLog("[4G] bootstrap complete, return USART1 to raw TCP payload mode\r\n");
  }
  else
  {
    g_y100mBootstrapOk = 0U;
    (void)snprintf(g_y100mLogBuffer,
                   sizeof(g_y100mLogBuffer),
                   "[4G] bootstrap incomplete, status=%d\r\n",
                   (int)status);
    Y100M_DebugLog(g_y100mLogBuffer);
  }
}

/* USER CODE END 1 */
