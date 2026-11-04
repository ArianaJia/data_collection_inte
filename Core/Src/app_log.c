/* Task-context logging. The default task is the sole DMA queue consumer. */
#include "app_log.h"
#include "freertos_app.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
static App_Uart_Tx_Item_t g_debugTxItem;
static App_Uart_Tx_Item_t g_uartQueueWorkItem;
static char g_formatBuffer[APP_UART_TX_MAX_PAYLOAD];
static uint8_t g_formatBusy;

void App_LogPrintf(const char *format, ...)
{
#if APP_DEBUG_LOG_ENABLED
  if (format == NULL) return;
  taskENTER_CRITICAL();
  if (g_formatBusy != 0U)
  {
    taskEXIT_CRITICAL();
    return;
  }
  g_formatBusy = 1U;
  taskEXIT_CRITICAL();
  va_list args;
  va_start(args, format);
  (void)vsnprintf(g_formatBuffer, sizeof(g_formatBuffer), format, args);
  va_end(args);
  App_DebugLogString(g_formatBuffer);
  taskENTER_CRITICAL();
  g_formatBusy = 0U;
  taskEXIT_CRITICAL();
#else
  (void)format;
#endif
}
static osStatus_t App_QueueBytes(osMessageQueueId_t queue_handle, const uint8_t *data, uint16_t length)
{
  /* Copy bytes into the shared UART queue item before posting to a mailbox. */
  App_Uart_Tx_Item_t *item = &g_uartQueueWorkItem;

  if ((queue_handle == NULL) || (data == NULL) || (length == 0U))
  {
    return osErrorParameter;
  }

  if (length > APP_UART_TX_MAX_PAYLOAD)
  {
    length = APP_UART_TX_MAX_PAYLOAD;
  }

  /* Protect the shared item; never wait for queue space while interrupts are masked. */
  taskENTER_CRITICAL();
  item->length = length;
  (void)memcpy(item->data, data, length);

  osStatus_t status = (xQueueSend((QueueHandle_t)queue_handle, item, 0U) == pdPASS)
                      ? osOK : osErrorResource;
  taskEXIT_CRITICAL();
  return status;
}

static osStatus_t App_QueueText(osMessageQueueId_t queue_handle, const char *text)
{
  /* Text is just a byte buffer with a known length. */
  if (text == NULL)
  {
    return osErrorParameter;
  }

  return App_QueueBytes(queue_handle, (const uint8_t *)text, (uint16_t)strlen(text));
}

void App_DebugLogString(const char *text)
{
#if (APP_DEBUG_LOG_ENABLED != 0U)
  /* All debug strings are funneled through the USART3 debug queue. */
  (void)App_QueueText(task01DebugQueueHandle, text);
#else
  (void)text;
#endif
}

static HAL_StatusTypeDef App_UART_TransmitDmaBlocking(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
  uint32_t start_tick;

  if ((huart == NULL) || (data == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  while ((HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX) ||
         (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX_RX))
  {
    if ((HAL_GetTick() - start_tick) >= 20U)
    {
      (void)HAL_UART_AbortTransmit(huart);
      return HAL_TIMEOUT;
    }
    osDelay(1);
  }

  HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, data, length);
  if (status != HAL_OK)
  {
    return status;
  }
  /* DMA owns the debug buffer until transmission completes. */
  start_tick = HAL_GetTick();
  while ((HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX) ||
         (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX_RX))
  {
    if ((HAL_GetTick() - start_tick) >= 100U)
    {
      (void)HAL_UART_AbortTransmit(huart);
      return HAL_TIMEOUT;
    }
    osDelay(1);
  }
  return HAL_OK;
}

static void App_LogCanRxContext(const char *prefix, const CAN_ControllerContext_t *ctx)
{
  if ((prefix == NULL) || (ctx == NULL))
  {
    return;
  }

  App_LogPrintf(
      "%s ide=%lu std=0x%03lX ext=0x%08lX dlc=%lu fifo=%lu data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
      prefix,
      (unsigned long)ctx->last_rx_ide,
      (unsigned long)((ctx->last_rx_ide == CAN_ID_EXT) ? 0UL : ctx->last_rx_id),
      (unsigned long)((ctx->last_rx_ide == CAN_ID_EXT) ? ctx->last_rx_id : 0UL),
      (unsigned long)ctx->last_rx_dlc,
      (unsigned long)ctx->last_rx_fifo_fill,
      ctx->last_rx_data[0],
      ctx->last_rx_data[1],
      ctx->last_rx_data[2],
      ctx->last_rx_data[3],
      ctx->last_rx_data[4],
      ctx->last_rx_data[5],
      ctx->last_rx_data[6],
      ctx->last_rx_data[7]);
}

void app_can_debug_log_rx_context(const char *prefix, const CAN_ControllerContext_t *ctx)
{
#if (APP_DEBUG_LOG_ENABLED != 0U)
  App_LogCanRxContext(prefix, ctx);
#else
  (void)prefix;
  (void)ctx;
#endif
}

void App_LogProcess(void)
{
  if (osMessageQueueGet(task01DebugQueueHandle, &g_debugTxItem, NULL, 0U) == osOK)
  {
    (void)App_UART_TransmitDmaBlocking(&huart3, g_debugTxItem.data, g_debugTxItem.length);
  }
}

void App_LogModem(const char *text)
{
#if (APP_MODEM_LOG_ENABLED != 0U)
  if (text == NULL)
  {
    return;
  }

  App_DebugLogString(text);
#else
  (void)text;
#endif
}
/* Boot diagnostics only: preserves the existing blocking MCP startup path.
 * Never call concurrently with App_LogProcess/DMA transmission. */
void App_LogBootImmediate(const char *text)
{
#if APP_DEBUG_LOG_ENABLED
  if (text != NULL && huart3.Instance != NULL)
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)text, (uint16_t)strlen(text), 100U);
#else
  (void)text;
#endif
}
