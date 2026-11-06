/* DTU MQTT firmware: UART carries framed protobuf, never AT or MQTT packets. */
#include "app_config.h"
#include "modem_transport.h"
#if APP_UPLINK_ENABLED
#include "cmsis_os2.h"
#include "publish.h"
#include "usart.h"
#include <string.h>

static uint8_t ready;
static uint8_t sequence;
static uint8_t tx[PUBLISH_MAX_PAYLOAD_SIZE + 7U];

void Modem_Init(void)
{
  /* Local UART readiness only; DTU owns network startup and reconnection. */
  ready = (huart1.Instance != NULL && huart1.hdmatx != NULL);
}
uint8_t Modem_IsReady(void) { return ready; }
/* Downlink control is not implemented in this acquisition firmware. */
void Modem_OnRxEvent(uint16_t size) { (void)size; }

HAL_StatusTypeDef Modem_Send(const uint8_t *payload, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  HAL_StatusTypeDef status;
  uint32_t start;
  if (!ready || payload == NULL || length == 0U || length > PUBLISH_MAX_PAYLOAD_SIZE)
    return HAL_ERROR;
  if (huart1.gState != HAL_UART_STATE_READY) return HAL_BUSY;
  tx[0] = 0xAAU;
  tx[1] = 0x55U;
  tx[2] = (uint8_t)length;
  tx[3] = (uint8_t)(length >> 8);
  tx[4] = sequence;
  memcpy(&tx[5], payload, length);
  for (uint16_t i = 2U; i < length + 5U; ++i)
  {
    crc ^= tx[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
      crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
  }
  tx[length + 5U] = (uint8_t)crc;
  tx[length + 6U] = (uint8_t)(crc >> 8);
  status = HAL_UART_Transmit_DMA(&huart1, tx, (uint16_t)(length + 7U));
  if (status != HAL_OK) return status;
  /* Once DMA starts, consume the sequence even if the frame is truncated. */
  ++sequence;
  start = HAL_GetTick();
  while (huart1.gState != HAL_UART_STATE_READY)
  {
    /* Allow maximum payload wire time at 115200 8N1 plus scheduling margin. */
    if (HAL_GetTick() - start >= ((uint32_t)length + 7U) * 10U * 1000U / 115200U + 200U)
    {
      (void)HAL_UART_AbortTransmit(&huart1);
      return HAL_TIMEOUT;
    }
    osDelay(1U);
  }
  return huart1.ErrorCode == HAL_UART_ERROR_NONE ? HAL_OK : HAL_ERROR;
}
#else
void Modem_Init(void) {}
uint8_t Modem_IsReady(void) { return 0U; }
void Modem_OnRxEvent(uint16_t size) { (void)size; }
HAL_StatusTypeDef Modem_Send(const uint8_t *payload, uint16_t length)
{
  (void)payload;
  (void)length;
  return HAL_ERROR;
}
#endif
