#ifndef __FREERTOS_APP_H
#define __FREERTOS_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os2.h"

#define APP_UART_TX_MAX_PAYLOAD        288U
#define DEBUG_QUEUE_DEPTH       2U
#define MLX_LINE_BUFFER_SIZE    APP_UART_TX_MAX_PAYLOAD
#define RS485_DMA_BUFFER_SIZE    256U
#define RS485_FLAG_RX_READY      (1UL << 0)
#define RS485_FLAG_RX_RESTART    (1UL << 1)
#define PUBLISH_COOLDOWN_MS      10U

typedef struct
{
  uint16_t length;
  uint8_t data[64];
} Peripheral_Rx_Frame_t;

typedef struct
{
  uint16_t length;
  uint8_t data[APP_UART_TX_MAX_PAYLOAD];
} App_Uart_Tx_Item_t;

extern osThreadId_t defaultTaskHandle;
extern osThreadId_t CanForViehcleHandle;
extern osThreadId_t CanForBMSHandle;
extern osThreadId_t MLX90640Handle;
extern osThreadId_t TensionSensorHandle;
extern osThreadId_t SPICanControlHandle;
extern osThreadId_t SPICanCDCHandle;
extern osThreadId_t Publish4GHandle;
extern osThreadId_t CanBTextHandle;
extern osThreadId_t initTaskBootHandle;

extern osMessageQueueId_t CAN_Msg_QueueHandle;
extern osMessageQueueId_t CAN_Msg_Queue_2Handle;
extern osMessageQueueId_t PublishQueueItemHandle;
extern osMessageQueueId_t CAN_Tx_QueueHandle;
extern osMessageQueueId_t task01DebugQueueHandle;

void MX_FREERTOS_Init(void);
void StartDefaultTask(void *argument);
void CanForViehcleTask(void *argument);
void CanForBMSTask(void *argument);
void MLX90640Task(void *argument);
void TensionSensorTask(void *argument);
void SPICanControlTask(void *argument);
void SPICanCDCTask(void *argument);
void Publish4GTask(void *argument);
void CanBTextTask(void *argument);
void InitTask_Boot(void *argument);

#ifdef __cplusplus
}
#endif

#endif
