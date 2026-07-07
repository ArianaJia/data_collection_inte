#ifndef __FREERTOS_APP_H
#define __FREERTOS_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os2.h"

#define APP_UART_TX_MAX_PAYLOAD        288U
#define TASK01_DEBUG_QUEUE_DEPTH       2U
#define TASK04_MLX_LINE_BUFFER_SIZE    APP_UART_TX_MAX_PAYLOAD

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
extern osThreadId_t myTask02Handle;
extern osThreadId_t myTask03Handle;
extern osThreadId_t myTask04Handle;
extern osThreadId_t myTask05Handle;
extern osThreadId_t myTask06Handle;
extern osThreadId_t myTask07Handle;
extern osThreadId_t myTask08Handle;
extern osThreadId_t myTask09Handle;

extern osMessageQueueId_t myQueue02Handle;
extern osMessageQueueId_t myQueue03Handle;
extern osMessageQueueId_t myQueue04Handle;
extern osMessageQueueId_t myQueue05Handle;
extern osMessageQueueId_t task01DebugQueueHandle;

void MX_FREERTOS_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);
void StartTask08(void *argument);
void StartTask09(void *argument);

#ifdef __cplusplus
}
#endif

#endif
