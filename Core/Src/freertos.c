/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "telemetry_data.h"
#include "can.h"
#include "can_decode.h"
#include "fdcan.h"
#include "gpio.h"
#include "mlx90640_app.h"
#include "MLX90640_API.h"
#include "publish.h"
#include "spi.h"
#include "usart.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK05_RS485_DMA_BUFFER_SIZE    256U
#define TASK05_FLAG_RS485_RX_READY      (1UL << 0)
#define TASK05_FLAG_RS485_RX_RESTART    (1UL << 1)
#define TASK08_PUBLISH_COOLDOWN_MS      10U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static App_Uart_Tx_Item_t g_task01DebugTxItem;
static App_Uart_Tx_Item_t g_uartQueueWorkItem;
static char g_task04MlxLineBuffer[APP_UART_TX_MAX_PAYLOAD];
static uint8_t g_task05Rs485DmaBuffer[TASK05_RS485_DMA_BUFFER_SIZE];
static volatile uint16_t g_task05Rs485DmaLastEventPos = 0U;
static volatile uint16_t g_task05Rs485DmaPendingBytes = 0U;
static volatile uint8_t g_task05Rs485DmaOverflow = 0U;
static uint16_t g_task05Rs485DmaReadPos = 0U;
static uint8_t g_task08PublishFrameBuffer[PUBLISH_MAX_FRAME_SIZE];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask08 */
osThreadId_t myTask08Handle;
const osThreadAttr_t myTask08_attributes = {
  .name = "myTask08",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myTask09 */
osThreadId_t myTask09Handle;
const osThreadAttr_t myTask09_attributes = {
  .name = "myTask09",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myQueue02 */
osMessageQueueId_t myQueue02Handle;
const osMessageQueueAttr_t myQueue02_attributes = {
  .name = "myQueue02"
};
/* Definitions for myQueue03 */
osMessageQueueId_t myQueue03Handle;
const osMessageQueueAttr_t myQueue03_attributes = {
  .name = "myQueue03"
};
/* Definitions for myQueue04 */
osMessageQueueId_t myQueue04Handle;
const osMessageQueueAttr_t myQueue04_attributes = {
  .name = "myQueue04"
};
/* Definitions for myQueue05 */
osMessageQueueId_t myQueue05Handle;
const osMessageQueueAttr_t myQueue05_attributes = {
  .name = "myQueue05"
};
/* Definitions for task01DebugQueue */
osMessageQueueId_t task01DebugQueueHandle;
const osMessageQueueAttr_t task01DebugQueue_attributes = {
  .name = "task01DebugQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void TaskDefault_ParseUsart3Message(void);
static void Task01_SendUsart3DebugMessage(void);
static void Task02_ParseCan1Message(const CAN_Msg_Queue_t *recv_data);
static void Task03_ParseCan2Message(const CAN_Msg_Queue_t *recv_data);
static void Task05_ParseRs485Message(const Peripheral_Rx_Frame_t *recv_data);
static void Task06_ParseSpi1Message(const Peripheral_Rx_Frame_t *recv_data);
static void Task07_ParseSpi3Message(const Peripheral_Rx_Frame_t *recv_data);
static void Task09_SendCan1Message(const CAN_Tx_Queue_t *tx_data);
static osStatus_t App_QueueBytes(osMessageQueueId_t queue_handle, const uint8_t *data, uint16_t length, uint32_t timeout_ms);
static osStatus_t App_QueueText(osMessageQueueId_t queue_handle, const char *text, uint32_t timeout_ms);
static void App_DebugLogString(const char *text);
static HAL_StatusTypeDef App_UART_TransmitDmaBlocking(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);
static uint16_t App_AppendTemperature(char *buffer, uint16_t offset, uint16_t size, float temperature);
static void Task04_SendMlxDebugMatrix(uint8_t sensor_id, const float *temp_map);
static void Task04_SendMlxSummary(uint8_t sensor_id);
static void Task04_SendMlxCanSummary(uint8_t sensor_id);
static uint32_t Task04_GetMlxCanSummaryId(uint8_t sensor_id);
static void Task04_EncodeCanSummaryPair(int32_t centi_c, uint8_t *integer_part, uint8_t *fraction_part);
static void TaskSpiPollCan(const Peripheral_Rx_Frame_t *recv_data, FDCAN_Bus_t bus, osMessageQueueId_t queue_handle, uint8_t can_channel);
static bool TaskSpiPollControllerCana(const Peripheral_Rx_Frame_t *recv_data, FDCAN_Bus_t bus);
static void TaskSpiPollCdcMonitor(const Peripheral_Rx_Frame_t *recv_data, FDCAN_Bus_t bus);
static HAL_StatusTypeDef App_Task05_StartRs485Dma(void);
static void App_Task05_ProcessRs485DmaBuffer(void);
static void App_Task05_ProcessRs485Span(uint16_t start, uint16_t end);
static void App_Task05_DispatchRs485Bytes(const uint8_t *data, uint16_t length);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);
void StartTask08(void *argument);
void StartTask09(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* Task08 owns the whole publish egress path:
   * myQueue04 -> protobuf/frame assembly -> USART1 DMA.
   */
  CDC_Data_Clear();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (2, sizeof(App_Uart_Tx_Item_t), &myQueue01_attributes);

  /* creation of myQueue02 */
  myQueue02Handle = osMessageQueueNew (16, sizeof(CAN_Msg_Queue_t), &myQueue02_attributes);

  /* creation of myQueue03 */
  myQueue03Handle = osMessageQueueNew (16, sizeof(CAN_Msg_Queue_t), &myQueue03_attributes);

  /* creation of myQueue04 */
  myQueue04Handle = osMessageQueueNew (16, sizeof(PublishQueueItem_t), &myQueue04_attributes);

  /* creation of myQueue05 */
  myQueue05Handle = osMessageQueueNew (8, sizeof(CAN_Tx_Queue_t), &myQueue05_attributes);

  /* creation of task01DebugQueue */
  task01DebugQueueHandle = osMessageQueueNew (2, sizeof(App_Uart_Tx_Item_t), &task01DebugQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(StartTask06, NULL, &myTask06_attributes);

  /* creation of myTask07 */
  myTask07Handle = osThreadNew(StartTask07, NULL, &myTask07_attributes);

  /* creation of myTask08 */
  myTask08Handle = osThreadNew(StartTask08, NULL, &myTask08_attributes);

  /* creation of myTask09 */
  myTask09Handle = osThreadNew(StartTask09, NULL, &myTask09_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Default task acts as the low-frequency service loop and health poller. */
  uint32_t mlx_health_tick_ms = HAL_GetTick();

  MLX90640_App_SetLogCallback(App_DebugLogString);
  (void)MLX90640_App_ServiceHealth();

  for(;;)
  {
    TaskDefault_ParseUsart3Message();
    Task01_SendUsart3DebugMessage();

    if ((HAL_GetTick() - mlx_health_tick_ms) >= 500U)
    {
      (void)MLX90640_App_ServiceHealth();
      mlx_health_tick_ms = HAL_GetTick();
    }

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Task02 consumes CAN1 frames and turns them into shared telemetry updates. */
  CAN_Msg_Queue_t recv_data;

  for(;;)
  {
    if(osMessageQueueGet(myQueue02Handle, &recv_data, NULL, osWaitForever) == osOK)
    {
      Task02_ParseCan1Message(&recv_data);
    }
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Task03 consumes CAN2 frames and updates the battery-box telemetry cache. */
  CAN_Msg_Queue_t recv_data;

  for(;;)
  {
    if(osMessageQueueGet(myQueue03Handle, &recv_data, NULL, osWaitForever) == osOK)
    {
      Task03_ParseCan2Message(&recv_data);
    }
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Task04 owns the MLX90640 sampling loop and thermal publish requests. */
  for(;;)
  {
    uint8_t thermal_summary_updated = 0U;

    for (uint8_t sensor_id = 0U; sensor_id < MLX90640_SENSOR_COUNT; sensor_id++)
    {
      if (MLX90640_App_IsSensorReady(sensor_id) == 0U)
      {
        continue;
      }

      int status = MLX90640_App_CaptureOnce(sensor_id);
      if (status == MLX90640_NO_ERROR)
      {
        Task04_SendMlxSummary(sensor_id);
        Task04_SendMlxCanSummary(sensor_id);
        Task04_SendMlxDebugMatrix(sensor_id, MLX90640_App_GetTempMap());
        thermal_summary_updated = 1U;
      }
      else
      {
        (void)snprintf(g_task04MlxLineBuffer, sizeof(g_task04MlxLineBuffer), "MLX90640 capture error, sensor=%u, status=%d\r\n", (unsigned int)sensor_id, status);
        App_DebugLogString(g_task04MlxLineBuffer);
      }

      osDelay(50);
    }

    if (thermal_summary_updated != 0U)
    {
      (void)Publish_QueueTopic(PUBLISH_TOPIC_THERMAL_SUMMARY);
    }

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    osDelay(125);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Task05 keeps USART2 DMA running and dispatches RS485 spans to the parser. */
  uint32_t flags = 0U;
  uint8_t dma_running = 0U;

  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
  g_task05Rs485DmaLastEventPos = 0U;
  g_task05Rs485DmaPendingBytes = 0U;
  g_task05Rs485DmaOverflow = 0U;
  g_task05Rs485DmaReadPos = 0U;

  for(;;)
  {
    if (dma_running == 0U)
    {
      if (App_Task05_StartRs485Dma() != HAL_OK)
      {
        App_DebugLogString("USART2 RX DMA start failed\r\n");
        osDelay(100);
        continue;
      }

      dma_running = 1U;
    }

    flags = osThreadFlagsWait(TASK05_FLAG_RS485_RX_READY | TASK05_FLAG_RS485_RX_RESTART,
                              osFlagsWaitAny,
                              osWaitForever);

    if ((flags & TASK05_FLAG_RS485_RX_READY) != 0U)
    {
      App_Task05_ProcessRs485DmaBuffer();
    }

    if ((flags & TASK05_FLAG_RS485_RX_RESTART) != 0U)
    {
      dma_running = 0U;
      App_DebugLogString("USART2 RX DMA restart\r\n");
    }
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Task06 initializes the controller-side MCP2518FD on SPI1 and polls it. */
  uint8_t fdcan_initialized = 0U;
  uint8_t fdcan_init_error_logged = 0U;
  Peripheral_Rx_Frame_t recv_data = {0};

  for(;;)
  {
    if (fdcan_initialized == 0U)
    {
      if (FDCAN_Init(FDCAN_BUS_A) == HAL_OK)
      {
        fdcan_initialized = 1U;
        fdcan_init_error_logged = 0U;
        App_DebugLogString("MCP2518FD A init ok\r\n");
      }
      else
      {
        if (fdcan_init_error_logged == 0U)
        {
          App_DebugLogString("MCP2518FD A init failed\r\n");
          fdcan_init_error_logged = 1U;
        }
        osDelay(250);
        continue;
      }
    }

    Task06_ParseSpi1Message(&recv_data);
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Task07 mirrors Task06 for the second MCP2518FD instance on SPI3. */
  uint8_t fdcan_initialized = 0U;
  uint8_t fdcan_init_error_logged = 0U;
  Peripheral_Rx_Frame_t recv_data = {0};

  for(;;)
  {
    if (fdcan_initialized == 0U)
    {
      if (FDCAN_Init(FDCAN_BUS_B) == HAL_OK)
      {
        fdcan_initialized = 1U;
        fdcan_init_error_logged = 0U;
        App_DebugLogString("MCP2518FD B init ok\r\n");
      }
      else
      {
        if (fdcan_init_error_logged == 0U)
        {
          App_DebugLogString("MCP2518FD B init failed\r\n");
          fdcan_init_error_logged = 1U;
        }
        osDelay(250);
        continue;
      }
    }

    Task07_ParseSpi3Message(&recv_data);
    osDelay(1);
  }
  /* USER CODE END StartTask07 */
}

/* USER CODE BEGIN Header_StartTask08 */
/**
* @brief Function implementing the myTask08 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask08 */
void StartTask08(void *argument)
{
  /* USER CODE BEGIN StartTask08 */
  /* Task08 is the only publish sender: build protobuf and send over USART1. */
  PublishQueueItem_t item;
  uint16_t frame_length = 0U;
  HAL_StatusTypeDef tx_status = HAL_OK;

  for(;;)
  {
    if (osMessageQueueGet(myQueue04Handle, &item, NULL, osWaitForever) == osOK)
    {
      Publish_OnTopicDequeued(item.topic);

      if (Publish_BuildFrame(item.topic,
                             g_task08PublishFrameBuffer,
                             (uint16_t)sizeof(g_task08PublishFrameBuffer),
                             &frame_length))
      {
        tx_status = App_UART_TransmitDmaBlocking(&huart1, g_task08PublishFrameBuffer, frame_length);
        if (tx_status == HAL_OK)
        {
          osDelay(TASK08_PUBLISH_COOLDOWN_MS);
        }
        else
        {
          (void)Publish_QueueTopic(item.topic);
        }
      }
      else
      {
        (void)Publish_QueueTopic(item.topic);
      }
    }
  }
  /* USER CODE END StartTask08 */
}

/* USER CODE BEGIN Header_StartTask09 */
/**
* @brief Function implementing the myTask09 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask09 */
void StartTask09(void *argument)
{
  /* USER CODE BEGIN StartTask09 */
  /* Task09 is the dedicated CAN1 transmit worker and nothing else. */
  CAN_Tx_Queue_t tx_data;

  for(;;)
  {
    if (osMessageQueueGet(myQueue05Handle, &tx_data, NULL, osWaitForever) == osOK)
    {
      Task09_SendCan1Message(&tx_data);
    }
  }
  /* USER CODE END StartTask09 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static osStatus_t App_QueueBytes(osMessageQueueId_t queue_handle, const uint8_t *data, uint16_t length, uint32_t timeout_ms)
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

  item->length = length;
  (void)memcpy(item->data, data, length);

  return osMessageQueuePut(queue_handle, item, 0U, timeout_ms);
}

static osStatus_t App_QueueText(osMessageQueueId_t queue_handle, const char *text, uint32_t timeout_ms)
{
  /* Text is just a byte buffer with a known length. */
  if (text == NULL)
  {
    return osErrorParameter;
  }

  return App_QueueBytes(queue_handle, (const uint8_t *)text, (uint16_t)strlen(text), timeout_ms);
}

static void App_DebugLogString(const char *text)
{
  /* All debug strings are funneled through the USART3 debug queue. */
  (void)App_QueueText(task01DebugQueueHandle, text, 250U);
}

static HAL_StatusTypeDef App_UART_TransmitDmaBlocking(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
  /* Block until the DMA transmitter is idle, then wait again for completion. */
  if ((huart == NULL) || (data == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  while ((HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX) ||
         (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX_RX))
  {
    osDelay(1);
  }

  HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, data, length);
  if (status != HAL_OK)
  {
    return status;
  }

  while ((HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX) ||
         (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX_RX))
  {
    osDelay(1);
  }

  return HAL_OK;
}

static HAL_StatusTypeDef App_Task05_StartRs485Dma(void)
{
  /* Restart ReceiveToIdle DMA and reset all ring-buffer tracking state. */
  (void)HAL_UART_DMAStop(&huart2);
  __HAL_UART_CLEAR_OREFLAG(&huart2);

  g_task05Rs485DmaLastEventPos = 0U;
  g_task05Rs485DmaPendingBytes = 0U;
  g_task05Rs485DmaOverflow = 0U;
  g_task05Rs485DmaReadPos = 0U;

  return HAL_UARTEx_ReceiveToIdle_DMA(&huart2,
                                      g_task05Rs485DmaBuffer,
                                      (uint16_t)sizeof(g_task05Rs485DmaBuffer));
}

static void App_Task05_ProcessRs485DmaBuffer(void)
{
  /* Drain the circular buffer in order and hand byte spans to the parser. */
  for (;;)
  {
    uint16_t pending_bytes = 0U;
    uint16_t contiguous_bytes = 0U;

    taskENTER_CRITICAL();
    pending_bytes = g_task05Rs485DmaPendingBytes;
    taskEXIT_CRITICAL();

    if (pending_bytes == 0U)
    {
      break;
    }

    contiguous_bytes = pending_bytes;
    if ((uint16_t)(sizeof(g_task05Rs485DmaBuffer) - g_task05Rs485DmaReadPos) < contiguous_bytes)
    {
      contiguous_bytes = (uint16_t)(sizeof(g_task05Rs485DmaBuffer) - g_task05Rs485DmaReadPos);
    }

    App_Task05_ProcessRs485Span(g_task05Rs485DmaReadPos,
                                (uint16_t)(g_task05Rs485DmaReadPos + contiguous_bytes));

    taskENTER_CRITICAL();
    if (g_task05Rs485DmaPendingBytes >= contiguous_bytes)
    {
      g_task05Rs485DmaPendingBytes = (uint16_t)(g_task05Rs485DmaPendingBytes - contiguous_bytes);
    }
    else
    {
      g_task05Rs485DmaPendingBytes = 0U;
    }
    g_task05Rs485DmaReadPos = (uint16_t)((g_task05Rs485DmaReadPos + contiguous_bytes) % sizeof(g_task05Rs485DmaBuffer));
    taskEXIT_CRITICAL();
  }

  if (g_task05Rs485DmaOverflow != 0U)
  {
    g_task05Rs485DmaOverflow = 0U;
    App_DebugLogString("USART2 RX DMA overflow\r\n");
  }
}

static void App_Task05_ProcessRs485Span(uint16_t start, uint16_t end)
{
  /* Split wrapped spans so the parser always sees a linear byte slice. */
  if (start == end)
  {
    return;
  }

  if (end > start)
  {
    App_Task05_DispatchRs485Bytes(&g_task05Rs485DmaBuffer[start], (uint16_t)(end - start));
    return;
  }

  App_Task05_DispatchRs485Bytes(&g_task05Rs485DmaBuffer[start],
                                (uint16_t)(sizeof(g_task05Rs485DmaBuffer) - start));
  if (end > 0U)
  {
    App_Task05_DispatchRs485Bytes(&g_task05Rs485DmaBuffer[0], end);
  }
}

static void App_Task05_DispatchRs485Bytes(const uint8_t *data, uint16_t length)
{
  /* Repackage arbitrary RS485 bytes into the generic receive-frame wrapper. */
  Peripheral_Rx_Frame_t recv_data = {0};

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  while (length > 0U)
  {
    uint16_t chunk = length;
    if (chunk > (uint16_t)sizeof(recv_data.data))
    {
      chunk = (uint16_t)sizeof(recv_data.data);
    }

    recv_data.length = chunk;
    (void)memcpy(recv_data.data, data, chunk);
    Task05_ParseRs485Message(&recv_data);

    data += chunk;
    length = (uint16_t)(length - chunk);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    HAL_UART_RxEventTypeTypeDef event_type = HAL_UARTEx_GetRxEventType(huart);
    uint16_t current_pos = Size;
    uint16_t delta = 0U;
    uint16_t pending = 0U;

    if (current_pos > (uint16_t)sizeof(g_task05Rs485DmaBuffer))
    {
      current_pos = (uint16_t)sizeof(g_task05Rs485DmaBuffer);
    }

    if (event_type == HAL_UART_RXEVENT_TC)
    {
      delta = (uint16_t)(sizeof(g_task05Rs485DmaBuffer) - g_task05Rs485DmaLastEventPos);
      current_pos = 0U;
    }
    else if (current_pos >= g_task05Rs485DmaLastEventPos)
    {
      delta = (uint16_t)(current_pos - g_task05Rs485DmaLastEventPos);
    }
    else
    {
      delta = (uint16_t)((sizeof(g_task05Rs485DmaBuffer) - g_task05Rs485DmaLastEventPos) + current_pos);
    }

    g_task05Rs485DmaLastEventPos = current_pos;
    pending = (uint16_t)(g_task05Rs485DmaPendingBytes + delta);
    if (pending > (uint16_t)sizeof(g_task05Rs485DmaBuffer))
    {
      g_task05Rs485DmaPendingBytes = (uint16_t)sizeof(g_task05Rs485DmaBuffer);
      g_task05Rs485DmaOverflow = 1U;
    }
    else
    {
      g_task05Rs485DmaPendingBytes = pending;
    }

    if (myTask05Handle != NULL)
    {
      (void)osThreadFlagsSet(myTask05Handle, TASK05_FLAG_RS485_RX_READY);
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    if (myTask05Handle != NULL)
    {
      (void)osThreadFlagsSet(myTask05Handle, TASK05_FLAG_RS485_RX_RESTART);
    }
  }
}

static void Task01_SendUsart3DebugMessage(void)
{
  if (osMessageQueueGet(task01DebugQueueHandle, &g_task01DebugTxItem, NULL, 0U) == osOK)
  {
    (void)App_UART_TransmitDmaBlocking(&huart3, g_task01DebugTxItem.data, g_task01DebugTxItem.length);
  }
}

static uint16_t App_AppendTemperature(char *buffer, uint16_t offset, uint16_t size, float temperature)
{
  int32_t centiDegrees;
  int32_t absValue;
  uint32_t integerPart;
  uint32_t fractionPart;
  char temp[16];
  uint32_t index = 0U;

  if (offset >= size)
  {
    return offset;
  }

  if (temperature >= 0.0f)
  {
    centiDegrees = (int32_t)(temperature * 100.0f + 0.5f);
  }
  else
  {
    centiDegrees = (int32_t)(temperature * 100.0f - 0.5f);
  }

  absValue = (centiDegrees < 0) ? -centiDegrees : centiDegrees;
  integerPart = (uint32_t)(absValue / 100);
  fractionPart = (uint32_t)(absValue % 100);

  if (centiDegrees < 0)
  {
    temp[index++] = '-';
  }

  if (integerPart >= 100U)
  {
    temp[index++] = (char)('0' + (integerPart / 100U) % 10U);
  }
  if (integerPart >= 10U)
  {
    temp[index++] = (char)('0' + (integerPart / 10U) % 10U);
  }
  temp[index++] = (char)('0' + (integerPart % 10U));
  temp[index++] = '.';
  temp[index++] = (char)('0' + (fractionPart / 10U));
  temp[index++] = (char)('0' + (fractionPart % 10U));
  temp[index++] = ' ';

  for (uint32_t copyIndex = 0U; (copyIndex < index) && (offset < (size - 1U)); copyIndex++)
  {
    buffer[offset++] = temp[copyIndex];
  }

  buffer[offset] = '\0';
  return offset;
}

static void Task04_SendMlxDebugMatrix(uint8_t sensor_id, const float *temp_map)
{
  char *line = g_task04MlxLineBuffer;

  if (temp_map == NULL)
  {
    return;
  }

  (void)snprintf(line, TASK04_MLX_LINE_BUFFER_SIZE, "----- Sensor %u Temperature Matrix -----\r\n", (unsigned int)sensor_id);
  App_DebugLogString(line);

  for (uint16_t row = 0U; row < MLX90640_LINE_NUM; row++)
  {
    uint16_t lineOffset = 0U;

    for (uint16_t col = 0U; col < MLX90640_COLUMN_NUM; col++)
    {
      uint16_t pixelIndex = (uint16_t)(row * MLX90640_COLUMN_NUM + col);
      lineOffset = App_AppendTemperature(line, lineOffset, TASK04_MLX_LINE_BUFFER_SIZE, temp_map[pixelIndex]);
    }

    if (lineOffset < (TASK04_MLX_LINE_BUFFER_SIZE - 2U))
    {
      line[lineOffset++] = '\r';
      line[lineOffset++] = '\n';
    }
    line[lineOffset] = '\0';
    App_DebugLogString(line);
  }

  (void)snprintf(line, TASK04_MLX_LINE_BUFFER_SIZE, "----- Sensor %u Temperature Matrix End -----\r\n", (unsigned int)sensor_id);
  App_DebugLogString(line);
}

static void Task04_SendMlxSummary(uint8_t sensor_id)
{
  char *line = g_task04MlxLineBuffer;

  (void)snprintf(line, APP_UART_TX_MAX_PAYLOAD,
      "MLX90640,S=%u,Ta=%ld,Tr=%ld,R0=%ld,R1=%ld,R2=%ld,R3=%ld,C=%lu\r\n",
      (unsigned int)sensor_id,
      (long)g_MLX90640_Frame.Ta[sensor_id],
      (long)g_MLX90640_Frame.Tr[sensor_id],
      (long)g_MLX90640_Frame.RegionTemp[sensor_id][0],
      (long)g_MLX90640_Frame.RegionTemp[sensor_id][1],
      (long)g_MLX90640_Frame.RegionTemp[sensor_id][2],
      (long)g_MLX90640_Frame.RegionTemp[sensor_id][3],
      (unsigned long)g_MLX90640_Frame.FrameCounter[sensor_id]);

  App_DebugLogString(line);
}

static void Task04_SendMlxCanSummary(uint8_t sensor_id)
{
  CAN_Tx_Queue_t tx_data = {0};

  tx_data.std_id = Task04_GetMlxCanSummaryId(sensor_id);
  if (tx_data.std_id == 0U)
  {
    return;
  }

  for (uint8_t region_index = 0U; region_index < MLX90640_REGION_COUNT; region_index++)
  {
    Task04_EncodeCanSummaryPair(
        g_MLX90640_Frame.RegionTemp[sensor_id][region_index],
        &tx_data.data[region_index * 2U],
        &tx_data.data[(region_index * 2U) + 1U]);
  }

  (void)freertos_can1_tx_queue_put(&tx_data, 0U);
}

static uint32_t Task04_GetMlxCanSummaryId(uint8_t sensor_id)
{
  switch (sensor_id)
  {
    case 0U:
      return CAN_ID_CANB_THERMAL_SUMMARY_FL;
    case 1U:
      return CAN_ID_CANB_THERMAL_SUMMARY_FR;
    case 2U:
      return CAN_ID_CANB_THERMAL_SUMMARY_RL;
    case 3U:
      return CAN_ID_CANB_THERMAL_SUMMARY_RR;
    default:
      return 0U;
  }
}

static void Task04_EncodeCanSummaryPair(int32_t centi_c, uint8_t *integer_part, uint8_t *fraction_part)
{
  uint32_t value;

  if ((integer_part == NULL) || (fraction_part == NULL))
  {
    return;
  }

  if (centi_c < 0)
  {
    value = 0U;
  }
  else
  {
    value = (uint32_t)centi_c;
  }

  *integer_part = (uint8_t)(((value / 100U) > 255U) ? 255U : (value / 100U));
  *fraction_part = (uint8_t)(value % 100U);
}
osStatus_t freertos_can_queue_send_from_isr(const CAN_Msg_Queue_t *pData)
{
  osMessageQueueId_t queue_handle = NULL;

  if(pData == NULL)
  {
    return osErrorParameter;
  }

  if(pData->can_channel == CAN_CHANNEL_VEHICLE_CANB)
  {
    queue_handle = myQueue02Handle;
  }
  else if(pData->can_channel == CAN_CHANNEL_BATTERY_BOX)
  {
    queue_handle = myQueue03Handle;
  }

  if(queue_handle == NULL)
  {
    return osErrorParameter;
  }

  QueueHandle_t xQueue = (QueueHandle_t)queue_handle;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  BaseType_t xResult = xQueueSendFromISR(xQueue, pData, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

  return (xResult == pdPASS) ? osOK : osError;
}

osStatus_t freertos_can1_tx_queue_put(const CAN_Tx_Queue_t *pData, uint32_t timeout_ms)
{
  if(myQueue05Handle == NULL || pData == NULL)
  {
    return osErrorParameter;
  }

  return osMessageQueuePut(myQueue05Handle, pData, 0U, timeout_ms);
}

static void TaskSpiPollCan(const Peripheral_Rx_Frame_t *recv_data, FDCAN_Bus_t bus, osMessageQueueId_t queue_handle, uint8_t can_channel)
{
  CAN_Msg_Queue_t queue_item;
  FDCAN_StdFrame_t rx_frame;
  uint8_t received = 0U;

  if ((recv_data == NULL) || (queue_handle == NULL) || (FDCAN_IsReady(bus) == 0U))
  {
    return;
  }

  do
  {
    if (FDCAN_Poll(bus, &rx_frame, &received) != HAL_OK || (received == 0U))
    {
      return;
    }

    if ((rx_frame.frame_format != FDCAN_FRAME_CLASSIC) || (rx_frame.length > 8U))
    {
      continue;
    }

    queue_item.can_channel = can_channel;
    queue_item.msg_id = rx_frame.std_id;
    queue_item.dlc = rx_frame.length;
    queue_item.is_ext_id = 0U;
    (void)memset(queue_item.msg_data, 0, sizeof(queue_item.msg_data));
    (void)memcpy(queue_item.msg_data, rx_frame.data, rx_frame.length);
    (void)osMessageQueuePut(queue_handle, &queue_item, 0U, 0U);
  } while (received != 0U);
}

static bool TaskSpiPollControllerCana(const Peripheral_Rx_Frame_t *recv_data, FDCAN_Bus_t bus)
{
  CAN_Msg_Queue_t frame_data;
  FDCAN_StdFrame_t rx_frame;
  uint8_t received = 0U;
  bool decoded_any = false;

  if ((recv_data == NULL) || (FDCAN_IsReady(bus) == 0U))
  {
    return false;
  }

  do
  {
    if (FDCAN_Poll(bus, &rx_frame, &received) != HAL_OK || (received == 0U))
    {
      return decoded_any;
    }

    if ((rx_frame.frame_format != FDCAN_FRAME_CLASSIC) || (rx_frame.length > 8U))
    {
      continue;
    }

    frame_data.can_channel = CAN_CHANNEL_CONTROLLER_CANA;
    frame_data.msg_id = rx_frame.std_id;
    frame_data.dlc = rx_frame.length;
    frame_data.is_ext_id = 0U;
    (void)memset(frame_data.msg_data, 0, sizeof(frame_data.msg_data));
    (void)memcpy(frame_data.msg_data, rx_frame.data, rx_frame.length);
    CAN_DecodeControllerCanaMessage(&frame_data);
    decoded_any = true;
  } while (received != 0U);

  return decoded_any;
}

static void TaskSpiPollCdcMonitor(const Peripheral_Rx_Frame_t *recv_data, FDCAN_Bus_t bus)
{
  CAN_Msg_Queue_t frame_data;
  FDCAN_StdFrame_t rx_frame;
  uint8_t received = 0U;

  if ((recv_data == NULL) || (FDCAN_IsReady(bus) == 0U))
  {
    return;
  }

  do
  {
    if (FDCAN_Poll(bus, &rx_frame, &received) != HAL_OK || (received == 0U))
    {
      return;
    }

    if ((rx_frame.frame_format != FDCAN_FRAME_CLASSIC) || (rx_frame.length > 8U))
    {
      continue;
    }

    frame_data.can_channel = CAN_CHANNEL_CDC_MONITOR;
    frame_data.msg_id = rx_frame.std_id;
    frame_data.dlc = rx_frame.length;
    frame_data.is_ext_id = 0U;
    (void)memset(frame_data.msg_data, 0, sizeof(frame_data.msg_data));
    (void)memcpy(frame_data.msg_data, rx_frame.data, rx_frame.length);
    CAN_DecodeCdcMonitorMessage(&frame_data);
  } while (received != 0U);
}

static void TaskDefault_ParseUsart3Message(void)
{
  /* USART3 parser entry. */
}

static void Task02_ParseCan1Message(const CAN_Msg_Queue_t *recv_data)
{
  if (recv_data == NULL)
  {
    return;
  }

  if (recv_data->can_channel == CAN_CHANNEL_VEHICLE_CANB)
  {
    CAN_DecodeVehicleCanbMessage(recv_data);
    (void)Publish_QueueTopic(PUBLISH_TOPIC_FAST_TELEMETRY);
    (void)Publish_QueueTopic(PUBLISH_TOPIC_VEHICLE_STATE);
    (void)Publish_QueueTopic(PUBLISH_TOPIC_BMS_SUMMARY);
  }
}

static void Task03_ParseCan2Message(const CAN_Msg_Queue_t *recv_data)
{
  CAN_DecodeBatteryBoxMessage(recv_data);
  (void)Publish_QueueTopic(PUBLISH_TOPIC_FAST_TELEMETRY);
  (void)Publish_QueueTopic(PUBLISH_TOPIC_BMS_SUMMARY);
  (void)Publish_QueueTopic(PUBLISH_TOPIC_BMS_DETAIL);
}

static void Task05_ParseRs485Message(const Peripheral_Rx_Frame_t *recv_data)
{
  (void)recv_data;
  /* USART2 + EN# RS485 parser entry. */
}

static void Task06_ParseSpi1Message(const Peripheral_Rx_Frame_t *recv_data)
{
  if (TaskSpiPollControllerCana(recv_data, FDCAN_BUS_A))
  {
    (void)Publish_QueueTopic(PUBLISH_TOPIC_VEHICLE_STATE);
    (void)Publish_QueueTopic(PUBLISH_TOPIC_BMS_SUMMARY);
  }
}

static void Task07_ParseSpi3Message(const Peripheral_Rx_Frame_t *recv_data)
{
  TaskSpiPollCdcMonitor(recv_data, FDCAN_BUS_B);
}

static void Task09_SendCan1Message(const CAN_Tx_Queue_t *tx_data)
{
  if (tx_data == NULL)
  {
    return;
  }

  CAN_Send_Msg(&hcan1, tx_data->std_id, (uint8_t *)tx_data->data);
}
/* USER CODE END Application */

