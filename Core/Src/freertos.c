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
#include "battery.h"
#include "can.h"
#include "gpio.h"
#include "mlx90640_app.h"
#include "MLX90640_API.h"
#include "publish.h"
#include "spi.h"
#include "usart.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint16_t length;
  uint8_t data[64];
} Peripheral_Rx_Frame_t;

typedef struct
{
  uint16_t length;
  uint8_t data[288U];
} App_Uart_Tx_Item_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_UART_TX_MAX_PAYLOAD        288U
#define TASK01_DEBUG_QUEUE_DEPTH       2U
#define TASK08_USART1_QUEUE_DEPTH      2U
#define TASK04_MLX_LINE_BUFFER_SIZE    APP_UART_TX_MAX_PAYLOAD
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static App_Uart_Tx_Item_t g_task01DebugTxItem;
static App_Uart_Tx_Item_t g_task08Usart1TxItem;
static App_Uart_Tx_Item_t g_uartQueueWorkItem;
static char g_task04MlxLineBuffer[APP_UART_TX_MAX_PAYLOAD];
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
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
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
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask09 */
osThreadId_t myTask09Handle;
const osThreadAttr_t myTask09_attributes = {
  .name = "myTask09",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
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
static void Task08_SendUsart1Message(const App_Uart_Tx_Item_t *tx_data);
static void Task09_SendCan1Message(const CAN_Tx_Queue_t *tx_data);
static osStatus_t App_QueueBytes(osMessageQueueId_t queue_handle, const uint8_t *data, uint16_t length, uint32_t timeout_ms);
static osStatus_t App_QueueText(osMessageQueueId_t queue_handle, const char *text, uint32_t timeout_ms);
static void App_DebugLogString(const char *text);
static HAL_StatusTypeDef App_UART_TransmitDmaBlocking(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);
static uint16_t App_AppendTemperature(char *buffer, uint16_t offset, uint16_t size, float temperature);
static void Task04_SendMlxDebugMatrix(uint8_t sensor_id, const float *temp_map);
static void Task04_SendMlxSummary(uint8_t sensor_id);
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
  /* Publish is intentionally bypassed while task08 sends queued USART1 DMA frames. */
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
  myQueue01Handle = osMessageQueueNew (TASK08_USART1_QUEUE_DEPTH, sizeof(App_Uart_Tx_Item_t), &myQueue01_attributes);

  /* creation of myQueue02 */
  myQueue02Handle = osMessageQueueNew (16, sizeof(CAN_Msg_Queue_t), &myQueue02_attributes);

  /* creation of myQueue03 */
  myQueue03Handle = osMessageQueueNew (16, sizeof(CAN_Msg_Queue_t), &myQueue03_attributes);

  /* creation of myQueue04 */
  myQueue04Handle = osMessageQueueNew (16, sizeof(PublishQueueItem_t), &myQueue04_attributes);

  /* creation of myQueue05 */
  myQueue05Handle = osMessageQueueNew (8, sizeof(CAN_Tx_Queue_t), &myQueue05_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  task01DebugQueueHandle = osMessageQueueNew (TASK01_DEBUG_QUEUE_DEPTH, sizeof(App_Uart_Tx_Item_t), &task01DebugQueue_attributes);
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
  for(;;)
  {
    TaskDefault_ParseUsart3Message();
    Task01_SendUsart3DebugMessage();
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
  MLX90640_App_SetLogCallback(App_DebugLogString);
  (void)MLX90640_App_Init();

  for(;;)
  {
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
        Task04_SendMlxDebugMatrix(sensor_id, MLX90640_App_GetTempMap());
      }
      else
      {
        (void)snprintf(g_task04MlxLineBuffer, sizeof(g_task04MlxLineBuffer), "MLX90640 capture error, sensor=%u, status=%d\r\n", (unsigned int)sensor_id, status);
        App_DebugLogString(g_task04MlxLineBuffer);
      }

      osDelay(50);
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
  uint8_t rx_byte = 0U;
  Peripheral_Rx_Frame_t recv_data = {0};

  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);

  for(;;)
  {
    if (HAL_UART_Receive(&huart2, &rx_byte, 1U, 20U) == HAL_OK)
    {
      recv_data.length = 1U;
      recv_data.data[0] = rx_byte;
      Task05_ParseRs485Message(&recv_data);
    }
    osDelay(1);
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
  Peripheral_Rx_Frame_t recv_data = {0};

  for(;;)
  {
    Task06_ParseSpi1Message(&recv_data);
    osDelay(10);
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
  Peripheral_Rx_Frame_t recv_data = {0};

  for(;;)
  {
    Task07_ParseSpi3Message(&recv_data);
    osDelay(10);
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
  for(;;)
  {
    if (osMessageQueueGet(myQueue01Handle, &g_task08Usart1TxItem, NULL, osWaitForever) == osOK)
    {
      Task08_SendUsart1Message(&g_task08Usart1TxItem);
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
  if (text == NULL)
  {
    return osErrorParameter;
  }

  return App_QueueBytes(queue_handle, (const uint8_t *)text, (uint16_t)strlen(text), timeout_ms);
}

static void App_DebugLogString(const char *text)
{
  (void)App_QueueText(task01DebugQueueHandle, text, 250U);
}

static HAL_StatusTypeDef App_UART_TransmitDmaBlocking(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
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

  (void)snprintf(line, sizeof(line), "----- Sensor %u Temperature Matrix -----\r\n", (unsigned int)sensor_id);
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

  (void)snprintf(line, sizeof(line), "----- Sensor %u Temperature Matrix End -----\r\n", (unsigned int)sensor_id);
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

  (void)App_QueueText(myQueue01Handle, line, 250U);
}
osStatus_t freertos_can_queue_send_from_isr(const CAN_Msg_Queue_t *pData)
{
  osMessageQueueId_t queue_handle = NULL;

  if(pData == NULL)
  {
    return osErrorParameter;
  }

  if(pData->can_channel == 1U)
  {
    queue_handle = myQueue02Handle;
  }
  else if(pData->can_channel == 2U)
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

static void TaskDefault_ParseUsart3Message(void)
{
  /* USART3 parser entry. */
}

static void Task02_ParseCan1Message(const CAN_Msg_Queue_t *recv_data)
{
  if (recv_data == NULL || recv_data->can_channel != 1U || recv_data->is_ext_id != 0U)
  {
    return;
  }

  switch (recv_data->msg_id)
  {
    case 0x401:
      break;

    case 0x501:
      g_CANB_LoopData.ECU.Vehicle_Speed = (int)recv_data->msg_data[0];
      g_CANB_LoopData.IMU.Long_Accel = (int)recv_data->msg_data[1];
      g_CANB_LoopData.IMU.Lat_Accel = (int)recv_data->msg_data[2];
      for (int i = 0; i < 4; i++)
      {
        g_CANB_LoopData.ECU.Motor_Torque[i] = (int)recv_data->msg_data[i + 3];
      }
      g_CANB_LoopData.ECU.driving_mode = (int)recv_data->msg_data[7];
      break;

    case 0x502:
      for (int i = 0; i < 4; i++)
      {
        g_CANB_LoopData.ECU.Motor_RPM[i] = (int)recv_data->msg_data[i];
        g_CANB_LoopData.ECU.Motor_Power[i] = (256 * (int)recv_data->msg_data[2 * i + 1] + (int)recv_data->msg_data[2 * i]) / 9549;
      }
      break;

    case 0x503:
      for (int i = 0; i < 4; i++)
      {
        g_CANB_LoopData.ECU.ERRO[i] = (256 * (int)recv_data->msg_data[2 * i] + (int)recv_data->msg_data[2 * i + 1]);
      }
      break;

    case 0x50:
      break;

    default:
      break;
  }
}

static void Task03_ParseCan2Message(const CAN_Msg_Queue_t *recv_data)
{
  if (recv_data == NULL || recv_data->can_channel != 2U || recv_data->is_ext_id != 1U)
  {
    return;
  }

  if (recv_data->msg_id >= 0x180050F3 && recv_data->msg_id <= 0x184550F3)
  {
    int module_id = 0;
    if (recv_data->msg_id >= 0x184050F3)
    {
      module_id = (int)((recv_data->msg_id >> 16) & 0x000F);
      for (int i = 0; i < BAT_TEMP_POINT_PER_MOD; i++)
      {
        g_BatteryInfo.ModTemp[module_id][i] = (int8_t)(recv_data->msg_data[i] - BAT_TEMP_OFFSET);
      }
    }
    else
    {
      int middle = (int)((recv_data->msg_id - 0x180050F3) >> 16);
      module_id = middle / 6;
      switch (middle % 6)
      {
        case 0:
          for (int i = 0; i <= 3; i++)
            g_BatteryInfo.CellVolt[module_id][i] = (uint16_t)(recv_data->msg_data[2 * i] + recv_data->msg_data[2 * i + 1] * 256);
          break;
        case 1:
          for (int i = 0; i <= 3; i++)
            g_BatteryInfo.CellVolt[module_id][i + 4] = (uint16_t)(recv_data->msg_data[2 * i] + recv_data->msg_data[2 * i + 1] * 256);
          break;
        case 2:
          for (int i = 0; i <= 3; i++)
            g_BatteryInfo.CellVolt[module_id][i + 8] = (uint16_t)(recv_data->msg_data[2 * i] + recv_data->msg_data[2 * i + 1] * 256);
          break;
        case 3:
          for (int i = 0; i <= 3; i++)
            g_BatteryInfo.CellVolt[module_id][i + 12] = (uint16_t)(recv_data->msg_data[2 * i] + recv_data->msg_data[2 * i + 1] * 256);
          break;
        case 4:
          for (int i = 0; i <= 3; i++)
            g_BatteryInfo.CellVolt[module_id][i + 16] = (uint16_t)(recv_data->msg_data[2 * i] + recv_data->msg_data[2 * i + 1] * 256);
          break;
        case 5:
          for (int i = 0; i <= 3; i++)
            g_BatteryInfo.CellVolt[module_id][i + 20] = (uint16_t)(recv_data->msg_data[2 * i] + recv_data->msg_data[2 * i + 1] * 256);
          break;
        default:
          break;
      }
    }
  }

  if (recv_data->msg_id >= 0x186050F4 && recv_data->msg_id <= 0x186350F4)
  {
    switch (recv_data->msg_id)
    {
      case 0x186050F4:
        g_BatteryInfo.TotalVolt = (uint16_t)(recv_data->msg_data[0] * 256 + recv_data->msg_data[1]);
        g_BatteryInfo.TotalCurrent = (int16_t)(recv_data->msg_data[2] * 256 + recv_data->msg_data[3]);
        g_BatteryInfo.SOC = recv_data->msg_data[4];
        g_BatteryInfo.IMD_State = recv_data->msg_data[5];
        g_BatteryInfo.Work_State = (uint8_t)(((recv_data->msg_data[6] & 0xF0) >> 4) - 2);
        if (g_BatteryInfo.Work_State == 3)
        {
          g_BatteryInfo.Work_State = g_BatteryInfo.Charge_Enable ? 4 : 3;
        }
        break;

      case 0x186150F4:
        g_BatteryInfo.MaxCellVolt = (uint16_t)(256 * recv_data->msg_data[0] + recv_data->msg_data[1]);
        g_BatteryInfo.MinCellVolt = (uint16_t)(256 * recv_data->msg_data[2] + recv_data->msg_data[3]);
        break;

      case 0x186250F4:
        g_BatteryInfo.MaxTemp = (int8_t)(recv_data->msg_data[0] - BAT_TEMP_OFFSET);
        g_BatteryInfo.MinTemp = (int8_t)(recv_data->msg_data[1] - BAT_TEMP_OFFSET);
        break;

      case 0x186350F4:
        g_BatteryInfo.Air1_State = (uint8_t)((recv_data->msg_data[0] & 0x0C) >> 2);
        g_BatteryInfo.Air2_State = (uint8_t)((recv_data->msg_data[0] & 0x30) >> 4);
        g_BatteryInfo.Air3_State = (uint8_t)((recv_data->msg_data[0] & 0xC0) >> 6);
        g_BatteryInfo.Charge_Enable = (uint8_t)((recv_data->msg_data[1] & 0x3F) >> 5);
        g_BatteryInfo.Charge_ReqVolt = (uint16_t)(256 * recv_data->msg_data[2] + recv_data->msg_data[3]);
        g_BatteryInfo.Charge_ReqCurr = (int16_t)((256 * recv_data->msg_data[4] + recv_data->msg_data[5]) * CHARGER_CURR_SCALE);
        g_BatteryInfo.Charger_OutVolt = (uint16_t)(256 * recv_data->msg_data[6] + recv_data->msg_data[7]);
        break;

      default:
        break;
    }
  }
}

static void Task05_ParseRs485Message(const Peripheral_Rx_Frame_t *recv_data)
{
  (void)recv_data;
  /* USART2 + EN# RS485 parser entry. */
}

static void Task06_ParseSpi1Message(const Peripheral_Rx_Frame_t *recv_data)
{
  (void)recv_data;
  (void)hspi1;
  /* SPI1 parser entry. */
}

static void Task07_ParseSpi3Message(const Peripheral_Rx_Frame_t *recv_data)
{
  (void)recv_data;
  (void)hspi3;
  /* SPI3 parser entry. */
}

static void Task08_SendUsart1Message(const App_Uart_Tx_Item_t *tx_data)
{
  if (tx_data == NULL)
  {
    return;
  }

  (void)App_UART_TransmitDmaBlocking(&huart1, (uint8_t *)tx_data->data, tx_data->length);
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





