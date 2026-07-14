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
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static App_Uart_Tx_Item_t g_debugTxItem;
static App_Uart_Tx_Item_t g_uartQueueWorkItem;
static char g_mlxLineBuffer[APP_UART_TX_MAX_PAYLOAD];
static uint8_t g_rs485DmaBuffer[RS485_DMA_BUFFER_SIZE];
static volatile uint16_t g_rs485DmaLastEventPos = 0U;
static volatile uint16_t g_rs485DmaPendingBytes = 0U;
static volatile uint8_t g_rs485DmaOverflow = 0U;
static uint16_t g_rs485DmaReadPos = 0U;
static uint8_t g_publishFrameBuffer[PUBLISH_MAX_FRAME_SIZE];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 192 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanForViehcle */
osThreadId_t CanForViehcleHandle;
uint32_t CanForViehcleBuffer[ 128 ];
osStaticThreadDef_t CanForViehcleControlBlock;
const osThreadAttr_t CanForViehcle_attributes = {
  .name = "CanForViehcle",
  .cb_mem = &CanForViehcleControlBlock,
  .cb_size = sizeof(CanForViehcleControlBlock),
  .stack_mem = &CanForViehcleBuffer[0],
  .stack_size = sizeof(CanForViehcleBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CanForBMS */
osThreadId_t CanForBMSHandle;
uint32_t CanForBMSTaskBuffer[ 128 ];
osStaticThreadDef_t CanForBMSTaskControlBlock;
const osThreadAttr_t CanForBMS_attributes = {
  .name = "CanForBMS",
  .cb_mem = &CanForBMSTaskControlBlock,
  .cb_size = sizeof(CanForBMSTaskControlBlock),
  .stack_mem = &CanForBMSTaskBuffer[0],
  .stack_size = sizeof(CanForBMSTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MLX90640 */
osThreadId_t MLX90640Handle;
uint32_t MLX90640Buffer[ 384 ];
osStaticThreadDef_t MLX90640ControlBlock;
const osThreadAttr_t MLX90640_attributes = {
  .name = "MLX90640",
  .cb_mem = &MLX90640ControlBlock,
  .cb_size = sizeof(MLX90640ControlBlock),
  .stack_mem = &MLX90640Buffer[0],
  .stack_size = sizeof(MLX90640Buffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TensionSensor */
osThreadId_t TensionSensorHandle;
uint32_t TensionSensorBuffer[ 128 ];
osStaticThreadDef_t TensionSensorControlBlock;
const osThreadAttr_t TensionSensor_attributes = {
  .name = "TensionSensor",
  .cb_mem = &TensionSensorControlBlock,
  .cb_size = sizeof(TensionSensorControlBlock),
  .stack_mem = &TensionSensorBuffer[0],
  .stack_size = sizeof(TensionSensorBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SPICanControl */
osThreadId_t SPICanControlHandle;
uint32_t SPICanControlBuffer[ 128 ];
osStaticThreadDef_t SPICanControlControlBlock;
const osThreadAttr_t SPICanControl_attributes = {
  .name = "SPICanControl",
  .cb_mem = &SPICanControlControlBlock,
  .cb_size = sizeof(SPICanControlControlBlock),
  .stack_mem = &SPICanControlBuffer[0],
  .stack_size = sizeof(SPICanControlBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for SPICanCDC */
osThreadId_t SPICanCDCHandle;
uint32_t SPICanCDCBuffer[ 128 ];
osStaticThreadDef_t SPICanCDCControlBlock;
const osThreadAttr_t SPICanCDC_attributes = {
  .name = "SPICanCDC",
  .cb_mem = &SPICanCDCControlBlock,
  .cb_size = sizeof(SPICanCDCControlBlock),
  .stack_mem = &SPICanCDCBuffer[0],
  .stack_size = sizeof(SPICanCDCBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Publish4G */
osThreadId_t Publish4GHandle;
uint32_t Publish4GBuffer[ 192 ];
osStaticThreadDef_t Publish4GControlBlock;
const osThreadAttr_t Publish4G_attributes = {
  .name = "Publish4G",
  .cb_mem = &Publish4GControlBlock,
  .cb_size = sizeof(Publish4GControlBlock),
  .stack_mem = &Publish4GBuffer[0],
  .stack_size = sizeof(Publish4GBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for CanBText */
osThreadId_t CanBTextHandle;
uint32_t CanBTextBuffer[ 128 ];
osStaticThreadDef_t CanBTextControlBlock;
const osThreadAttr_t CanBText_attributes = {
  .name = "CanBText",
  .cb_mem = &CanBTextControlBlock,
  .cb_size = sizeof(CanBTextControlBlock),
  .stack_mem = &CanBTextBuffer[0],
  .stack_size = sizeof(CanBTextBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for initTaskBoot */
osThreadId_t initTaskBootHandle;
uint32_t initTaskBootBuffer[ 384 ];
osStaticThreadDef_t initTaskBootControlBlock;
const osThreadAttr_t initTaskBoot_attributes = {
  .name = "initTaskBoot",
  .cb_mem = &initTaskBootControlBlock,
  .cb_size = sizeof(initTaskBootControlBlock),
  .stack_mem = &initTaskBootBuffer[0],
  .stack_size = sizeof(initTaskBootBuffer),
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for App_Uart_Tx */
osMessageQueueId_t App_Uart_TxHandle;
uint8_t App_Uart_TxBuffer[ 2 * sizeof( App_Uart_Tx_Item_t ) ];
osStaticMessageQDef_t App_Uart_TxControlBlock;
const osMessageQueueAttr_t App_Uart_Tx_attributes = {
  .name = "App_Uart_Tx",
  .cb_mem = &App_Uart_TxControlBlock,
  .cb_size = sizeof(App_Uart_TxControlBlock),
  .mq_mem = &App_Uart_TxBuffer,
  .mq_size = sizeof(App_Uart_TxBuffer)
};
/* Definitions for CAN_Msg_Queue */
osMessageQueueId_t CAN_Msg_QueueHandle;
uint8_t CAN_Msg_QueueBuffer[ 16 * sizeof( CAN_Msg_Queue_t ) ];
osStaticMessageQDef_t CAN_Msg_QueueControlBlock;
const osMessageQueueAttr_t CAN_Msg_Queue_attributes = {
  .name = "CAN_Msg_Queue",
  .cb_mem = &CAN_Msg_QueueControlBlock,
  .cb_size = sizeof(CAN_Msg_QueueControlBlock),
  .mq_mem = &CAN_Msg_QueueBuffer,
  .mq_size = sizeof(CAN_Msg_QueueBuffer)
};
/* Definitions for CAN_Msg_Queue_2 */
osMessageQueueId_t CAN_Msg_Queue_2Handle;
uint8_t CAN_Msg_Queue_2Buffer[ 16 * sizeof( CAN_Msg_Queue_t ) ];
osStaticMessageQDef_t CAN_Msg_Queue_2ControlBlock;
const osMessageQueueAttr_t CAN_Msg_Queue_2_attributes = {
  .name = "CAN_Msg_Queue_2",
  .cb_mem = &CAN_Msg_Queue_2ControlBlock,
  .cb_size = sizeof(CAN_Msg_Queue_2ControlBlock),
  .mq_mem = &CAN_Msg_Queue_2Buffer,
  .mq_size = sizeof(CAN_Msg_Queue_2Buffer)
};
/* Definitions for PublishQueueItem */
osMessageQueueId_t PublishQueueItemHandle;
uint8_t PublishQueueItemBuffer[ 16 * sizeof( PublishQueueItem_t ) ];
osStaticMessageQDef_t PublishQueueItemControlBlock;
const osMessageQueueAttr_t PublishQueueItem_attributes = {
  .name = "PublishQueueItem",
  .cb_mem = &PublishQueueItemControlBlock,
  .cb_size = sizeof(PublishQueueItemControlBlock),
  .mq_mem = &PublishQueueItemBuffer,
  .mq_size = sizeof(PublishQueueItemBuffer)
};
/* Definitions for CAN_Tx_Queue */
osMessageQueueId_t CAN_Tx_QueueHandle;
uint8_t CAN_Tx_QueueBuffer[ 8 * sizeof( CAN_Tx_Queue_t ) ];
osStaticMessageQDef_t CAN_Tx_QueueControlBlock;
const osMessageQueueAttr_t CAN_Tx_Queue_attributes = {
  .name = "CAN_Tx_Queue",
  .cb_mem = &CAN_Tx_QueueControlBlock,
  .cb_size = sizeof(CAN_Tx_QueueControlBlock),
  .mq_mem = &CAN_Tx_QueueBuffer,
  .mq_size = sizeof(CAN_Tx_QueueBuffer)
};
/* Definitions for task01DebugQueue */
osMessageQueueId_t task01DebugQueueHandle;
uint8_t task01DebugQueueBuffer[ 2 * sizeof( App_Uart_Tx_Item_t ) ];
osStaticMessageQDef_t task01DebugQueueControlBlock;
const osMessageQueueAttr_t task01DebugQueue_attributes = {
  .name = "task01DebugQueue",
  .cb_mem = &task01DebugQueueControlBlock,
  .cb_size = sizeof(task01DebugQueueControlBlock),
  .mq_mem = &task01DebugQueueBuffer,
  .mq_size = sizeof(task01DebugQueueBuffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void Usart3_ParseInput(void);
static void Usart3_SendDebug(void);
static void CanVehicle_Dispatch(const CAN_Msg_Queue_t *recv_data);
static void CanBattery_Dispatch(const CAN_Msg_Queue_t *recv_data);
static void Rs485_Dispatch(const Peripheral_Rx_Frame_t *recv_data);
static void SpiCana_Dispatch(const Peripheral_Rx_Frame_t *recv_data);
static void SpiCdc_Dispatch(const Peripheral_Rx_Frame_t *recv_data);
static void Can1_Send(const CAN_Tx_Queue_t *tx_data);
static osStatus_t App_QueueBytes(osMessageQueueId_t queue_handle, const uint8_t *data, uint16_t length, uint32_t timeout_ms);
static osStatus_t App_QueueText(osMessageQueueId_t queue_handle, const char *text, uint32_t timeout_ms);
static void App_DebugLogString(const char *text);
static HAL_StatusTypeDef App_UART_TransmitDmaBlocking(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);
static uint16_t App_AppendTemperature(char *buffer, uint16_t offset, uint16_t size, float temperature);
static void Mlx_SendDebugMatrix(uint8_t sensor_id, const float *temp_map);
static void Mlx_SendSummary(uint8_t sensor_id);
static void Mlx_SendCanSummary(uint8_t sensor_id);
static uint32_t Mlx_GetCanSummaryId(uint8_t sensor_id);
static void Mlx_EncodeCanSummaryPair(int32_t centi_c, uint8_t *integer_part, uint8_t *fraction_part);
static void SpiCan_PollToQueue(const Peripheral_Rx_Frame_t *recv_data, FDCAN_Bus_t bus, osMessageQueueId_t queue_handle, uint8_t can_channel);
static bool SpiCana_Poll(const Peripheral_Rx_Frame_t *recv_data, FDCAN_Bus_t bus);
static void SpiCdc_Poll(const Peripheral_Rx_Frame_t *recv_data, FDCAN_Bus_t bus);
static HAL_StatusTypeDef Rs485_StartDma(void);
static void Rs485_DrainDmaBuffer(void);
static void Rs485_ProcessSpan(uint16_t start, uint16_t end);
static void Rs485_DispatchBytes(const uint8_t *data, uint16_t length);
/* USER CODE END FunctionPrototypes */

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
  /* creation of App_Uart_Tx */
  App_Uart_TxHandle = osMessageQueueNew (2, sizeof(App_Uart_Tx_Item_t), &App_Uart_Tx_attributes);

  /* creation of CAN_Msg_Queue */
  CAN_Msg_QueueHandle = osMessageQueueNew (16, sizeof(CAN_Msg_Queue_t), &CAN_Msg_Queue_attributes);

  /* creation of CAN_Msg_Queue_2 */
  CAN_Msg_Queue_2Handle = osMessageQueueNew (16, sizeof(CAN_Msg_Queue_t), &CAN_Msg_Queue_2_attributes);

  /* creation of PublishQueueItem */
  PublishQueueItemHandle = osMessageQueueNew (16, sizeof(PublishQueueItem_t), &PublishQueueItem_attributes);

  /* creation of CAN_Tx_Queue */
  CAN_Tx_QueueHandle = osMessageQueueNew (8, sizeof(CAN_Tx_Queue_t), &CAN_Tx_Queue_attributes);

  /* creation of task01DebugQueue */
  task01DebugQueueHandle = osMessageQueueNew (2, sizeof(App_Uart_Tx_Item_t), &task01DebugQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CanForViehcle */
  CanForViehcleHandle = osThreadNew(CanForViehcleTask, NULL, &CanForViehcle_attributes);

  /* creation of CanForBMS */
  CanForBMSHandle = osThreadNew(CanForBMSTask, NULL, &CanForBMS_attributes);

  /* creation of MLX90640 */
  MLX90640Handle = osThreadNew(MLX90640Task, NULL, &MLX90640_attributes);

  /* creation of TensionSensor */
  TensionSensorHandle = osThreadNew(TensionSensorTask, NULL, &TensionSensor_attributes);

  /* creation of SPICanControl */
  SPICanControlHandle = osThreadNew(SPICanControlTask, NULL, &SPICanControl_attributes);

  /* creation of SPICanCDC */
  SPICanCDCHandle = osThreadNew(SPICanCDCTask, NULL, &SPICanCDC_attributes);

  /* creation of Publish4G */
  Publish4GHandle = osThreadNew(Publish4GTask, NULL, &Publish4G_attributes);

  /* creation of CanBText */
  CanBTextHandle = osThreadNew(CanBTextTask, NULL, &CanBText_attributes);

  /* creation of initTaskBoot */
  initTaskBootHandle = osThreadNew(InitTask_Boot, NULL, &initTaskBoot_attributes);

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
  MLX90640_App_SetLogCallback(App_DebugLogString);

  for(;;)
  {
    Usart3_ParseInput();
    Usart3_SendDebug();
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_CanForViehcleTask */
/**
* @brief Function implementing the CanForViehcle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanForViehcleTask */
void CanForViehcleTask(void *argument)
{
  /* USER CODE BEGIN CanForViehcleTask */
  CAN_Msg_Queue_t recv_data;

  for(;;)
  {
    if(osMessageQueueGet(CAN_Msg_QueueHandle, &recv_data, NULL, osWaitForever) == osOK)
    {
      CanVehicle_Dispatch(&recv_data);
    }
  }
  /* USER CODE END CanForViehcleTask */
}

/* USER CODE BEGIN Header_CanForBMSTask */
/**
* @brief Function implementing the CanForBMS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanForBMSTask */
void CanForBMSTask(void *argument)
{
  /* USER CODE BEGIN CanForBMSTask */
  CAN_Msg_Queue_t recv_data;

  for(;;)
  {
    if(osMessageQueueGet(CAN_Msg_Queue_2Handle, &recv_data, NULL, osWaitForever) == osOK)
    {
      CanBattery_Dispatch(&recv_data);
    }
  }
  /* USER CODE END CanForBMSTask */
}

/* USER CODE BEGIN Header_MLX90640Task */
/**
* @brief Function implementing the MLX90640 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MLX90640Task */
void MLX90640Task(void *argument)
{
  /* USER CODE BEGIN MLX90640Task */
  uint32_t health_tick = HAL_GetTick();

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
        Mlx_SendSummary(sensor_id);
        Mlx_SendCanSummary(sensor_id);
        Mlx_SendDebugMatrix(sensor_id, MLX90640_App_GetTempMap());
        thermal_summary_updated = 1U;
      }
      else
      {
        (void)snprintf(g_mlxLineBuffer, sizeof(g_mlxLineBuffer),
            "MLX90640 capture error, sensor=%u, status=%d\r\n",
            (unsigned int)sensor_id, status);
        App_DebugLogString(g_mlxLineBuffer);
      }
      osDelay(50);
    }

    if ((HAL_GetTick() - health_tick) >= 2000U)
    {
      (void)MLX90640_App_ServiceHealth();
      health_tick = HAL_GetTick();
    }

    if (thermal_summary_updated != 0U)
    {
      (void)Publish_QueueTopic(PUBLISH_TOPIC_THERMAL_SUMMARY);
    }

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    osDelay(125);
  }
  /* USER CODE END MLX90640Task */
}

/* USER CODE BEGIN Header_TensionSensorTask */
/**
* @brief Function implementing the TensionSensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TensionSensorTask */
void TensionSensorTask(void *argument)
{
  /* USER CODE BEGIN TensionSensorTask */
  uint32_t flags = 0U;
  uint8_t dma_running = 0U;

  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
  g_rs485DmaLastEventPos = 0U;
  g_rs485DmaPendingBytes = 0U;
  g_rs485DmaOverflow = 0U;
  g_rs485DmaReadPos = 0U;

  for(;;)
  {
    if (dma_running == 0U)
    {
      if (Rs485_StartDma() != HAL_OK)
      {
        App_DebugLogString("USART2 RX DMA start failed\r\n");
        osDelay(100);
        continue;
      }
      dma_running = 1U;
    }

    flags = osThreadFlagsWait(RS485_FLAG_RX_READY | RS485_FLAG_RX_RESTART,
                              osFlagsWaitAny, osWaitForever);

    if ((flags & RS485_FLAG_RX_READY) != 0U)
    {
      Rs485_DrainDmaBuffer();
    }
    if ((flags & RS485_FLAG_RX_RESTART) != 0U)
    {
      dma_running = 0U;
      App_DebugLogString("USART2 RX DMA restart\r\n");
    }
  }
  /* USER CODE END TensionSensorTask */
}

/* USER CODE BEGIN Header_SPICanControlTask */
/**
* @brief Function implementing the SPICanControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SPICanControlTask */
void SPICanControlTask(void *argument)
{
  /* USER CODE BEGIN SPICanControlTask */
  Peripheral_Rx_Frame_t recv_data = {0};

  for(;;)
  {
    if (FDCAN_IsReady(FDCAN_BUS_A))
    {
      SpiCana_Dispatch(&recv_data);
    }
    osDelay(1);
  }
  /* USER CODE END SPICanControlTask */
}

/* USER CODE BEGIN Header_SPICanCDCTask */
/**
* @brief Function implementing the SPICanCDC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SPICanCDCTask */
void SPICanCDCTask(void *argument)
{
  /* USER CODE BEGIN SPICanCDCTask */
  Peripheral_Rx_Frame_t recv_data = {0};

  for(;;)
  {
    if (FDCAN_IsReady(FDCAN_BUS_B))
    {
      SpiCdc_Dispatch(&recv_data);
    }
    osDelay(1);
  }
  /* USER CODE END SPICanCDCTask */
}

/* USER CODE BEGIN Header_Publish4GTask */
/**
* @brief Function implementing the Publish4G thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Publish4GTask */
void Publish4GTask(void *argument)
{
  /* USER CODE BEGIN Publish4GTask */
  PublishQueueItem_t item;
  uint16_t frame_length = 0U;
  HAL_StatusTypeDef tx_status = HAL_OK;

  for(;;)
  {
    if (osMessageQueueGet(PublishQueueItemHandle, &item, NULL, osWaitForever) == osOK)
    {
      Publish_OnTopicDequeued(item.topic);

      if (Publish_BuildFrame(item.topic, g_publishFrameBuffer,
                             (uint16_t)sizeof(g_publishFrameBuffer), &frame_length))
      {
        tx_status = App_UART_TransmitDmaBlocking(&huart1, g_publishFrameBuffer, frame_length);
        if (tx_status == HAL_OK)
        {
          osDelay(PUBLISH_COOLDOWN_MS);
        }
        else
        {
        (void)Publish_QueueTopic(item.topic);
    }
  }
  /* USER CODE END Publish4GTask */
}
      else
      {
        (void)Publish_QueueTopic(item.topic);
    }
  }
  /* USER CODE END Publish4GTask */
}

/* USER CODE BEGIN Header_CanBTextTask */
/**
* @brief Function implementing the CanBText thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanBTextTask */
void CanBTextTask(void *argument)
{
  /* USER CODE BEGIN CanBTextTask */
  CAN_Tx_Queue_t tx_data;

  for(;;)
  {
    if (osMessageQueueGet(CAN_Tx_QueueHandle, &tx_data, NULL, osWaitForever) == osOK)
    {
      Can1_Send(&tx_data);
    }
  }
  /* USER CODE END CanBTextTask */
}

/* USER CODE BEGIN Header_InitTask_Boot */
/**
* @brief Function implementing the initTaskBoot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InitTask_Boot */
void InitTask_Boot(void *argument)
{
  /* USER CODE BEGIN InitTask_Boot */
  int status;
  int retry;

  App_DebugLogString("\r\n=== Boot Init Start ===\r\n");

  App_DebugLogString("[BOOT] MLX90640 init...\r\n");
  status = MLX90640_App_Init();
  if (status == MLX90640_NO_ERROR)
  {
    App_DebugLogString("[BOOT] MLX90640 init ok\r\n");
  }
  else
  {
    App_DebugLogString("[BOOT] MLX90640 init pending, will retry via health check\r\n");
  }

  App_DebugLogString("[BOOT] MCP2518FD-A init...\r\n");
  for (retry = 0; retry < 3; retry++)
  {
    if (FDCAN_Init(FDCAN_BUS_A) == HAL_OK)
    {
      App_DebugLogString("[BOOT] MCP2518FD-A init ok\r\n");
      break;
    }
    App_DebugLogString("[BOOT] MCP2518FD-A retry...\r\n");
    osDelay(500U);
  }
  if (retry >= 3)
  {
    App_DebugLogString("[BOOT] MCP2518FD-A init FAILED after 3 retries\r\n");
  }

  App_DebugLogString("[BOOT] MCP2518FD-B init...\r\n");
  for (retry = 0; retry < 3; retry++)
  {
    if (FDCAN_Init(FDCAN_BUS_B) == HAL_OK)
    {
      App_DebugLogString("[BOOT] MCP2518FD-B init ok\r\n");
      break;
    }
    App_DebugLogString("[BOOT] MCP2518FD-B retry...\r\n");
    osDelay(500U);
  }
  if (retry >= 3)
  {
    App_DebugLogString("[BOOT] MCP2518FD-B init FAILED after 3 retries\r\n");
  }

  App_DebugLogString("[BOOT] 4G bootstrap...\r\n");
  Y100M_BootstrapOnce();

  App_DebugLogString("=== Boot Init Complete ===\r\n");
  osThreadExit();
  /* USER CODE END InitTask_Boot */
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

static HAL_StatusTypeDef Rs485_StartDma(void)
{
  /* Restart ReceiveToIdle DMA and reset all ring-buffer tracking state. */
  (void)HAL_UART_DMAStop(&huart2);
  __HAL_UART_CLEAR_OREFLAG(&huart2);

  g_rs485DmaLastEventPos = 0U;
  g_rs485DmaPendingBytes = 0U;
  g_rs485DmaOverflow = 0U;
  g_rs485DmaReadPos = 0U;

  return HAL_UARTEx_ReceiveToIdle_DMA(&huart2,
                                      g_rs485DmaBuffer,
                                      (uint16_t)sizeof(g_rs485DmaBuffer));
}

static void Rs485_DrainDmaBuffer(void)
{
  /* Drain the circular buffer in order and hand byte spans to the parser. */
  for (;;)
  {
    uint16_t pending_bytes = 0U;
    uint16_t contiguous_bytes = 0U;

    taskENTER_CRITICAL();
    pending_bytes = g_rs485DmaPendingBytes;
    taskEXIT_CRITICAL();

    if (pending_bytes == 0U)
    {
      break;
    }

    contiguous_bytes = pending_bytes;
    if ((uint16_t)(sizeof(g_rs485DmaBuffer) - g_rs485DmaReadPos) < contiguous_bytes)
    {
      contiguous_bytes = (uint16_t)(sizeof(g_rs485DmaBuffer) - g_rs485DmaReadPos);
    }

    Rs485_ProcessSpan(g_rs485DmaReadPos,
                                (uint16_t)(g_rs485DmaReadPos + contiguous_bytes));

    taskENTER_CRITICAL();
    if (g_rs485DmaPendingBytes >= contiguous_bytes)
    {
      g_rs485DmaPendingBytes = (uint16_t)(g_rs485DmaPendingBytes - contiguous_bytes);
    }
    else
    {
      g_rs485DmaPendingBytes = 0U;
    }
    g_rs485DmaReadPos = (uint16_t)((g_rs485DmaReadPos + contiguous_bytes) % sizeof(g_rs485DmaBuffer));
    taskEXIT_CRITICAL();
  }

  if (g_rs485DmaOverflow != 0U)
  {
    g_rs485DmaOverflow = 0U;
    App_DebugLogString("USART2 RX DMA overflow\r\n");
  }
}

static void Rs485_ProcessSpan(uint16_t start, uint16_t end)
{
  /* Split wrapped spans so the parser always sees a linear byte slice. */
  if (start == end)
  {
    return;
  }

  if (end > start)
  {
    Rs485_DispatchBytes(&g_rs485DmaBuffer[start], (uint16_t)(end - start));
    return;
  }

  Rs485_DispatchBytes(&g_rs485DmaBuffer[start],
                                (uint16_t)(sizeof(g_rs485DmaBuffer) - start));
  if (end > 0U)
  {
    Rs485_DispatchBytes(&g_rs485DmaBuffer[0], end);
  }
}

static void Rs485_DispatchBytes(const uint8_t *data, uint16_t length)
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
    Rs485_Dispatch(&recv_data);

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

    if (current_pos > (uint16_t)sizeof(g_rs485DmaBuffer))
    {
      current_pos = (uint16_t)sizeof(g_rs485DmaBuffer);
    }

    if (event_type == HAL_UART_RXEVENT_TC)
    {
      delta = (uint16_t)(sizeof(g_rs485DmaBuffer) - g_rs485DmaLastEventPos);
      current_pos = 0U;
    }
    else if (current_pos >= g_rs485DmaLastEventPos)
    {
      delta = (uint16_t)(current_pos - g_rs485DmaLastEventPos);
    }
    else
    {
      delta = (uint16_t)((sizeof(g_rs485DmaBuffer) - g_rs485DmaLastEventPos) + current_pos);
    }

    g_rs485DmaLastEventPos = current_pos;
    pending = (uint16_t)(g_rs485DmaPendingBytes + delta);
    if (pending > (uint16_t)sizeof(g_rs485DmaBuffer))
    {
      g_rs485DmaPendingBytes = (uint16_t)sizeof(g_rs485DmaBuffer);
      g_rs485DmaOverflow = 1U;
    }
    else
    {
      g_rs485DmaPendingBytes = pending;
    }

    if (TensionSensorHandle != NULL)
    {
      (void)osThreadFlagsSet(TensionSensorHandle, RS485_FLAG_RX_READY);
    }
  }

  if ((huart != NULL) && (huart->Instance == USART1))
  {
    Y100M_OnUsart1RxIdle(Size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    if (TensionSensorHandle != NULL)
    {
      (void)osThreadFlagsSet(TensionSensorHandle, RS485_FLAG_RX_RESTART);
    }
  }
}

static void Usart3_SendDebug(void)
{
  if (osMessageQueueGet(task01DebugQueueHandle, &g_debugTxItem, NULL, 0U) == osOK)
  {
    (void)App_UART_TransmitDmaBlocking(&huart3, g_debugTxItem.data, g_debugTxItem.length);
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

static void Mlx_SendDebugMatrix(uint8_t sensor_id, const float *temp_map)
{
  char *line = g_mlxLineBuffer;

  if (temp_map == NULL)
  {
    return;
  }

  (void)snprintf(line, MLX_LINE_BUFFER_SIZE, "----- Sensor %u Temperature Matrix -----\r\n", (unsigned int)sensor_id);
  App_DebugLogString(line);

  for (uint16_t row = 0U; row < MLX90640_LINE_NUM; row++)
  {
    uint16_t lineOffset = 0U;

    for (uint16_t col = 0U; col < MLX90640_COLUMN_NUM; col++)
    {
      uint16_t pixelIndex = (uint16_t)(row * MLX90640_COLUMN_NUM + col);
      lineOffset = App_AppendTemperature(line, lineOffset, MLX_LINE_BUFFER_SIZE, temp_map[pixelIndex]);
    }

    if (lineOffset < (MLX_LINE_BUFFER_SIZE - 2U))
    {
      line[lineOffset++] = '\r';
      line[lineOffset++] = '\n';
    }
    line[lineOffset] = '\0';
    App_DebugLogString(line);
  }

  (void)snprintf(line, MLX_LINE_BUFFER_SIZE, "----- Sensor %u Temperature Matrix End -----\r\n", (unsigned int)sensor_id);
  App_DebugLogString(line);
}

static void Mlx_SendSummary(uint8_t sensor_id)
{
  char *line = g_mlxLineBuffer;

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

static void Mlx_SendCanSummary(uint8_t sensor_id)
{
  CAN_Tx_Queue_t tx_data = {0};

  tx_data.std_id = Mlx_GetCanSummaryId(sensor_id);
  if (tx_data.std_id == 0U)
  {
    return;
  }

  for (uint8_t region_index = 0U; region_index < MLX90640_REGION_COUNT; region_index++)
  {
    Mlx_EncodeCanSummaryPair(
        g_MLX90640_Frame.RegionTemp[sensor_id][region_index],
        &tx_data.data[region_index * 2U],
        &tx_data.data[(region_index * 2U) + 1U]);
  }

  (void)freertos_can1_tx_queue_put(&tx_data, 0U);
}

static uint32_t Mlx_GetCanSummaryId(uint8_t sensor_id)
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

static void Mlx_EncodeCanSummaryPair(int32_t centi_c, uint8_t *integer_part, uint8_t *fraction_part)
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
    queue_handle = CAN_Msg_QueueHandle;
  }
  else if(pData->can_channel == CAN_CHANNEL_BATTERY_BOX)
  {
    queue_handle = CAN_Msg_Queue_2Handle;
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
  if(CAN_Tx_QueueHandle == NULL || pData == NULL)
  {
    return osErrorParameter;
  }

  return osMessageQueuePut(CAN_Tx_QueueHandle, pData, 0U, timeout_ms);
}

static void SpiCan_PollToQueue(const Peripheral_Rx_Frame_t *recv_data, FDCAN_Bus_t bus, osMessageQueueId_t queue_handle, uint8_t can_channel)
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

static bool SpiCana_Poll(const Peripheral_Rx_Frame_t *recv_data, FDCAN_Bus_t bus)
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

static void SpiCdc_Poll(const Peripheral_Rx_Frame_t *recv_data, FDCAN_Bus_t bus)
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

static void Usart3_ParseInput(void)
{
  /* USART3 parser entry. */
}

static void CanVehicle_Dispatch(const CAN_Msg_Queue_t *recv_data)
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

static void CanBattery_Dispatch(const CAN_Msg_Queue_t *recv_data)
{
  CAN_DecodeBatteryBoxMessage(recv_data);
  (void)Publish_QueueTopic(PUBLISH_TOPIC_FAST_TELEMETRY);
  (void)Publish_QueueTopic(PUBLISH_TOPIC_BMS_SUMMARY);
  (void)Publish_QueueTopic(PUBLISH_TOPIC_BMS_DETAIL);
}

static void Rs485_Dispatch(const Peripheral_Rx_Frame_t *recv_data)
{
  (void)recv_data;
  /* USART2 + EN# RS485 parser entry. */
}

static void SpiCana_Dispatch(const Peripheral_Rx_Frame_t *recv_data)
{
  if (SpiCana_Poll(recv_data, FDCAN_BUS_A))
  {
    (void)Publish_QueueTopic(PUBLISH_TOPIC_VEHICLE_STATE);
    (void)Publish_QueueTopic(PUBLISH_TOPIC_BMS_SUMMARY);
  }
}

static void SpiCdc_Dispatch(const Peripheral_Rx_Frame_t *recv_data)
{
  SpiCdc_Poll(recv_data, FDCAN_BUS_B);
}

static void Can1_Send(const CAN_Tx_Queue_t *tx_data)
{
  if (tx_data == NULL)
  {
    return;
  }

  CAN_Send_Msg(&hcan1, tx_data->std_id, (uint8_t *)tx_data->data);
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
}
/* USER CODE END Application */

