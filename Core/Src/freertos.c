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
#include "modem_transport.h"
#include "app_log.h"
#include "app_config.h"
#include "queue.h"
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
/* Set to 1 to restore the one-shot modem AT/MQTT bootstrap. */
#define APP_4G_BOOTSTRAP_ENABLED         0U
#define APP_4G_PUBLISH_LOG_ENABLED       0U
#define APP_SPICANA_READY_CHECK_ENABLED  1U
#define APP_SPICANA_POLL_ENABLED         1U
#define APP_SPICANA_DECODE_ENABLED       1U
#define APP_SPICANA_PUBLISH_ENABLED      1U
#define APP_MLX90640_PUBLISH_ENABLED     1U
#define APP_SPICDC_TASK_ENABLED          0U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static volatile uint8_t g_bootInitDone = 0U;
static uint8_t g_mlxCurrentSensor = 0U;
static uint8_t g_rs485DmaBuffer[RS485_DMA_BUFFER_SIZE];
static volatile uint16_t g_rs485DmaLastEventPos = 0U;
static volatile uint16_t g_rs485DmaPendingBytes = 0U;
static volatile uint8_t g_rs485DmaOverflow = 0U;
static uint16_t g_rs485DmaReadPos = 0U;
#if APP_UPLINK_ENABLED
static uint8_t g_publishFrameBuffer[PUBLISH_MAX_PAYLOAD_SIZE];
static volatile uint16_t g_diagPublishPayloadLen = 0U;
static volatile int g_diagPublishStatus = 0;
static volatile uint32_t g_diagPublishOkCount = 0U;
static volatile uint32_t g_diagPublishFailCount = 0U;
#endif
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
  .priority = (osPriority_t) osPriorityRealtime,
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
uint32_t CanBTextBuffer[ 512 ];
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
  .priority = (osPriority_t) osPriorityRealtime7,
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
/* Definitions for VehicleCanTxQueue */
osMessageQueueId_t VehicleCanTxQueueHandle;
uint8_t VehicleCanTxQueueBuffer[ 8 * sizeof( CAN_Tx_Queue_t ) ];
osStaticMessageQDef_t VehicleCanTxQueueControlBlock;
const osMessageQueueAttr_t VehicleCanTxQueue_attributes = {
  .name = "VehicleCanTxQueue",
  .cb_mem = &VehicleCanTxQueueControlBlock,
  .cb_size = sizeof(VehicleCanTxQueueControlBlock),
  .mq_mem = &VehicleCanTxQueueBuffer,
  .mq_size = sizeof(VehicleCanTxQueueBuffer)
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
static void App_WaitForBootInit(void);
static void CanVehicle_Dispatch(const CAN_Msg_Queue_t *recv_data);
static void CanBattery_Dispatch(const CAN_Msg_Queue_t *recv_data);
static void Rs485_Dispatch(const Peripheral_Rx_Frame_t *recv_data);
static void SpiCana_Dispatch(const Peripheral_Rx_Frame_t *recv_data);
static void SpiCdc_Dispatch(const Peripheral_Rx_Frame_t *recv_data);
static void VehicleCan_Send(const CAN_Tx_Queue_t *tx_data);

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

  /* creation of VehicleCanTxQueue */
  VehicleCanTxQueueHandle = osMessageQueueNew (8, sizeof(CAN_Tx_Queue_t), &VehicleCanTxQueue_attributes);

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

  for(;;)
  {
    static uint32_t led_tick;
    if ((HAL_GetTick() - led_tick) >= 200U)
    {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
      led_tick = HAL_GetTick();
    }
    App_LogProcess();
    osDelay(1);
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

  App_WaitForBootInit();

  for(;;)
  {
    if(osMessageQueueGet(CAN_Msg_QueueHandle, &recv_data, NULL, pdMS_TO_TICKS(20U)) == osOK)
    {
      app_can_debug_log_rx_context("CAN2 RX", &g_can2_ctx);
      CanVehicle_Dispatch(&recv_data);
    }
    CAN_DecodeVehicleService(HAL_GetTick());
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

  App_WaitForBootInit();

  for(;;)
  {
    if(osMessageQueueGet(CAN_Msg_Queue_2Handle, &recv_data, NULL, pdMS_TO_TICKS(20U)) == osOK)
    {
      app_can_debug_log_rx_context("BAT RX",
          recv_data.can_channel == CAN_CHANNEL_BATTERY_BOX ? &g_can1_ctx : &g_can2_ctx);
      CanBattery_Dispatch(&recv_data);
    }
    CAN_DecodeBatteryService(HAL_GetTick());
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
  uint32_t retry_tick[MLX90640_SENSOR_COUNT] = {0};
  uint8_t attempted[MLX90640_SENSOR_COUNT] = {0};

  App_WaitForBootInit();
  MLX90640_App_SetLogCallback(App_DebugLogString);
  MLX90640_App_Prepare();

  for(;;)
  {
    /* One channel at a time: successful channels capture without waiting for
     * the other channels to initialize. The shared I2C bus has one owner. */
    if (!MLX90640_App_IsSensorReady(g_mlxCurrentSensor) &&
        (!attempted[g_mlxCurrentSensor] ||
         (HAL_GetTick() - retry_tick[g_mlxCurrentSensor]) >= 2000U))
    {
      (void)MLX90640_App_InitSensor(g_mlxCurrentSensor);
      attempted[g_mlxCurrentSensor] = 1U;
      retry_tick[g_mlxCurrentSensor] = HAL_GetTick();
    }
    if (MLX90640_App_IsSensorReady(g_mlxCurrentSensor) != 0U)
    {
      int status = MLX90640_App_CaptureOnce(g_mlxCurrentSensor);
      if (status == MLX90640_NO_ERROR)
      {
        Mlx_SendSummary(g_mlxCurrentSensor);
        Mlx_SendCanSummary(g_mlxCurrentSensor);
#if (APP_MLX90640_PUBLISH_ENABLED != 0U)
        (void)Publish_QueueTopic(PUBLISH_TOPIC_THERMAL_SUMMARY);
#endif
      }
      else if ((status == -MLX90640_FRAME_NOT_READY_ERROR) ||
               (status == -MLX90640_FRAME_INCOMPLETE_ERROR))
      {
      }
      else
      {
        App_LogPrintf(
            "MLX90640 capture error, sensor=%u, status=%d\r\n",
            (unsigned int)g_mlxCurrentSensor, status);
      }
    }

    g_mlxCurrentSensor++;
    if (g_mlxCurrentSensor >= MLX90640_SENSOR_COUNT)
    {
      g_mlxCurrentSensor = 0U;

    }

    // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    osDelay(200);
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

  App_WaitForBootInit();

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
    osDelay(200);
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

  App_WaitForBootInit();

  for(;;)
  {
#if (APP_SPICANA_READY_CHECK_ENABLED != 0U)
    if (FDCAN_IsReady(FDCAN_BUS_A))
    {
      SpiCana_Dispatch(&recv_data);
    }
#else
    (void)recv_data;
#endif
    osDelay(200);
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

  App_WaitForBootInit();

  for(;;)
  {
    if (FDCAN_IsReady(FDCAN_BUS_B))
    {
      SpiCdc_Dispatch(&recv_data);
    }
    osDelay(200);
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
#if (APP_UPLINK_ENABLED == 0U)
  /* Keep this guard inside USER CODE so CubeMX regeneration stays safe. */
  (void)argument;
  osThreadExit();
#else
  PublishQueueItem_t item;
  uint16_t frame_length = 0U;
  HAL_StatusTypeDef tx_status = HAL_OK;
  uint32_t wait_log_tick = 0U;

  App_WaitForBootInit();

  /* Only the publishing task owns modem initialization. */
#if (APP_4G_BOOTSTRAP_ENABLED != 0U)
  Modem_Init();
#else
  Modem_AssumeReady();
#endif

  for(;;)
  {
    if (Modem_IsReady() == 0U)
    {
      uint32_t now = HAL_GetTick();
      if ((APP_4G_PUBLISH_LOG_ENABLED != 0U) && ((now - wait_log_tick) >= 1000U))
      {
        App_DebugLogString("[4G-PUB] waiting for MQTT ready\r\n");
        wait_log_tick = now;
      }
      osDelay(100);
      continue;
    }

    if (osMessageQueueGet(PublishQueueItemHandle, &item, NULL, osWaitForever) == osOK)
    {
      Publish_OnTopicDequeued(item.topic);

      if (Publish_BuildFrame(item.topic, g_publishFrameBuffer,
                             (uint16_t)sizeof(g_publishFrameBuffer), &frame_length))
      {
        g_diagPublishPayloadLen = frame_length;
        tx_status = Modem_Send(g_publishFrameBuffer, frame_length);
        g_diagPublishStatus = (int)tx_status;

        if (tx_status == HAL_OK)
        {
          g_diagPublishOkCount++;
          osDelay(PUBLISH_COOLDOWN_MS);
        }
        else
        {
          g_diagPublishFailCount++;
          if (APP_4G_PUBLISH_LOG_ENABLED != 0U)
          {
            App_LogPrintf(
                         "[PUB-DIAG] send fail len=%u status=%d ok=%lu fail=%lu\r\n",
                         (unsigned int)g_diagPublishPayloadLen,
                         g_diagPublishStatus,
                         (unsigned long)g_diagPublishOkCount,
                         (unsigned long)g_diagPublishFailCount);
          }
          osDelay(1000U);
          (void)Publish_QueueTopic(item.topic);
        }
      }
      else
      {
        g_diagPublishFailCount++;
        if (APP_4G_PUBLISH_LOG_ENABLED != 0U)
        {
          App_LogPrintf(
                       "[PUB-DIAG] build frame failed topic=%u fail=%lu\r\n",
                       (unsigned int)item.topic,
                       (unsigned long)g_diagPublishFailCount);
        }
        osDelay(1000U);
        (void)Publish_QueueTopic(item.topic);
      }
      osDelay(200);
    }
  }
#endif
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
  App_WaitForBootInit();
  for(;;)
  {
    if (osMessageQueueGet(VehicleCanTxQueueHandle, &tx_data, NULL, osWaitForever) == osOK)
    {
      VehicleCan_Send(&tx_data);
    }
    osDelay(1);
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
  int retry;

  App_DebugLogString("\r\n=== Boot Init Start ===\r\n");

  App_DebugLogString("[BOOT] CAN start begin\r\n");
  CAN_Start();
  App_DebugLogString("[BOOT] CAN start done\r\n");

  App_DebugLogString("[BOOT] MCP2518FD-A init...\r\n");
 for (retry = 0; retry < 3; retry++)
 {
   if (FDCAN_Init(FDCAN_BUS_A) == HAL_OK)
   {
     App_DebugLogString("[BOOT] MCP2518FD-A init ok\r\n");
     break;
   }
   App_DebugLogString("[BOOT] MCP2518FD-A retry...\r\n");
   HAL_Delay(10);
 }
 if (retry >= 3)
 {
   App_DebugLogString("[BOOT] MCP2518FD-A init FAILED after 3 retries\r\n");
 }
  HAL_Delay(10);//初始化就不要考虑CPU让渡问题了
  App_DebugLogString("[BOOT] MCP2518FD-B init...\r\n");
#if (APP_SPICDC_TASK_ENABLED != 0U)
  for (retry = 0; retry < 3; retry++)
  {
    if (FDCAN_Init(FDCAN_BUS_B) == HAL_OK)
    {
      App_DebugLogString("[BOOT] MCP2518FD-B init ok\r\n");
      break;
    }
    App_DebugLogString("[BOOT] MCP2518FD-B retry...\r\n");
    HAL_Delay(10);
  }
  if (retry >= 3)
  {
    App_DebugLogString("[BOOT] MCP2518FD-B init FAILED after 3 retries\r\n");
  }
#endif

  g_bootInitDone = 1U;
  App_DebugLogString("=== Local Hardware Init Complete ===\r\n");
  osThreadExit();
  /* USER CODE END InitTask_Boot */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void App_WaitForBootInit(void)
{
  while (g_bootInitDone == 0U)
  {
    osDelay(100U);
  }
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
#if APP_UPLINK_ENABLED
    Modem_OnRxEvent(Size);
#endif
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

static void Mlx_SendSummary(uint8_t sensor_id)
{
  App_LogPrintf(
      "MLX90640 capture ok, sensor=%u, frames=%lu\r\n",
      (unsigned int)sensor_id,
      (unsigned long)g_MLX90640_Frame.FrameCounter[sensor_id]);
}

static void Mlx_SendCanSummary(uint8_t sensor_id)
{
  CAN_Tx_Queue_t tx_data = {0};
  osStatus_t queue_status;

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

  queue_status = freertos_vehicle_can_tx_queue_put(&tx_data, 0U);
  if (queue_status != osOK)
  {
    App_LogPrintf(
        "MLX CAN queue drop, sensor=%u, id=0x%03lX, status=%ld\r\n",
        (unsigned int)sensor_id,
        (unsigned long)tx_data.std_id,
        (long)queue_status);

  }
}

static uint32_t Mlx_GetCanSummaryId(uint8_t sensor_id)
{
  switch (sensor_id)
  {
    case 0U:
      return CAN_ID_CANB_THERMAL_SUMMARY_FR;
    case 1U:
      return CAN_ID_CANB_THERMAL_SUMMARY_FL;
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

  if(CAN_IsBatteryCanbMessage(pData))
  {
    queue_handle = CAN_Msg_Queue_2Handle;
  }
  else if(pData->can_channel == CAN_CHANNEL_VEHICLE_CANB)
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

osStatus_t freertos_vehicle_can_tx_queue_put(const CAN_Tx_Queue_t *pData, uint32_t timeout_ms)
{
  if(VehicleCanTxQueueHandle == NULL || pData == NULL)
  {
    return osErrorParameter;
  }

  return osMessageQueuePut(VehicleCanTxQueueHandle, pData, 0U, timeout_ms);
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
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
#if (APP_SPICANA_DECODE_ENABLED != 0U)
    CAN_DecodeControllerCanaMessage(&frame_data);
#endif
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
#if (APP_SPICANA_POLL_ENABLED != 0U)
  if (SpiCana_Poll(recv_data, FDCAN_BUS_A))
  {
#if (APP_SPICANA_PUBLISH_ENABLED != 0U)
    (void)Publish_QueueTopic(PUBLISH_TOPIC_VEHICLE_STATE);
    (void)Publish_QueueTopic(PUBLISH_TOPIC_BMS_SUMMARY);
#endif
  }
#else
  (void)recv_data;
#endif
}

static void SpiCdc_Dispatch(const Peripheral_Rx_Frame_t *recv_data)
{
  SpiCdc_Poll(recv_data, FDCAN_BUS_B);
}

static void VehicleCan_Send(const CAN_Tx_Queue_t *tx_data)
{
  if (tx_data == NULL)
  {
    return;
  }
  CAN_Send_Msg(&hcan2, tx_data->std_id, (uint8_t *)tx_data->data);
  if (g_can2_ctx.last_tx_status != HAL_OK)
  {
    App_LogPrintf(
        "CAN2 tx fail, id=0x%03lX, free=%lu, err=0x%08lX, req=%lu, drop=%lu, abort=%lu, ast=%lu, afree=%lu, aerr=0x%08lX\r\n",
        (unsigned long)tx_data->std_id,
        (unsigned long)g_can2_ctx.last_tx_free_level,
        (unsigned long)g_can2_ctx.last_tx_error,
        (unsigned long)g_can2_ctx.tx_request_count,
        (unsigned long)g_can2_ctx.tx_drop_count,
        (unsigned long)g_can2_ctx.tx_abort_count,
        (unsigned long)g_can2_ctx.last_tx_abort_status,
        (unsigned long)g_can2_ctx.last_tx_free_after_abort,
        (unsigned long)g_can2_ctx.last_tx_error_after_abort);

  }
  // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
}
/* USER CODE END Application */

