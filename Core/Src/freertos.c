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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
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
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

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
  myQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue01_attributes);

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
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13, GPIO_PIN_SET);
    //LED Lighting test
	  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
	  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
	  osDelay(100);
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
	CAN_Msg_Queue_t recv_data; // 接收队列数据的临时变量
	osStatus_t recv_status;
  /* Infinite loop */
  for(;;)
  {
    //CAN逻辑任务处理
    // 阻塞等待队列数据（永久等待，portMAX_DELAY对应osWaitForever）
	recv_status = osMessageQueueGet(myQueue01Handle, &recv_data, NULL, osWaitForever);
	if(recv_status == osOK)
	        {
	            // 原解析逻辑移到这
	            // 示例：处理CAN2电池报文
	            if(recv_data.can_channel == 2 && recv_data.is_ext_id == 1)
	            {
	            	 // 处理电池模组电压/温度扩展帧：0x180050F3 ~ 0x184550F3
	            	if(recv_data.msg_id >= 0x180050F3 && recv_data.msg_id <= 0x184550F3)
	               {
	            	   int module_id = 0;
	            	   if(recv_data.msg_id >= 0x184050F3) // 温度帧：解析后直接更新校准后温度
	            	      {
	            	        module_id = (int)((recv_data.msg_id >> 16) & 0x000F);
	            	        for(int i=0; i<BAT_TEMP_POINT_PER_MOD; i++)
	            	           {
	            	             // 校准：原始值 - BAT_TEMP_OFFSET = 实际温度（℃），直接存入结构体
	            	             g_BatteryInfo.ModTemp[module_id][i] = (int8_t)(recv_data.msg_data[i] - BAT_TEMP_OFFSET);
	            	              }
	            	          }
	            	 else // 电压帧：解析单体电压，直接存入结构体（原始值，mV）
	            	    {
	            	      int middle = (int)((recv_data.msg_id - 0x180050F3) >> 16);
	            	       module_id = middle / 6;
	            	       switch(middle % 6)
	            	         {
	            	            case 0: // 第1-4个单体（原逻辑保留，数组维度由宏控制）
	            	              for(int i=0;i<=3;i++)
	            	            	  g_BatteryInfo.CellVolt[module_id][i] = (uint16_t)(recv_data.msg_data[2*i] + recv_data.msg_data[2*i+1]*256);
	            	              break;
	            	            case 1: // 第5-8个单体
	            	              for(int i=0;i<=3;i++)
	            	          	  	  g_BatteryInfo.CellVolt[module_id][i+4] = (uint16_t)(recv_data.msg_data[2*i] + recv_data.msg_data[2*i+1]*256);
	            	              break;
	            	            case 2: // 第9-12个单体
	            	              for(int i=0;i<=3;i++)
	            	            	  g_BatteryInfo.CellVolt[module_id][i+8] = (uint16_t)(recv_data.msg_data[2*i] + recv_data.msg_data[2*i+1]*256);
	            	              break;
	            	            case 3: // 第13-16个单体
	            	              for(int i=0;i<=3;i++)
	            	            	   g_BatteryInfo.CellVolt[module_id][i+12] = (uint16_t)(recv_data.msg_data[2*i] + recv_data.msg_data[2*i+1]*256);
	            	              break;
	            	            case 4: // 第17-20个单体
	            	              for(int i=0;i<=3;i++)
	            	            	  g_BatteryInfo.CellVolt[module_id][i+16] = (uint16_t)(recv_data.msg_data[2*i] + recv_data.msg_data[2*i+1]*256);
	            	              break;
	            	            case 5: // 第21-24个单体
	            	              for(int i=0;i<=3;i++)
	            	            	  g_BatteryInfo.CellVolt[module_id][i+20] = (uint16_t)(recv_data.msg_data[2*i] + recv_data.msg_data[2*i+1]*256);
	            	              break;
	            	            default:
	            	               break;
	            	                    }
	            	                  }
	            	                }

	            	       // 处理电池整体参数扩展帧：0x186050F4 ~ 0x186350F4，直接更新结构体成员
	            	       if(recv_data.msg_id >= 0x186050F4 && recv_data.msg_id <= 0x186350F4)
	            	          {
	            	           switch(recv_data.msg_id)
	            	              {
	            	               case 0x186050F4: // 总电压/总电流/SOC/绝缘状态/工作状态
	            	                 g_BatteryInfo.TotalVolt = (uint16_t)(recv_data.msg_data[0]*256 + recv_data.msg_data[1]);
	            	                 g_BatteryInfo.TotalCurrent = (int16_t)(recv_data.msg_data[2]*256 + recv_data.msg_data[3]);
	            	                 g_BatteryInfo.SOC = recv_data.msg_data[4];
	            	                 g_BatteryInfo.IMD_State = recv_data.msg_data[5];
	            	                 g_BatteryInfo.Work_State = (uint8_t)(((recv_data.msg_data[6] & 0xF0) >> 4) - 2);
	            	                      // 充电状态修正
	            	                 if(g_BatteryInfo.Work_State == 3)
	            	                     {
	            	                       g_BatteryInfo.Work_State = g_BatteryInfo.Charge_Enable ? 4 : 3;
	            	                     }
	            	                   break;

	            	                case 0x186150F4: // 单体最高/最低电压
	            	                      g_BatteryInfo.MaxCellVolt = (uint16_t)(256*recv_data.msg_data[0] + recv_data.msg_data[1]);
	            	                      g_BatteryInfo.MinCellVolt = (uint16_t)(256*recv_data.msg_data[2] + recv_data.msg_data[3]);
	            	                      break;

	            	                case 0x186250F4: // 最高/最低温度（已校准）
	            	                      g_BatteryInfo.MaxTemp = (int8_t)(recv_data.msg_data[0] - BAT_TEMP_OFFSET);
	            	                      g_BatteryInfo.MinTemp = (int8_t)(recv_data.msg_data[1] - BAT_TEMP_OFFSET);
	            	                      break;

	            	                case 0x186350F4: // 继电器/充电信号/充电参数（电流已校准）
	            	                      g_BatteryInfo.Air1_State = (uint8_t)((recv_data.msg_data[0] & 0x0C) >> 2);
	            	                      g_BatteryInfo.Air2_State = (uint8_t)((recv_data.msg_data[0] & 0x30) >> 4);
	            	                      g_BatteryInfo.Air3_State = (uint8_t)((recv_data.msg_data[0] & 0xC0) >> 6);
	            	                      g_BatteryInfo.Charge_Enable = (uint8_t)((recv_data.msg_data[1] & 0x3F) >> 5);
	            	                      g_BatteryInfo.Charge_ReqVolt = (uint16_t)(256*recv_data.msg_data[2] + recv_data.msg_data[3]);
	            	                      // 校准：原始值 / BAT_CHARGE_I_DIV = 实际充电电流（mA）
	            	                      g_BatteryInfo.Charge_ReqCurr = (int16_t)((256*recv_data.msg_data[4] + recv_data.msg_data[5]) * CHARGER_CURR_SCALE);
	            	                      g_BatteryInfo.Charger_OutVolt = (uint16_t)(256*recv_data.msg_data[6] + recv_data.msg_data[7]);
	            	                      break;

	            	                 default:
	            	                      break;
	            	                  }
	            	       }
	            }
	            // CAN1报文解析同理
	            else if(recv_data.can_channel == 1 && recv_data.is_ext_id == 0)
	            {
	            	  switch(recv_data.msg_id)
	            	  {
	            	    case 0x401:
	            	      // 预留：0x401 ID报文处理逻辑
	            	    {

	            	    }
	            	      break;
	            	    case 0x501:
	            	      // 预留：电机扭矩与常态化驾驶模式处理逻辑
	            	    {
	            	    	g_CANB_LoopData.ECU.Vehicle_Speed=(int)recv_data.msg_data[0];
	            	    	g_CANB_LoopData.IMU.Long_Accel=(int)recv_data.msg_data[1];
	            	    	g_CANB_LoopData.IMU.Lat_Accel=(int)recv_data.msg_data[2];
	            	    	for(int i=0;i<4;i++)
	            	    	   {g_CANB_LoopData.ECU.Motor_Torque[i]=(int)recv_data.msg_data[i+3];}
	            	    	g_CANB_LoopData.ECU.driving_mode=(int)recv_data.msg_data[7];
	            	    }
	            	      break;
	            	    case 0x502:
	            	      // 预留：0x502 ID报文处理逻辑
	            	    {
	            	      /*四轮扭矩，RL,FL,RR,FR */
	            	    	for(int i=0;i<4;i++)
	            	    	  {
	            	    		g_CANB_LoopData.ECU.Wheel_RPM[i] = (int)recv_data.msg_data[i];
	            	    		g_CANB_LoopData.ECU.Motor_Power[i] =
	            	    		(256*(int)recv_data.msg_data[2*i+1]+(int)recv_data.msg_data[2*i])/9549;
	            	    	  }
	     	            	}
	            	      break;
	            	    case 0x50:
	            	      // 预留：IMU数据回发与处理逻辑
	            	    	//	{
	            	    	////		User_CAN_Send_sq(0x03,recv_data.msg_data);
	            	    	//		if(recv_data.msg_data[1]==0x50)
	            	    	//		{
	            	    	//			User_CAN_Send_sq(0x60,recv_data.msg_data);
	            	    	//		}else if(recv_data.msg_data[1]==0x51)
	            	    	//		{
	            	    	//			User_CAN_Send_sq(0x61,recv_data.msg_data);
	            	    	//
	            	    	//		}else if(recv_data.msg_data[1]==0x52){
	            	    	//			User_CAN_Send_sq(0x62,recv_data.msg_data);
	            	    	//
	            	    	//		}else if(recv_data.msg_data[1]==0x53){
	            	    	//			if(recv_data.msg_data[2]==0x01)
	            	    	//			{
	            	    	//				User_CAN_Send_sq(0x63,recv_data.msg_data);
	            	    	//			}else if(recv_data.msg_data[2]==0x02)
	            	    	//			{
	            	    	//				User_CAN_Send_sq(0x64,recv_data.msg_data);
	            	    	//			}else if(recv_data.msg_data[2]==0x03)
	            	    	//			{
	            	    	//				User_CAN_Send_sq(0x65,recv_data.msg_data);
	            	    	//			}
	            	    	//
	            	    	//		}else if(recv_data.msg_data[1]==0x54){
	            	    	//			User_CAN_Send_sq(0x66,recv_data.msg_data);
	            	    	//		}
	            	    	//	}
	            	      break;
	            	    default:
	            	      break;
	            	  }
	            }
	        }
    osDelay(10);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// 引入FreeRTOS原生API头文件（必须，否则找不到xQueueSendFromISR）
#include "queue.h"

// 修正后的CAN队列发送接口（中断安全）
osStatus_t freertos_can_queue_send_from_isr(const CAN_Msg_Queue_t *pData)
{
  // 1. 参数校验
  if(myQueue01Handle == NULL || pData == NULL)
  {
    return osErrorParameter;
  }

  // 2. CMSIS-RTOS V2句柄 → 原生FreeRTOS句柄（强转，完全兼容）
  QueueHandle_t xQueue = (QueueHandle_t)myQueue01Handle;

  // 3. 调用原生FreeRTOS中断安全API（关键：替代不存在的osMessageQueuePutFromISR）
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  BaseType_t xResult = xQueueSendFromISR(
    xQueue,          // 原生队列句柄
    pData,           // 要发送的数据
    &xHigherPriorityTaskWoken // 唤醒标记（FreeRTOS要求）
  );

  // 4. 触发任务调度（中断中必须做）
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

  // 5. 转换返回值（原生FreeRTOS → CMSIS-RTOS V2）
  return (xResult == pdPASS) ? osOK : osError;
}
/* USER CODE END Application */

