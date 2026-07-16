/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   STM32F407 CAN1/CAN2控制器驱动：底层配置、滤波初始化、
  *          报文收发/中断处理，支持标准帧/扩展帧通信，适配电池箱、
  *          电机、IMU等外设；遥测数据已抽离至telemetry_data.c/h，解耦设计
  *          CAN1: vehicle CANB bus.
  *          CAN2: battery-box bus, mapped from reference project CAN1.
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics. All rights reserved.
  * This software is licensed under the LICENSE file
  * in the root directory of this software component. If no LICENSE file, AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

// 定义结构体实例并初始化（保留原变量初始化值）
/* Shared CAN message scratchpad used by both transmit and receive paths. */
can_msg g_can_msg = {
  .g_CANTxData = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77},
  .g_CANRxData = {0},
  .g_CAN1DefSendID = CAN_ID_CANB_BATTERY_POWER_STATUS
};

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */
  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_SCE_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief CAN通用滤波配置函数（整合CAN1/CAN2，统一入口，无电池相关逻辑）
  * @param canHandle CAN句柄指针（&hcan1/&hcan2）
  * @note CAN1 uses filter bank 0 and accepts the vehicle CANB bus into FIFO0.
  *       CAN2 uses filter bank 14 and accepts the battery-box bus into FIFO0.
  */
void CAN_Filter_Config(CAN_HandleTypeDef *canHandle)
{
  /* Configure the shared filter banks before either controller starts. */
  CAN_FilterTypeDef CAN_FilterInitStructure = {0};

  if(canHandle->Instance == CAN1)  // CAN1滤波配置：16位ID列表模式
  {
    CAN_FilterInitStructure.FilterActivation = ENABLE;
    CAN_FilterInitStructure.FilterBank = 0x00;
    CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_FilterInitStructure.FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.FilterIdLow = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterInitStructure.SlaveStartFilterBank = 14;
  }
  else if(canHandle->Instance == CAN2) // CAN2滤波配置：32位掩码全接收模式
  {
    CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterInitStructure.FilterActivation = ENABLE;
    CAN_FilterInitStructure.FilterBank = 14;
    CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_FilterInitStructure.FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.FilterIdLow = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterInitStructure.SlaveStartFilterBank = 14;
  }

  if(HAL_CAN_ConfigFilter(canHandle, &CAN_FilterInitStructure) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN通用发送函数（CAN1/CAN2共用，标准帧8字节数据帧，无电池相关逻辑）
  * @param canHandle CAN句柄指针（&hcan1/&hcan2）
  * @param SendStdId 要发送的11位标准ID（0~0x7FF）
  * @param pTxData 发送数据缓冲区指针（必须8字节）
  * @note 固定配置：数据帧、标准ID、8字节长度、禁用全局时间戳，自动选择发送邮箱
  */
void CAN_Send_Msg(CAN_HandleTypeDef *canHandle, uint32_t SendStdId, uint8_t *pTxData)
{
  /* Send one standard 8-byte CAN frame through the selected controller. */
  // 配置CAN发送帧头（固定参数，无需修改）
  g_can_msg.g_CANTxHeader.RTR = CAN_RTR_DATA;
  g_can_msg.g_CANTxHeader.IDE = CAN_ID_STD;
  g_can_msg.g_CANTxHeader.StdId = SendStdId;
  g_can_msg.g_CANTxHeader.TransmitGlobalTime = DISABLE;
  g_can_msg.g_CANTxHeader.DLC = 8;

  // 添加报文到发送邮箱；发送邮箱满或总线暂不可用时丢弃本帧，避免进入 Error_Handler 卡死调度。
  if(HAL_CAN_AddTxMessage(canHandle, &g_can_msg.g_CANTxHeader, pTxData, &g_can_msg.g_CANTxMailBox) != HAL_OK)
  {
    return;
  }
}

/**
  * @brief CAN控制器统一启动函数（上层调用入口，一键完成初始化）
  * @note 按CAN1→CAN2顺序启动，保证CAN2依赖的CAN1时钟正常，启动后使能FIFO0中断
  * @usage 主函数中调用：CAN_Start(); 即可完成所有CAN配置
  */
void CAN_Start(void)
{
//  MX_CAN1_Init();
//  MX_CAN2_Init();
  /* Start both controllers only after filters have been installed. */
  CAN_Filter_Config(&hcan1);
  CAN_Filter_Config(&hcan2);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  // 使能CAN1/CAN2 FIFO0报文挂起中断（核心接收中断）
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief CAN1 FIFO0报文挂起中断回调函数（处理电机/IMU等标准帧，无电池逻辑）
  * @param hcan CAN句柄指针（仅处理hcan1实例）
  * @note 保留原有业务逻辑入口，接收后可根据ID做对应处理，不影响电池数据
  */
/* CAN1 FIFO0 callback: decode the received frame in the RTOS-friendly path. */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance != CAN1) return; // 仅处理CAN1中断，防止跨实例误触发

  // 读取CAN1 FIFO0接收报文，失败则错误处理
  if(HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &g_can_msg.g_CANRxHeader, g_can_msg.g_CANRxData) != HAL_OK)
  {
    Error_Handler();
    return;
  }

  // 2. 封装CAN报文到队列结构体，在freertos中执行解析部分，节省中断时间，避免卡顿
      CAN_Msg_Queue_t can_queue_data = {0};
      can_queue_data.can_channel = CAN_CHANNEL_VEHICLE_CANB;
      can_queue_data.dlc = (uint8_t)g_can_msg.g_CANRxHeader.DLC;
      if (g_can_msg.g_CANRxHeader.IDE == CAN_ID_EXT)
      {
        can_queue_data.msg_id = g_can_msg.g_CANRxHeader.ExtId;
        can_queue_data.is_ext_id = 1U;
      }
      else
      {
        can_queue_data.msg_id = g_can_msg.g_CANRxHeader.StdId;
        can_queue_data.is_ext_id = 0U;
      }
      memcpy(can_queue_data.msg_data, g_can_msg.g_CANRxData, 8); // 拷贝8字节数据

      // 3. 中断中发送到FreeRTOS队列（CMSIS-RTOS V2中断安全API）
      // 3. 调用统一队列接口发送数据，不直接耦合具体任务内部实现
        osStatus_t send_status = freertos_can_queue_send_from_isr(&can_queue_data);
        (void)send_status;

}

/**
  * @brief CAN2 FIFO0报文挂起中断回调函数（唯一处理电池箱扩展帧，更新全局电池数据）
  * @param hcan CAN句柄指针（仅处理hcan2实例）
  * @note 1. 所有解析后的数据直接更新至g_BatteryInfo全局结构体
  *       2. 数据已按telemetry_data.h中的宏完成校准，上层可直接使用，无需再处理
  *       3. 保留原有解析逻辑，仅修改变量为结构体成员，无功能变化
  */
/* CAN2 FIFO0 callback: decode the received frame in the RTOS-friendly path. */
void HAL_CAN2_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance != CAN2) return; // 仅处理CAN2中断，防止跨实例误触发

  // 读取CAN2 FIFO0接收报文，失败则错误处理
  if(HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &g_can_msg.g_CANRxHeader, g_can_msg.g_CANRxData) != HAL_OK)
  {
    Error_Handler();
    return;
  }
//CAN2同理即是
  // 2. 封装CAN报文到队列结构体
      CAN_Msg_Queue_t can_queue_data = {0};
      can_queue_data.can_channel = CAN_CHANNEL_BATTERY_BOX;
      can_queue_data.dlc = (uint8_t)g_can_msg.g_CANRxHeader.DLC;
      if (g_can_msg.g_CANRxHeader.IDE == CAN_ID_EXT)
      {
        can_queue_data.msg_id = g_can_msg.g_CANRxHeader.ExtId;
        can_queue_data.is_ext_id = 1U;
      }
      else
      {
        can_queue_data.msg_id = g_can_msg.g_CANRxHeader.StdId;
        can_queue_data.is_ext_id = 0U;
      }
      memcpy(can_queue_data.msg_data, g_can_msg.g_CANRxData, 8); // 拷贝8字节数据

      // 3. 中断中发送到FreeRTOS队列
      // 3. 调用统一队列接口发送数据，不直接耦合具体任务内部实现
        osStatus_t send_status = freertos_can_queue_send_from_isr(&can_queue_data);
        (void)send_status;

}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
