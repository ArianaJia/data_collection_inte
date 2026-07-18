/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   STM32F407 CAN1/CAN2控制器驱动：底层配置、滤波初始化、
  *          报文收发/中断处理，支持标准帧/扩展帧通信，适配电池箱、
  *          电机、IMU等外设；遥测数据已抽离至telemetry_data.c/h，解耦设计
 *          CAN1: battery-box bus.
 *          CAN2: vehicle CANB bus.
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

void app_can_debug_log_rx_context(const char *prefix, const CAN_ControllerContext_t *ctx);

/* USER CODE BEGIN 0 */

static const uint16_t g_can2_vehicle_filter_ids[] = {
  CAN_ID_CANB_LEGACY_VEHICLE_STATE,
  CAN_ID_CANB_WHEEL_ACTUAL_TORQUE,
  CAN_ID_CANB_REAR_MOTOR_DIAGNOSTIC_CODE,
  CAN_ID_CANB_FRONT_MOTOR_DIAGNOSTIC_CODE,
  CAN_ID_CANB_WHEEL_ACTUAL_SPEED,
  CAN_ID_CANB_MOTOR_TEMPERATURE,
  CAN_ID_CANB_INVERTER_TEMPERATURE,
  CAN_ID_CANB_IGBT_TEMPERATURE,
  CAN_ID_CANB_AMK_STATUS_MODE_LOGIC,
  CAN_ID_CANB_BATTERY_POWER_STATUS,
  CAN_ID_CANB_BATTERY_FAULT_STATUS,
  CAN_ID_CANB_ENERGY_METER_STATUS,
  CAN_ID_IMU_RAW,
  CAN_ID_IMU_TIME,
  CAN_ID_IMU_ACCEL,
  CAN_ID_IMU_GYRO,
  CAN_ID_IMU_ROLL,
  CAN_ID_IMU_PITCH,
  CAN_ID_IMU_YAW,
  CAN_ID_IMU_MAGNETIC,
  CAN_ID_IVT_CURRENT,
  CAN_ID_IVT_U1,
  CAN_ID_IVT_POWER,
  CAN_ID_IVT_WH,
  CAN_ID_CANB_THERMAL_SUMMARY_FL,
  CAN_ID_CANB_THERMAL_SUMMARY_FR,
  CAN_ID_CANB_THERMAL_SUMMARY_RL,
  CAN_ID_CANB_THERMAL_SUMMARY_RR,
  CAN_ID_CANB_GPS_SPEED,
  CAN_ID_CANB_DRIVER_INPUTS_OIL_PRESSURE,
  CAN_ID_CANB_DRIVING_MODE_CMD
};

CAN_ControllerContext_t g_can1_ctx = {
  .tx_data = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77},
  .rx_data = {0}
};

CAN_ControllerContext_t g_can2_ctx = {
  .tx_data = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77},
  .rx_data = {0}
};

static const uint32_t g_can1_battery_filter_ids[] = {
  CAN_ID_BATTERY_PACK_SUMMARY,
  CAN_ID_BATTERY_CELL_EXTREMA,
  CAN_ID_BATTERY_TEMP_EXTREMA,
  CAN_ID_BATTERY_STATUS,
  CAN_ID_BATTERY_BALANCE_0,
  CAN_ID_BATTERY_BALANCE_1,
  CAN_ID_BATTERY_BALANCE_2,
  CAN_ID_BATTERY_CELL_SUM,
  CAN_ID_BATTERY_IMD_DIAG,
  CAN_ID_BATTERY_ALARM_STATUS,
  CAN_ID_BATTERY_ALARM_THRESHOLD,
  CAN_ID_BATTERY_ALARM_SWITCH,
  CAN_ID_BATTERY_TOOL_CHARGE_PARAM,
  CAN_ID_BATTERY_TOOL_ALARM_VALUE,
  CAN_ID_BATTERY_TOOL_ALARM_SWITCH,
  CAN_ID_BATTERY_TOOL_FAULT_RESET,
  CAN_ID_BATTERY_TOOL_ADC_CAL,
  CAN_ID_BATTERY_TOOL_CURRENT_DIR,
  CAN_ID_BATTERY_TOOL_RTC_SET,
  CAN_ID_BATTERY_CHARGE_REQUEST,
  CAN_ID_BATTERY_CHARGER_FEEDBACK,
  CAN_ID_BATTERY_HALL_CURRENT,
  CAN_ID_BATTERY_CELL_VOLTAGE_BASE,
  CAN_ID_BATTERY_CELL_TEMP_BASE
};

static CAN_ControllerContext_t *CAN_GetContext(CAN_HandleTypeDef *canHandle)
{
  if (canHandle == NULL)
  {
    return NULL;
  }

  if (canHandle->Instance == CAN1)
  {
    return &g_can1_ctx;
  }

  if (canHandle->Instance == CAN2)
  {
    return &g_can2_ctx;
  }

  return NULL;
}

static HAL_StatusTypeDef CAN_ConfigCan1Filters(CAN_HandleTypeDef *canHandle)
{
  CAN_FilterTypeDef filter = {0};
  uint32_t bank_index;
  uint32_t id_index = 0U;
  uint32_t id_count = (uint32_t)(sizeof(g_can1_battery_filter_ids) / sizeof(g_can1_battery_filter_ids[0]));
  uint32_t bank_count = (id_count + 1U) / 2U;

  if (canHandle == NULL)
  {
    return HAL_ERROR;
  }

  filter.FilterActivation = ENABLE;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.SlaveStartFilterBank = 14;

  for (bank_index = 0U; bank_index < bank_count; bank_index++)
  {
    uint32_t id0 = (g_can1_battery_filter_ids[id_index] <= 0x7FFUL)
        ? (((uint32_t)g_can1_battery_filter_ids[id_index]) << 21)
        : ((g_can1_battery_filter_ids[id_index] << 3) | CAN_ID_EXT);
    uint32_t id1 = 0U;

    id_index++;
    if (id_index < id_count)
    {
      id1 = (g_can1_battery_filter_ids[id_index] <= 0x7FFUL)
          ? (((uint32_t)g_can1_battery_filter_ids[id_index]) << 21)
          : ((g_can1_battery_filter_ids[id_index] << 3) | CAN_ID_EXT);
      id_index++;
    }
    else
    {
      id1 = id0;
    }

    filter.FilterBank = bank_index;
    filter.FilterIdHigh = (uint16_t)(id0 >> 16);
    filter.FilterIdLow = (uint16_t)(id0 & 0xFFFFU);
    filter.FilterMaskIdHigh = (uint16_t)(id1 >> 16);
    filter.FilterMaskIdLow = (uint16_t)(id1 & 0xFFFFU);

    if (HAL_CAN_ConfigFilter(canHandle, &filter) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef CAN_ConfigCan2Filters(CAN_HandleTypeDef *canHandle)
{
  CAN_FilterTypeDef filter = {0};
  uint32_t bank_index;
  uint32_t id_index = 0U;
  uint32_t id_count = (uint32_t)(sizeof(g_can2_vehicle_filter_ids) / sizeof(g_can2_vehicle_filter_ids[0]));
  uint32_t bank_count = (id_count + 3U) / 4U;

  if (canHandle == NULL)
  {
    return HAL_ERROR;
  }

  filter.FilterActivation = ENABLE;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.SlaveStartFilterBank = 14;

  for (bank_index = 0U; bank_index < bank_count; bank_index++)
  {
    uint16_t id0 = (uint16_t)(g_can2_vehicle_filter_ids[id_index++] << 5);
    uint16_t id1 = (id_index < id_count) ? (uint16_t)(g_can2_vehicle_filter_ids[id_index++] << 5) : id0;
    uint16_t id2 = (id_index < id_count) ? (uint16_t)(g_can2_vehicle_filter_ids[id_index++] << 5) : id1;
    uint16_t id3 = (id_index < id_count) ? (uint16_t)(g_can2_vehicle_filter_ids[id_index++] << 5) : id2;

    filter.FilterBank = 14U + bank_index;
    filter.FilterIdHigh = id0;
    filter.FilterIdLow = id1;
    filter.FilterMaskIdHigh = id2;
    filter.FilterMaskIdLow = id3;

    if (HAL_CAN_ConfigFilter(canHandle, &filter) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

static void CAN_AbortPendingTx(CAN_HandleTypeDef *canHandle, CAN_ControllerContext_t *ctx)
{
  if ((canHandle == NULL) || (ctx == NULL))
  {
    return;
  }

  ctx->last_tx_abort_status = HAL_CAN_AbortTxRequest(canHandle,
      CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
  ctx->tx_abort_count++;
  (void)HAL_CAN_ResetError(canHandle);
  ctx->last_tx_error_after_abort = HAL_CAN_GetError(canHandle);
  ctx->last_tx_free_after_abort = HAL_CAN_GetTxMailboxesFreeLevel(canHandle);
}

static void CAN_DispatchRxMessage(CAN_HandleTypeDef *hcan,
                                  CAN_ControllerContext_t *ctx,
                                  uint8_t can_channel)
{
  CAN_Msg_Queue_t can_queue_data = {0};
  osStatus_t send_status;

  if ((hcan == NULL) || (ctx == NULL))
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &ctx->rx_header, ctx->rx_data) != HAL_OK)
  {
    ctx->rx_error_count++;
    ctx->last_rx_error = HAL_CAN_GetError(hcan);
    return;
  }

  ctx->rx_count++;
  ctx->last_rx_ide = ctx->rx_header.IDE;
  ctx->last_rx_dlc = ctx->rx_header.DLC;
  ctx->last_rx_fifo_fill = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
  can_queue_data.can_channel = can_channel;
  can_queue_data.dlc = (uint8_t)ctx->rx_header.DLC;
  if (ctx->rx_header.IDE == CAN_ID_EXT)
  {
    can_queue_data.msg_id = ctx->rx_header.ExtId;
    can_queue_data.is_ext_id = 1U;
    ctx->last_rx_id = ctx->rx_header.ExtId;
  }
  else
  {
    can_queue_data.msg_id = ctx->rx_header.StdId;
    can_queue_data.is_ext_id = 0U;
    ctx->last_rx_id = ctx->rx_header.StdId;
  }
  memcpy(can_queue_data.msg_data, ctx->rx_data, 8);
  memcpy((void *)ctx->last_rx_data, ctx->rx_data, sizeof(ctx->last_rx_data));

  send_status = freertos_can_queue_send_from_isr(&can_queue_data);
  (void)send_status;
}

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
  * @note CAN1 uses filter bank 0 and accepts the battery-box bus into FIFO0.
  *       CAN2 uses filter bank 14 and accepts the vehicle CANB bus into FIFO0.
  */
void CAN_Filter_Config(CAN_HandleTypeDef *canHandle)
{
  if ((canHandle->Instance == CAN1) && (CAN_ConfigCan2Filters(canHandle) == HAL_OK))
  {
    return;
  }

  if ((canHandle->Instance == CAN2) && (CAN_ConfigCan1Filters(canHandle) == HAL_OK))
  {
    return;
  }

  if (1)
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
  CAN_ControllerContext_t *ctx = CAN_GetContext(canHandle);
  if ((ctx == NULL) || (pTxData == NULL))
  {
    return;
  }

  /* Send one standard 8-byte CAN frame through the selected controller. */
  ctx->tx_request_count++;
  ctx->last_tx_id = SendStdId;
  ctx->tx_header.RTR = CAN_RTR_DATA;
  ctx->tx_header.IDE = CAN_ID_STD;
  ctx->tx_header.StdId = SendStdId;
  ctx->tx_header.TransmitGlobalTime = DISABLE;
  ctx->tx_header.DLC = 8;
  ctx->last_tx_free_level = HAL_CAN_GetTxMailboxesFreeLevel(canHandle);
  if (ctx->last_tx_free_level == 0U)
  {
    CAN_AbortPendingTx(canHandle, ctx);
    ctx->tx_drop_count++;
    ctx->last_tx_status = HAL_ERROR;
    return;
  }

  // 添加报文到发送邮箱；发送邮箱满或总线暂不可用时丢弃本帧，避免进入 Error_Handler 卡死调度。
  if(HAL_CAN_AddTxMessage(canHandle, &ctx->tx_header, pTxData, &ctx->tx_mailbox) != HAL_OK)
  {
    ctx->last_tx_status = HAL_ERROR;
    ctx->last_tx_error = HAL_CAN_GetError(canHandle);
    ctx->tx_drop_count++;
    if (HAL_CAN_GetTxMailboxesFreeLevel(canHandle) == 0U)
    {
      CAN_AbortPendingTx(canHandle, ctx);
    }
    return;
  }

  ctx->last_tx_status = HAL_OK;
  ctx->last_tx_error = HAL_CAN_GetError(canHandle);
  ctx->tx_success_count++;
}

/**
  * @brief CAN控制器统一启动函数（上层调用入口，一键完成初始化）
  * @note 按CAN1→CAN2顺序启动，保证CAN2依赖的CAN1时钟正常，启动后使能FIFO0中断
  * @usage 主函数中调用：CAN_Start(); 即可完成所有CAN配置
  */
void CAN_Start(void)
{
  CAN_ControllerContext_t *can1_ctx = &g_can1_ctx;
  CAN_ControllerContext_t *can2_ctx = &g_can2_ctx;

//  MX_CAN1_Init();
//  MX_CAN2_Init();
  /* Start both controllers only after filters have been installed. */
  CAN_Filter_Config(&hcan1);
  CAN_Filter_Config(&hcan2);
  can1_ctx->start_status = HAL_CAN_Start(&hcan1);
  can2_ctx->start_status = HAL_CAN_Start(&hcan2);
  // 使能CAN1/CAN2 FIFO0报文挂起中断（核心接收中断）
  can1_ctx->notify_status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  can2_ctx->notify_status = HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief CAN1 FIFO0报文挂起中断回调函数（处理电机/IMU等标准帧，无电池逻辑）
  * @param hcan CAN句柄指针（仅处理hcan1实例）
  * @note 保留原有业务逻辑入口，接收后可根据ID做对应处理，不影响电池数据
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == NULL)
  {
    return;
  }

  if (hcan->Instance == CAN1)
  {
    CAN_DispatchRxMessage(hcan, &g_can1_ctx, CAN_CHANNEL_BATTERY_BOX);
    return;
  }

  if (hcan->Instance == CAN2)
  {
    CAN_DispatchRxMessage(hcan, &g_can2_ctx, CAN_CHANNEL_VEHICLE_CANB);
    // app_can_debug_log_rx_context("CAN2 RX", &g_can2_ctx);
  }
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
