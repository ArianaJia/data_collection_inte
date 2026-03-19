/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "cmsis_os2.h" // 引入CMSIS-RTOS V2类型定义
#include "string.h"
#include "freertos.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
/************************** CAN通信专属全局变量（仅CAN驱动使用，无电池相关变量） **************************/
// 声明CAN通信专属结构体（供外部文件引用）,封装CAN通信专属全局变量到结构体，保留所有原有变量名
typedef struct
{
  CAN_TxHeaderTypeDef  g_CANTxHeader;
  CAN_RxHeaderTypeDef  g_CANRxHeader;
  uint8_t              g_CANTxData[8];
  uint8_t              g_CANRxData[8];
  uint32_t             g_CANTxMailBox;
  uint32_t             g_CAN1DefSendID;
} can_msg;

// 声明FreeRTOS CAN队列数据结构体（核心：供freertos.c调用）
typedef struct
{
    uint8_t can_channel;   // 1=CAN1，2=CAN2
    uint32_t msg_id;       // 标准帧/StdId，扩展帧/ExtId
    uint8_t msg_data[8];   // CAN报文数据
    uint8_t is_ext_id;     // 0=标准帧，1=扩展帧
} CAN_Msg_Queue_t;

// 声明CAN通信结构体实例（修正命名冲突）
extern can_msg g_can_msg;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_Filter_Config(CAN_HandleTypeDef *canHandle);
void User_CAN1_Send(void);
void CAN_Send_Msg(CAN_HandleTypeDef *canHandle, uint32_t SendStdId, uint8_t *pTxData);
void CAN_Start(void);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

