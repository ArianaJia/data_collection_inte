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
/************************** CAN bus/channel ownership **************************/
/* Physical/virtual channel mapping used by CAN_Msg_Queue_t.can_channel. */
#define CAN_CHANNEL_BATTERY_BOX           1U  /* STM32 CAN1: battery box bus. */
#define CAN_CHANNEL_VEHICLE_CANB          2U  /* STM32 CAN2: vehicle CANB bus. */
#define CAN_CHANNEL_CONTROLLER_CANA       3U  /* MCP2518/SPI1: controller CANA bus, reserved for AMK frames. */
#define CAN_CHANNEL_CDC_MONITOR           4U  /* MCP2518/SPI3: CDC monitor bus, read-only monitoring path. */

/************************** Battery box bus mailbox IDs **************************/
#define CAN_ID_BATTERY_CELL_VOLTAGE_BASE  0x180050F3UL
#define CAN_ID_BATTERY_CELL_TEMP_BASE     0x184050F3UL
#define CAN_ID_BATTERY_PACK_SUMMARY       0x186050F4UL
#define CAN_ID_BATTERY_CELL_EXTREMA       0x186150F4UL
#define CAN_ID_BATTERY_TEMP_EXTREMA       0x186250F4UL
#define CAN_ID_BATTERY_STATUS             0x186350F4UL
#define CAN_ID_BATTERY_BALANCE_0          0x186450F4UL
#define CAN_ID_BATTERY_BALANCE_1          0x186550F4UL
#define CAN_ID_BATTERY_BALANCE_2          0x186650F4UL
#define CAN_ID_BATTERY_CELL_SUM           0x186750F4UL
#define CAN_ID_BATTERY_IMD_DIAG           0x186850F4UL
#define CAN_ID_BATTERY_ALARM_STATUS       0x187650F4UL
#define CAN_ID_BATTERY_ALARM_THRESHOLD    0x187750F4UL
#define CAN_ID_BATTERY_ALARM_SWITCH       0x187F50F4UL
#define CAN_ID_BATTERY_TOOL_CHARGE_PARAM  0x188050F5UL
#define CAN_ID_BATTERY_TOOL_ALARM_VALUE   0x188150F5UL
#define CAN_ID_BATTERY_TOOL_ALARM_SWITCH  0x188250F5UL
#define CAN_ID_BATTERY_TOOL_FAULT_RESET   0x188350F5UL
#define CAN_ID_BATTERY_TOOL_ADC_CAL       0x18A050F5UL
#define CAN_ID_BATTERY_TOOL_CURRENT_DIR   0x18A150F5UL
#define CAN_ID_BATTERY_TOOL_RTC_SET       0x18A350F5UL
#define CAN_ID_BATTERY_CHARGE_REQUEST     0x1806E5F4UL
#define CAN_ID_BATTERY_CHARGER_FEEDBACK   0x18FF50E5UL
#define CAN_ID_BATTERY_HALL_CURRENT       0x03C0U

#define CAN_BATTERY_MODULE_COUNT          6U
#define CAN_BATTERY_CELL_VOLTAGE_FRAMES_PER_MODULE 6U
#define CAN_BATTERY_CELL_VOLTAGE_FRAME_COUNT (CAN_BATTERY_MODULE_COUNT * CAN_BATTERY_CELL_VOLTAGE_FRAMES_PER_MODULE)
#define CAN_BATTERY_CELL_TEMP_FRAME_COUNT CAN_BATTERY_MODULE_COUNT
#define CAN_BATTERY_EXT_ID_STEP           0x00010000UL
#define CAN_ID_BATTERY_CELL_VOLTAGE_LAST  (CAN_ID_BATTERY_CELL_VOLTAGE_BASE + ((CAN_BATTERY_CELL_VOLTAGE_FRAME_COUNT - 1U) << 16))
#define CAN_ID_BATTERY_CELL_TEMP_LAST     (CAN_ID_BATTERY_CELL_TEMP_BASE + ((CAN_BATTERY_CELL_TEMP_FRAME_COUNT - 1U) << 16))

/************************** Vehicle CANB mailbox IDs **************************/
#define CAN_ID_CANB_GPS_SPEED            0x0301U
#define CAN_ID_CANB_DRIVER_INPUTS_OIL_PRESSURE 0x0305U
#define CAN_ID_CANB_DRIVING_MODE_CMD      0x0310U
#define CAN_ID_CANB_BATTERY_POWER_STATUS  0x0401U
#define CAN_ID_CANB_BATTERY_FAULT_STATUS  0x0402U
#define CAN_ID_CANB_ENERGY_METER_STATUS   0x0430U
#define CAN_ID_CANB_LEGACY_VEHICLE_STATE  0x0501U
#define CAN_ID_CANB_WHEEL_ACTUAL_TORQUE   0x0502U
#define CAN_ID_CANB_REAR_MOTOR_DIAGNOSTIC_CODE 0x0503U
#define CAN_ID_CANB_FRONT_MOTOR_DIAGNOSTIC_CODE 0x0504U
#define CAN_ID_CANB_WHEEL_ACTUAL_SPEED    0x0505U
#define CAN_ID_CANB_MOTOR_TEMPERATURE     0x0506U
#define CAN_ID_CANB_INVERTER_TEMPERATURE  0x0507U
#define CAN_ID_CANB_IGBT_TEMPERATURE      0x0508U
#define CAN_ID_CANB_AMK_STATUS_MODE_LOGIC 0x0509U
#define CAN_ID_CANB_THERMAL_SUMMARY_FL    0x0071U
#define CAN_ID_CANB_THERMAL_SUMMARY_FR    0x0072U
#define CAN_ID_CANB_THERMAL_SUMMARY_RL    0x0073U
#define CAN_ID_CANB_THERMAL_SUMMARY_RR    0x0074U
#define CAN_ID_IVT_CURRENT                0x0521U
#define CAN_ID_IVT_U1                     0x0522U
#define CAN_ID_IVT_POWER                  0x0526U
#define CAN_ID_IVT_WH                     0x0528U
#define CAN_ID_IMU_RAW                    0x0050U
#define CAN_ID_IMU_TIME                   0x0060U
#define CAN_ID_IMU_ACCEL                  0x0061U
#define CAN_ID_IMU_GYRO                   0x0062U
#define CAN_ID_IMU_ROLL                   0x0063U
#define CAN_ID_IMU_PITCH                  0x0064U
#define CAN_ID_IMU_YAW                    0x0065U
#define CAN_ID_IMU_MAGNETIC               0x0066U

/************************** Controller CANA mailbox IDs **************************/
#define CAN_ID_CANA_FR_MOTOR_CONTROL_CMD  0x0184U
#define CAN_ID_CANA_FL_MOTOR_CONTROL_CMD  0x0185U
#define CAN_ID_CANA_RL_MOTOR_CONTROL_CMD  0x0188U
#define CAN_ID_CANA_RR_MOTOR_CONTROL_CMD  0x0189U
#define CAN_ID_CANA_FR_STATUS_SPEED_TORQUE_CURRENT 0x0283U
#define CAN_ID_CANA_FL_STATUS_SPEED_TORQUE_CURRENT 0x0284U
#define CAN_ID_CANA_FR_DIAGNOSTIC_INFO    0x0285U
#define CAN_ID_CANA_FL_DIAGNOSTIC_INFO    0x0286U
#define CAN_ID_CANA_RR_STATUS_SPEED_TORQUE_CURRENT 0x0287U
#define CAN_ID_CANA_RL_STATUS_SPEED_TORQUE_CURRENT 0x0288U
#define CAN_ID_CANA_RR_DIAGNOSTIC_INFO    0x0289U
#define CAN_ID_CANA_RL_DIAGNOSTIC_INFO    0x028AU
#define CAN_ID_CANA_FR_MOTOR_INVERTER_IGBT_TEMP_DCBUS 0x028BU
#define CAN_ID_CANA_FL_MOTOR_INVERTER_IGBT_TEMP_DCBUS 0x028CU
#define CAN_ID_CANA_RR_MOTOR_INVERTER_IGBT_TEMP_DCBUS 0x028DU
#define CAN_ID_CANA_RL_MOTOR_INVERTER_IGBT_TEMP_DCBUS 0x028EU
#define CAN_ID_CANA_FR_TORQUE_CURRENT_LIMIT_COUNTER 0x028FU
#define CAN_ID_CANA_FL_TORQUE_CURRENT_LIMIT_COUNTER 0x0290U
#define CAN_ID_CANA_RR_TORQUE_CURRENT_LIMIT_COUNTER 0x0291U
#define CAN_ID_CANA_RL_TORQUE_CURRENT_LIMIT_COUNTER 0x0292U
/************************** CAN通信专属全局变量（仅CAN驱动使用，无电池相关变量） **************************/
// 声明CAN通信专属结构体（供外部文件引用）,封装CAN通信专属全局变量到结构体，保留所有原有变量名
typedef struct
{
  CAN_TxHeaderTypeDef  tx_header;
  CAN_RxHeaderTypeDef  rx_header;
  uint8_t              tx_data[8];
  uint8_t              rx_data[8];
  uint32_t             tx_mailbox;
  volatile uint32_t    start_status;
  volatile uint32_t    notify_status;
  volatile uint32_t    last_tx_status;
  volatile uint32_t    last_tx_error;
  volatile uint32_t    last_tx_free_level;
  volatile uint32_t    tx_request_count;
  volatile uint32_t    tx_success_count;
  volatile uint32_t    tx_drop_count;
  volatile uint32_t    tx_abort_count;
  volatile uint32_t    last_tx_abort_status;
  volatile uint32_t    last_tx_free_after_abort;
  volatile uint32_t    last_tx_error_after_abort;
  volatile uint32_t    rx_count;
  volatile uint32_t    rx_error_count;
  volatile uint32_t    last_rx_error;
  volatile uint32_t    last_tx_id;
  volatile uint32_t    last_rx_id;
  volatile uint32_t    last_rx_ide;
  volatile uint32_t    last_rx_dlc;
  volatile uint32_t    last_rx_fifo_fill;
  volatile uint8_t     last_rx_data[8];
} CAN_ControllerContext_t;

// 声明FreeRTOS CAN队列数据结构体（核心：供freertos.c调用）
typedef struct
{
    uint8_t can_channel;   // 1=CAN1，2=CAN2
    uint32_t msg_id;       // 标准帧/StdId，扩展帧/ExtId
    uint8_t msg_data[8];   // CAN报文数据
    uint8_t dlc;           // Valid payload length, 0..8 for classic CAN frames.
    uint8_t is_ext_id;     // 0=标准帧，1=扩展帧
} CAN_Msg_Queue_t;


typedef struct
{
    uint32_t std_id;
    uint8_t data[8];
} CAN_Tx_Queue_t;

extern CAN_ControllerContext_t g_can1_ctx;
extern CAN_ControllerContext_t g_can2_ctx;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_Filter_Config(CAN_HandleTypeDef *canHandle);
void User_CAN1_Send(void);
void CAN_Send_Msg(CAN_HandleTypeDef *canHandle, uint32_t SendStdId, uint8_t *pTxData);
void CAN_Start(void);
osStatus_t freertos_can_queue_send_from_isr(const CAN_Msg_Queue_t *pData);
osStatus_t freertos_vehicle_can_tx_queue_put(const CAN_Tx_Queue_t *pData, uint32_t timeout_ms);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

