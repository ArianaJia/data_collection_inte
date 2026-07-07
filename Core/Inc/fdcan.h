/*
 * fdcan.h
 *
 *  Created on: Jul 6, 2026
 *      Author: ArianaJia
 *
 *  This module keeps the historical "fdcan" name used by the project.
 *  On STM32F407 it is only a CAN abstraction layer for an external
 *  MCP2518FD controller connected over SPI.
 *  It does not use the STM32 FDCAN peripheral.
 */

#ifndef INC_FDCAN_H_
#define INC_FDCAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef enum
{
  FDCAN_BUS_A = 0,
  FDCAN_BUS_B
} FDCAN_Bus_t;

typedef enum
{
  FDCAN_FRAME_CLASSIC = 0,
  FDCAN_FRAME_FD = 1
} FDCAN_FrameFormat_t;

typedef struct
{
  uint8_t frame_format;
  uint8_t bit_rate_switch;
  uint8_t esi;
  uint32_t std_id;
  uint8_t dlc;
  uint8_t length;
  uint8_t data[64];
} FDCAN_StdFrame_t;

HAL_StatusTypeDef FDCAN_Init(FDCAN_Bus_t bus);
uint8_t FDCAN_IsReady(FDCAN_Bus_t bus);
HAL_StatusTypeDef FDCAN_Poll(FDCAN_Bus_t bus, FDCAN_StdFrame_t *frame, uint8_t *received);
HAL_StatusTypeDef FDCAN_TransmitStd(FDCAN_Bus_t bus, const FDCAN_StdFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* INC_FDCAN_H_ */
