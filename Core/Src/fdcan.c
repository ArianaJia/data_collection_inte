/*
 * fdcan.c
 *
 *  Created on: Jul 6, 2026
 *      Author: ArianaJia
 */

#include "fdcan.h"

#include <string.h>

#include "main.h"
#include "mcp2518fd.h"
#include "spi.h"

static MCP2518FD_Handle_t g_fdcan_bus_a;
static MCP2518FD_Handle_t g_fdcan_bus_b;

static MCP2518FD_Handle_t *FDCAN_GetHandle(FDCAN_Bus_t bus);

HAL_StatusTypeDef FDCAN_Init(FDCAN_Bus_t bus)
{
  MCP2518FD_Handle_t *handle = FDCAN_GetHandle(bus);

  if (handle == NULL)
  {
    return HAL_ERROR;
  }

  if (bus == FDCAN_BUS_A)
  {
    return MCP2518FD_Init(handle, &hspi1, MCP2518_CS_A_GPIO_Port, MCP2518_CS_A_Pin);
  }

  return MCP2518FD_Init(handle, &hspi3, MCP2518_CS_B_GPIO_Port, MCP2518_CS_B_Pin);
}

uint8_t FDCAN_IsReady(FDCAN_Bus_t bus)
{
  return MCP2518FD_IsReady(FDCAN_GetHandle(bus));
}

HAL_StatusTypeDef FDCAN_Poll(FDCAN_Bus_t bus, FDCAN_StdFrame_t *frame, uint8_t *received)
{
  MCP2518FD_Handle_t *handle = FDCAN_GetHandle(bus);
  MCP2518FD_StdFrame_t raw_frame;
  HAL_StatusTypeDef status;

  if ((handle == NULL) || (frame == NULL) || (received == NULL))
  {
    return HAL_ERROR;
  }

  status = MCP2518FD_PollReceive(handle, &raw_frame, received);
  if ((status != HAL_OK) || (*received == 0U))
  {
    return status;
  }

  frame->std_id = raw_frame.id;
  frame->dlc = raw_frame.dlc;
  frame->length = raw_frame.length;
  frame->frame_format = raw_frame.frame_format;
  frame->bit_rate_switch = raw_frame.bit_rate_switch;
  frame->esi = raw_frame.esi;
  (void)memset(frame->data, 0, sizeof(frame->data));
  (void)memcpy(frame->data, raw_frame.data, raw_frame.length);

  return HAL_OK;
}

HAL_StatusTypeDef FDCAN_TransmitStd(FDCAN_Bus_t bus, const FDCAN_StdFrame_t *frame)
{
  MCP2518FD_Handle_t *handle = FDCAN_GetHandle(bus);
  MCP2518FD_StdFrame_t raw_frame;

  if ((handle == NULL) || (frame == NULL) || (frame->length > 64U) || (frame->std_id > 0x7FFU))
  {
    return HAL_ERROR;
  }

  raw_frame.frame_format = frame->frame_format;
  raw_frame.bit_rate_switch = frame->bit_rate_switch;
  raw_frame.esi = frame->esi;
  raw_frame.id = frame->std_id;
  raw_frame.dlc = frame->dlc;
  raw_frame.length = frame->length;
  (void)memset(raw_frame.data, 0, sizeof(raw_frame.data));
  (void)memcpy(raw_frame.data, frame->data, frame->length);

  return MCP2518FD_TransmitStd(handle, &raw_frame);
}

static MCP2518FD_Handle_t *FDCAN_GetHandle(FDCAN_Bus_t bus)
{
  switch (bus)
  {
    case FDCAN_BUS_A:
      return &g_fdcan_bus_a;

    case FDCAN_BUS_B:
      return &g_fdcan_bus_b;

    default:
      return NULL;
  }
}
