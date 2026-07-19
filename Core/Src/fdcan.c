/*
 * fdcan.c
 *
 *  Created on: Jul 6, 2026
 *      Author: ArianaJia
 */

#include "fdcan.h"

#include <string.h>

#include "main.h"
#include "can.h"
#include "mcp2518fd.h"
#include "spi.h"

static MCP2518FD_Handle_t g_fdcan_bus_a;
static MCP2518FD_Handle_t g_fdcan_bus_b;

#define FDCAN_BUS_A_EXACT_FILTER_ENABLED 1U

static const uint16_t g_fdcan_bus_a_filters[] = {
  CAN_ID_CANA_FR_MOTOR_CONTROL_CMD,
  CAN_ID_CANA_FL_MOTOR_CONTROL_CMD,
  CAN_ID_CANA_RL_MOTOR_CONTROL_CMD,
  CAN_ID_CANA_RR_MOTOR_CONTROL_CMD,
  CAN_ID_CANA_FR_STATUS_SPEED_TORQUE_CURRENT,
  CAN_ID_CANA_FL_STATUS_SPEED_TORQUE_CURRENT,
  CAN_ID_CANA_FR_DIAGNOSTIC_INFO,
  CAN_ID_CANA_FL_DIAGNOSTIC_INFO,
  CAN_ID_CANA_RR_STATUS_SPEED_TORQUE_CURRENT,
  CAN_ID_CANA_RL_STATUS_SPEED_TORQUE_CURRENT,
  CAN_ID_CANA_RR_DIAGNOSTIC_INFO,
  CAN_ID_CANA_RL_DIAGNOSTIC_INFO,
  CAN_ID_CANA_FR_MOTOR_INVERTER_IGBT_TEMP_DCBUS,
  CAN_ID_CANA_FL_MOTOR_INVERTER_IGBT_TEMP_DCBUS,
  CAN_ID_CANA_RR_MOTOR_INVERTER_IGBT_TEMP_DCBUS,
  CAN_ID_CANA_RL_MOTOR_INVERTER_IGBT_TEMP_DCBUS,
  CAN_ID_CANA_FR_TORQUE_CURRENT_LIMIT_COUNTER,
  CAN_ID_CANA_FL_TORQUE_CURRENT_LIMIT_COUNTER,
  CAN_ID_CANA_RR_TORQUE_CURRENT_LIMIT_COUNTER,
  CAN_ID_CANA_RL_TORQUE_CURRENT_LIMIT_COUNTER
};

/* Map the logical bus selector to its backing MCP2518FD handle. */
static MCP2518FD_Handle_t *FDCAN_GetHandle(FDCAN_Bus_t bus);

/* Initialize one CAN bus controller and bind it to the correct SPI chip select. */
HAL_StatusTypeDef FDCAN_Init(FDCAN_Bus_t bus)
{
  MCP2518FD_Handle_t *handle = FDCAN_GetHandle(bus);

  if (handle == NULL)
  {
    return HAL_ERROR;
  }

  if (bus == FDCAN_BUS_A)
  {
#if (FDCAN_BUS_A_EXACT_FILTER_ENABLED != 0U)
    MCP2518FD_SetStdFilters(handle,
                             g_fdcan_bus_a_filters,
                             (uint8_t)(sizeof(g_fdcan_bus_a_filters) / sizeof(g_fdcan_bus_a_filters[0])));
#else
    MCP2518FD_SetStdFilters(handle, NULL, 0U);
#endif
    return MCP2518FD_Init(handle, &hspi1, MCP2518_CS_A_GPIO_Port, MCP2518_CS_A_Pin);
  }

  MCP2518FD_SetStdFilters(handle, NULL, 0U);
  return MCP2518FD_Init(handle, &hspi3, MCP2518_CS_B_GPIO_Port, MCP2518_CS_B_Pin);
}

/* Check whether the selected CAN bus controller has already been initialized. */
uint8_t FDCAN_IsReady(FDCAN_Bus_t bus)
{
  return MCP2518FD_IsReady(FDCAN_GetHandle(bus));
}

/* Poll one received frame from the selected CAN bus and convert it to the local frame format. */
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

/* Transmit one standard CAN frame through the selected bus controller. */
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

/* Return the backing handle for bus A or bus B, or NULL for invalid input. */
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
