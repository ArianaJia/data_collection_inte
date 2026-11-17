#ifndef __CAN_DECODE_H__
#define __CAN_DECODE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"
#include "telemetry_updates.h"

uint8_t CAN_IsBatteryCanbMessage(const CAN_Msg_Queue_t *frame);
TelemetryUpdates CAN_DecodeBatteryService(uint32_t now);
TelemetryUpdates CAN_DecodeVehicleService(uint32_t now);
/* Task-context APIs: accepted cache groups, including zero/status updates. */
TelemetryUpdates CAN_DecodeVehicleCanbMessage(const CAN_Msg_Queue_t *recv_data);
TelemetryUpdates CAN_DecodeBatteryBoxMessage(const CAN_Msg_Queue_t *recv_data);
TelemetryUpdates CAN_DecodeControllerCanaMessage(const CAN_Msg_Queue_t *recv_data);
TelemetryUpdates CAN_DecodeCdcMonitorMessage(const CAN_Msg_Queue_t *recv_data);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_DECODE_H__ */
