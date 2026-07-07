#ifndef __CAN_DECODE_H__
#define __CAN_DECODE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"

void CAN_DecodeVehicleCanbMessage(const CAN_Msg_Queue_t *recv_data);
void CAN_DecodeBatteryBoxMessage(const CAN_Msg_Queue_t *recv_data);
void CAN_DecodeControllerCanaMessage(const CAN_Msg_Queue_t *recv_data);
void CAN_DecodeCdcMonitorMessage(const CAN_Msg_Queue_t *recv_data);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_DECODE_H__ */
