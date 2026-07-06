#ifndef __PUBLISH_H
#define __PUBLISH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "cmsis_os2.h"
#include "fsae_telemetry.pb.h"

typedef enum
{
    PUBLISH_TOPIC_FAST_TELEMETRY = 0,
    PUBLISH_TOPIC_BMS_SUMMARY,
    PUBLISH_TOPIC_VEHICLE_STATE,
    PUBLISH_TOPIC_BMS_DETAIL,
    PUBLISH_TOPIC_THERMAL_SUMMARY
} PublishTopic_t;

typedef struct
{
    PublishTopic_t topic;
} PublishQueueItem_t;

void Publish_Init(void);
void Publish_Process(void);
osStatus_t Publish_QueueTopic(PublishTopic_t topic);
void Publish_TxTaskStep(uint32_t timeout_ms);

/*
 * CANB loop -> fsae_VehicleState mapping
 *
 * vehicle_state.speed_kmh
 *   <- g_CANB_LoopData.ECU.Vehicle_Speed
 *
 * vehicle_state.driving_mode
 *   <- g_CANB_LoopData.ECU.driving_mode
 *
 * vehicle_state.motors_count
 *   <- fixed 4
 *
 * vehicle_state.motors[i].position
 *   <- CANB motor array order:
 *      i=0 -> REAR_LEFT
 *      i=1 -> FRONT_LEFT
 *      i=2 -> REAR_RIGHT
 *      i=3 -> FRONT_RIGHT
 *
 * vehicle_state.motors[i].rpm
 *   <- g_CANB_LoopData.ECU.Motor_RPM[i]
 *
 * vehicle_state.motors[i].torque_nm
 *   <- g_CANB_LoopData.ECU.Motor_Torque[i]
 *
 * vehicle_state.motors[i].power_w
 *   <- g_CANB_LoopData.ECU.Motor_Power[i] * 1000
 *      (CANB stores kW, protobuf expects W)
 *
 * vehicle_state.motors[i].motor_error
 *   <- g_CANB_LoopData.ECU.ERRO[i]
 *
 * Fields that currently have no CANB source are initialized to zero or
 * UNSPECIFIED:
 *   throttle_position
 *   brake_position
 *   ready_to_drive
 *   vcu_status
 *   motor_temp_dc
 *   inverter_temp_dc
 */
void Publish_MapCanbVehicleState(fsae_VehicleState *vehicle_state);

#ifdef __cplusplus
}
#endif

#endif /* __PUBLISH_H */
