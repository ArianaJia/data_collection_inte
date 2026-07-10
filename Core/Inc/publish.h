#ifndef __PUBLISH_H
#define __PUBLISH_H

#ifdef __cplusplus
extern "C" {
#endif

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
 *      i=1 -> REAR_RIGHT
 *      i=2 -> FRONT_LEFT
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
 * Fields that currently have no direct CANB source are initialized to zero or
 * UNSPECIFIED:
 *   brake_position
 *   vcu_status
 */

#include <stdint.h>
#include <stdbool.h>

#include "cmsis_os2.h"
#include "fsae_telemetry.pb.h"


#define PUBLISH_FAST_PERIOD_MS         100U
#define PUBLISH_MEDIUM_PERIOD_MS       200U
#define PUBLISH_SLOW_PERIOD_MS         500U

#define PUBLISH_FRAME_MAGIC_0          0x46U
#define PUBLISH_FRAME_MAGIC_1          0x53U
#define PUBLISH_FRAME_HEADER_SIZE         5U
#define PUBLISH_MAX_TOPIC_LEN            32U
#define PUBLISH_MAX_PAYLOAD_SIZE       FSAE_FSAE_TELEMETRY_PB_H_MAX_SIZE
#define PUBLISH_MAX_FRAME_SIZE         (PUBLISH_FRAME_HEADER_SIZE + PUBLISH_MAX_TOPIC_LEN + PUBLISH_MAX_PAYLOAD_SIZE)
#define PUBLISH_BMS_DETAIL_CELL_COUNT    23U
typedef enum
{
    PUBLISH_TOPIC_FAST_TELEMETRY = 0,
    PUBLISH_TOPIC_BMS_SUMMARY,
    PUBLISH_TOPIC_VEHICLE_STATE,
    PUBLISH_TOPIC_BMS_DETAIL,
    PUBLISH_TOPIC_THERMAL_SUMMARY
} PublishTopic_t;

#define PUBLISH_TOPIC_COUNT  ((uint8_t)PUBLISH_TOPIC_THERMAL_SUMMARY + 1U)

typedef struct
{
    PublishTopic_t topic;
} PublishQueueItem_t;

void Publish_Init(void);
void Publish_Process(void);
osStatus_t Publish_QueueTopic(PublishTopic_t topic);
void Publish_OnTopicDequeued(PublishTopic_t topic);
bool Publish_BuildFrame(PublishTopic_t topic, uint8_t *frame_buffer, uint16_t frame_capacity, uint16_t *frame_size);

void Publish_MapCanbVehicleState(fsae_VehicleState *vehicle_state);

#ifdef __cplusplus
}
#endif

#endif /* __PUBLISH_H */
