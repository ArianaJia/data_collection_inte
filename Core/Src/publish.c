/* Publish converts the shared telemetry caches into protobuf payloads and
 * hands topic requests to task08 through myQueue04.
 */
#include "publish.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <string.h>

#include "telemetry_data.h"
#include "pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "freertos_app.h"
#include "usart.h"

extern osMessageQueueId_t PublishQueueItemHandle;

static fsae_FastTelemetry g_publishFastTelemetry;
static fsae_TelemetryFrame g_publishTelemetryFrame;
static fsae_VehicleState g_publishVehicleState;
static fsae_ThermalSummary g_publishThermalSummary;
static volatile uint32_t g_publishQueuedTopicMask = 0U;

static fsae_DrivingMode Publish_MapDrivingMode(uint8_t raw_mode);
static uint32_t Publish_GetVehicleSpeedKmh(void);
static fsae_MotorPosition Publish_MapCanbMotorPosition(uint8_t motor_index);
static uint8_t Publish_MapMotorOrderToThermalSensorIndex(uint8_t motor_index);
static void Publish_MapCanbMotorState(fsae_MotorState *motor_state, uint8_t motor_index);
static void Publish_MapFastTelemetry(fsae_FastTelemetry *fast_telemetry);
/* Export the raw IVT cache into protobuf form. */
static bool Publish_MapIvtTelemetry(fsae_IvtTelemetry *ivt_telemetry);
/* Export the unified energy meter view.
 * Source may be IVT or FS, but protobuf sees one stable message shape.
 */
static bool Publish_MapEnergyMeterTelemetry(fsae_EnergyMeterTelemetry *energy_meter);
/* Export the CANB/GPS/IMU subset currently used by protobuf motion telemetry. */
static bool Publish_MapMotionTelemetry(fsae_MotionTelemetry *motion);
/* Fill the shared TelemetryFrame fields used by both BMS summary and detail
 * topics so they stay consistent on the wire.
 */
static void Publish_MapCommonTelemetryFrame(fsae_TelemetryFrame *frame);
static void Publish_MapBmsSummaryFrame(fsae_TelemetryFrame *frame);
static void Publish_MapBmsDetailFrame(fsae_TelemetryFrame *frame);
static void Publish_MapBatteryModule(fsae_BatteryModule *module, uint8_t module_index);
static void Publish_MapThermalSummary(fsae_ThermalSummary *thermal_summary);
static bool Publish_EncodeTopicPayload(PublishTopic_t topic, uint8_t *payload_buffer, size_t payload_capacity, size_t *payload_size);
static const char *Publish_GetTopicName(PublishTopic_t topic);
static bool Publish_IsTopicValid(PublishTopic_t topic);

void Publish_Init(void)
{
    /* Publish keeps no private runtime state at startup. */
}

void Publish_Process(void)
{
    /* Publish requests are now producer-driven:
     * data-producing tasks enqueue topics for task08 after updating caches.
     */
}

osStatus_t Publish_QueueTopic(PublishTopic_t topic)
{
    /* Coalesce duplicate requests so one topic is queued at most once. */
    PublishQueueItem_t item;
    uint32_t topic_mask;
    osStatus_t status;

    if ((PublishQueueItemHandle == NULL) || !Publish_IsTopicValid(topic))
    {
        return osErrorResource;
    }

    topic_mask = (1UL << (uint32_t)topic);

    taskENTER_CRITICAL();
    if ((g_publishQueuedTopicMask & topic_mask) != 0U)
    {
        taskEXIT_CRITICAL();
        return osOK;
    }
    g_publishQueuedTopicMask |= topic_mask;
    taskEXIT_CRITICAL();

    item.topic = topic;
    status = osMessageQueuePut(PublishQueueItemHandle, &item, 0U, 0U);
    if (status != osOK)
    {
        taskENTER_CRITICAL();
        g_publishQueuedTopicMask &= ~topic_mask;
        taskEXIT_CRITICAL();
    }

    return status;
}

void Publish_OnTopicDequeued(PublishTopic_t topic)
{
    /* Release the deduplication bit once task08 has consumed the request. */
    if (!Publish_IsTopicValid(topic))
    {
        return;
    }

    taskENTER_CRITICAL();
    g_publishQueuedTopicMask &= ~(1UL << (uint32_t)topic);
    taskEXIT_CRITICAL();
}

bool Publish_BuildFrame(PublishTopic_t topic, uint8_t *frame_buffer, uint16_t frame_capacity, uint16_t *frame_size)
{
    size_t payload_size = 0U;

    /* Route A uplink publishes raw protobuf to one MQTT topic and does not use
     * the legacy FS wrapper. For the current GPS verification path, vehicle_state
     * is repointed to a TelemetryFrame so motion.gps_speed_kmh can be sent to
     * the cloud without touching BMS payload selection.
     */
    if ((frame_buffer == NULL) || (frame_size == NULL))
    {
        return false;
    }

    if (!Publish_EncodeTopicPayload(topic,
                                    frame_buffer,
                                    frame_capacity,
                                    &payload_size))
    {
        return false;
    }

    *frame_size = (uint16_t)payload_size;

    return true;
}

void Publish_MapCanbVehicleState(fsae_VehicleState *vehicle_state)
{
    if (vehicle_state == NULL)
    {
        return;
    }

    *vehicle_state = (fsae_VehicleState)fsae_VehicleState_init_zero;
    vehicle_state->speed_kmh = Publish_GetVehicleSpeedKmh();
    vehicle_state->driving_mode = Publish_MapDrivingMode(g_CANB_LoopData.ECU.driving_mode);
    vehicle_state->motors_count = 4U;

    vehicle_state->throttle_position = (uint32_t)((g_CANB_LoopData.ECU.ThrottlePositionDeciPct + 5U) / 10U);
    vehicle_state->brake_position = 0U;
    vehicle_state->vcu_status = fsae_VcuStatus_VCU_STATUS_UNSPECIFIED;

    for (uint8_t i = 0U; i < vehicle_state->motors_count; i++)
    {
        Publish_MapCanbMotorState(&vehicle_state->motors[i], i);
    }
}

static uint32_t Publish_GetVehicleSpeedKmh(void)
{
    /* Prefer GPS speed when it is valid; otherwise fall back to CANB speed. */
    /* GPS speed is preferred when available because it is less ambiguous than
     * the legacy vehicle-speed byte carried by older CANB packets.
     */
    if ((g_CANB_LoopData.IMU.ValidFlags & CANB_MOTION_VALID_GPS) != 0U)
    {
        return (uint32_t)g_CANB_LoopData.IMU.GpsSpeedKmh;
    }

    return (uint32_t)g_CANB_LoopData.ECU.Vehicle_Speed;
}

static fsae_DrivingMode Publish_MapDrivingMode(uint8_t raw_mode)
{
    /* Normalize the project-specific driving mode byte into protobuf enums. */
    switch (raw_mode)
    {
        case 1U:
            return fsae_DrivingMode_DRIVING_MODE_DEFAULT;
        case 2U:
            return fsae_DrivingMode_DRIVING_MODE_STRAIGHT;
        case 3U:
            return fsae_DrivingMode_DRIVING_MODE_AUTOCROSS;
        case 4U:
            return fsae_DrivingMode_DRIVING_MODE_SKIDPAD;
        case 5U:
            return fsae_DrivingMode_DRIVING_MODE_ENDURANCE;
        default:
            return fsae_DrivingMode_DRIVING_MODE_UNSPECIFIED;
    }
}

static fsae_MotorPosition Publish_MapCanbMotorPosition(uint8_t motor_index)
{
    /* Keep the CANB motor order stable when mapping to protobuf positions. */
    switch (motor_index)
    {
        case 0U:
            return fsae_MotorPosition_MOTOR_POSITION_REAR_LEFT;
        case 1U:
            return fsae_MotorPosition_MOTOR_POSITION_REAR_RIGHT;
        case 2U:
            return fsae_MotorPosition_MOTOR_POSITION_FRONT_LEFT;
        case 3U:
            return fsae_MotorPosition_MOTOR_POSITION_FRONT_RIGHT;
        default:
            return fsae_MotorPosition_MOTOR_POSITION_UNSPECIFIED;
    }
}

static uint8_t Publish_MapMotorOrderToThermalSensorIndex(uint8_t motor_index)
{
    /* Thermal sensors are exposed in a different order than the motor array. */
    switch (motor_index)
    {
        case 0U:
            return 2U;
        case 1U:
            return 3U;
        case 2U:
            return 0U;
        case 3U:
            return 1U;
        default:
            return 0xFFU;
    }
}

static void Publish_MapCanbMotorState(fsae_MotorState *motor_state, uint8_t motor_index)
{
    /* Fill one motor entry directly from the CANB cache slot. */
    if (motor_state == NULL || motor_index >= 4U)
    {
        return;
    }

    *motor_state = (fsae_MotorState)fsae_MotorState_init_zero;
    motor_state->position = Publish_MapCanbMotorPosition(motor_index);
    motor_state->rpm = (int32_t)g_CANB_LoopData.ECU.Motor_RPM[motor_index];
    motor_state->torque_nm = (int32_t)g_CANB_LoopData.ECU.Motor_Torque[motor_index];
    motor_state->power_w = (int32_t)(g_CANB_LoopData.ECU.Motor_Power[motor_index] * 1000.0f);
    motor_state->motor_error = (int32_t)g_CANB_LoopData.ECU.ERRO[motor_index];

    motor_state->motor_temp_dc = (int32_t)g_CANB_LoopData.ECU.MotorTempDc[motor_index];
    motor_state->inverter_temp_dc = (int32_t)g_CANB_LoopData.ECU.InverterTempDc[motor_index];
    motor_state->igbt_temp_dc = (int32_t)g_CANB_LoopData.ECU.IgbtTempDc[motor_index];
    motor_state->diagnostic_number = g_CANB_LoopData.ECU.DiagnosticNumber[motor_index];
    motor_state->has_logic_state = true;
    motor_state->logic_state = (uint32_t)g_CANB_LoopData.ECU.LogicState[motor_index];
}

static void Publish_MapFastTelemetry(fsae_FastTelemetry *fast_telemetry)
{
    /* Fast telemetry carries the compact vehicle summary used most often. */
    if (fast_telemetry == NULL)
    {
        return;
    }

    *fast_telemetry = (fsae_FastTelemetry)fsae_FastTelemetry_init_zero;
    fast_telemetry->hv_voltage_dv = (int32_t)((g_BatteryInfo.TotalVolt + 50U) / 100U);
    fast_telemetry->hv_current_ma = (int32_t)g_BatteryInfo.TotalCurrent;
    fast_telemetry->battery_temp_max_dc = (int32_t)g_BatteryInfo.MaxTemp * 10;
    fast_telemetry->driving_mode = Publish_MapDrivingMode(g_CANB_LoopData.ECU.driving_mode);
    fast_telemetry->speed_kmh = Publish_GetVehicleSpeedKmh();
}

static bool Publish_MapIvtTelemetry(fsae_IvtTelemetry *ivt_telemetry)
{
    /* Export the legacy IVT cache into its protobuf representation. */
    uint8_t valid_flags = g_CANB_LoopData.IVT.ValidFlags;

    if ((ivt_telemetry == NULL) || (valid_flags == 0U))
    {
        return false;
    }

    *ivt_telemetry = (fsae_IvtTelemetry)fsae_IvtTelemetry_init_zero;

    if ((valid_flags & CANB_IVT_VALID_CURRENT) != 0U)
    {
        ivt_telemetry->current_ma = g_CANB_LoopData.IVT.CurrentMa;
        ivt_telemetry->current_state = g_CANB_LoopData.IVT.CurrentState;
    }

    if ((valid_flags & CANB_IVT_VALID_U1) != 0U)
    {
        ivt_telemetry->voltage_u1_mv = g_CANB_LoopData.IVT.VoltageU1Mv;
        ivt_telemetry->voltage_u1_state = g_CANB_LoopData.IVT.VoltageU1State;
    }

    if ((valid_flags & CANB_IVT_VALID_POWER) != 0U)
    {
        ivt_telemetry->power_w = g_CANB_LoopData.IVT.PowerW;
    }
    else if ((valid_flags & (CANB_IVT_VALID_CURRENT | CANB_IVT_VALID_U1)) ==
             (CANB_IVT_VALID_CURRENT | CANB_IVT_VALID_U1))
    {
        ivt_telemetry->power_w =
            (int32_t)(((int64_t)g_CANB_LoopData.IVT.CurrentMa * (int64_t)g_CANB_LoopData.IVT.VoltageU1Mv) / 1000000LL);
    }

    if ((valid_flags & CANB_IVT_VALID_ENERGY) != 0U)
    {
        ivt_telemetry->energy_wh = g_CANB_LoopData.IVT.EnergyWh;
        ivt_telemetry->energy_state = g_CANB_LoopData.IVT.EnergyState;
    }

    return true;
}

static bool Publish_MapEnergyMeterTelemetry(fsae_EnergyMeterTelemetry *energy_meter)
{
    /* Present one stable energy-meter protobuf regardless of physical source. */
    uint8_t valid_flags;
    uint8_t source;

    if (energy_meter == NULL)
    {
        return false;
    }

    source = g_CANB_LoopData.EnergyMeter.AutoSource;
    if (source == CANB_ENERGY_METER_SOURCE_UNKNOWN)
    {
        return false;
    }

    *energy_meter = (fsae_EnergyMeterTelemetry)fsae_EnergyMeterTelemetry_init_zero;
    energy_meter->source = (uint32_t)source;

    if (source == CANB_ENERGY_METER_SOURCE_IVT)
    {
        /* IVT source reads directly from the private team meter cache. */
        valid_flags = g_CANB_LoopData.IVT.ValidFlags;
        if (valid_flags == 0U)
        {
            return false;
        }

        if ((valid_flags & CANB_IVT_VALID_CURRENT) != 0U)
        {
            energy_meter->current_ma = g_CANB_LoopData.IVT.CurrentMa;
            energy_meter->status = g_CANB_LoopData.IVT.CurrentState;
        }

        if ((valid_flags & CANB_IVT_VALID_U1) != 0U)
        {
            energy_meter->voltage_mv = g_CANB_LoopData.IVT.VoltageU1Mv;
        }

        if ((valid_flags & CANB_IVT_VALID_ENERGY) != 0U)
        {
            energy_meter->energy_wh = g_CANB_LoopData.IVT.EnergyWh;
        }

        if ((valid_flags & CANB_IVT_VALID_POWER) != 0U)
        {
            energy_meter->power_w = g_CANB_LoopData.IVT.PowerW;
        }
        else if ((valid_flags & (CANB_IVT_VALID_CURRENT | CANB_IVT_VALID_U1)) ==
                 (CANB_IVT_VALID_CURRENT | CANB_IVT_VALID_U1))
        {
            energy_meter->power_w =
                (int32_t)(((int64_t)g_CANB_LoopData.IVT.CurrentMa *
                           (int64_t)g_CANB_LoopData.IVT.VoltageU1Mv) / 1000000LL);
        }

        return true;
    }

    /* FS source reads from the generic energy meter cache populated by the
     * FS status/result frames.
     */
    valid_flags = g_CANB_LoopData.EnergyMeter.ValidFlags;
    if (valid_flags == 0U)
    {
        return false;
    }

    if ((valid_flags & CANB_ENERGY_VALID_CURRENT) != 0U)
    {
        energy_meter->current_ma = g_CANB_LoopData.EnergyMeter.CurrentMa;
    }

    if ((valid_flags & CANB_ENERGY_VALID_VOLTAGE) != 0U)
    {
        energy_meter->voltage_mv = g_CANB_LoopData.EnergyMeter.VoltageMv;
    }

    if ((valid_flags & CANB_ENERGY_VALID_ENERGY) != 0U)
    {
        energy_meter->energy_wh = g_CANB_LoopData.EnergyMeter.EnergyWh;
    }

    if ((valid_flags & CANB_ENERGY_VALID_POWER) != 0U)
    {
        energy_meter->power_w = g_CANB_LoopData.EnergyMeter.PowerW;
    }
    else if ((valid_flags & (CANB_ENERGY_VALID_CURRENT | CANB_ENERGY_VALID_VOLTAGE)) ==
             (CANB_ENERGY_VALID_CURRENT | CANB_ENERGY_VALID_VOLTAGE))
    {
        energy_meter->power_w =
            (int32_t)(((int64_t)g_CANB_LoopData.EnergyMeter.CurrentMa *
                       (int64_t)g_CANB_LoopData.EnergyMeter.VoltageMv) / 1000000LL);
    }

    if ((valid_flags & CANB_ENERGY_VALID_STATUS) != 0U)
    {
        energy_meter->status = g_CANB_LoopData.EnergyMeter.Status;
        energy_meter->msg_counter = (uint32_t)g_CANB_LoopData.EnergyMeter.MsgCounter;
    }

    return true;
}

static bool Publish_MapMotionTelemetry(fsae_MotionTelemetry *motion)
{
    /* Route-A debug path: always emit a fixed GPS speed so the uplink chain can
     * be validated independently from live IMU/GPS cache timing.
     */
    if (motion == NULL)
    {
        return false;
    }

    *motion = (fsae_MotionTelemetry)fsae_MotionTelemetry_init_zero;

    motion->gps_speed_kmh = 150U;

    return true;
}

static void Publish_MapCommonTelemetryFrame(fsae_TelemetryFrame *frame)
{
    /* Shared BMS fields are filled once and reused by summary and detail. */
    uint32_t now;

    if (frame == NULL)
    {
        return;
    }

    *frame = (fsae_TelemetryFrame)fsae_TelemetryFrame_init_zero;
    now = HAL_GetTick();

    frame->timestamp_ms = now;
    frame->apps_position = (float)g_CANB_LoopData.ECU.ThrottlePositionDeciPct / 10.0f;
    frame->brake_pressure = (float)g_CANB_LoopData.ECU.OilPressureMilliKpa / 1000.0f;
    frame->steering_angle = (float)g_CANB_LoopData.ECU.SteeringAngleDeciDeg / 10.0f;
    frame->hv_voltage = (float)g_BatteryInfo.TotalVolt / 1000.0f;
    frame->hv_current = (float)g_BatteryInfo.TotalCurrent / 1000.0f;
    frame->battery_temp_max = (float)g_BatteryInfo.MaxTemp;
    frame->fault_code = g_BatteryInfo.BatteryFaultCode;
    frame->motor_rpm = (int32_t)g_CANB_LoopData.ECU.Motor_RPM[0];
    frame->motor_temp = (float)g_CANB_LoopData.ECU.MotorTempDc[0] / 10.0f;
    frame->inverter_temp = (float)g_CANB_LoopData.ECU.InverterTempDc[0] / 10.0f;
    frame->ready_to_drive = 0U;
    frame->vcu_status = 0U;
    frame->battery_soc = (uint32_t)g_BatteryInfo.SOC;
    frame->max_cell_voltage = (uint32_t)g_BatteryInfo.MaxCellVolt;
    frame->min_cell_voltage = (uint32_t)g_BatteryInfo.MinCellVolt;
    frame->max_cell_voltage_no = (uint32_t)g_BatteryInfo.MaxCellVoltNo;
    frame->min_cell_voltage_no = (uint32_t)g_BatteryInfo.MinCellVoltNo;
    frame->max_temp = (int32_t)g_BatteryInfo.MaxTemp;
    frame->min_temp = (int32_t)g_BatteryInfo.MinTemp;
    frame->max_temp_no = (uint32_t)g_BatteryInfo.MaxTempNo;
    frame->min_temp_no = (uint32_t)g_BatteryInfo.MinTempNo;
    frame->battery_fault_code = g_BatteryInfo.BatteryFaultCode;
    frame->has_header = true;
    frame->header.timestamp_ms = now;
    frame->has_fast_telemetry = true;
    Publish_MapFastTelemetry(&frame->fast_telemetry);
    frame->has_vehicle_state = true;
    Publish_MapCanbVehicleState(&frame->vehicle_state);

    /* These nested messages are emitted only when their backing caches carry
     * at least one valid sample.
     */
    if (Publish_MapIvtTelemetry(&frame->ivt_telemetry))
    {
        frame->has_ivt_telemetry = true;
    }

    if (Publish_MapEnergyMeterTelemetry(&frame->energy_meter))
    {
        frame->has_energy_meter = true;
    }

    if (Publish_MapMotionTelemetry(&frame->motion))
    {
        frame->has_motion = true;
    }
}

static void Publish_MapBmsSummaryFrame(fsae_TelemetryFrame *frame)
{
    if (frame == NULL)
    {
        return;
    }

    Publish_MapCommonTelemetryFrame(frame);
}

static void Publish_MapBmsDetailFrame(fsae_TelemetryFrame *frame)
{
    /* Detail mode extends the common frame with cell-level and module data. */
    if (frame == NULL)
    {
        return;
    }

    Publish_MapBmsSummaryFrame(frame);
    frame->modules_count = 6U;

    for (uint8_t module_index = 0U; module_index < frame->modules_count; module_index++)
    {
        Publish_MapBatteryModule(&frame->modules[module_index], module_index);
    }
}

static void Publish_MapBatteryModule(fsae_BatteryModule *module, uint8_t module_index)
{
    /* Copy one battery module from the shared cache into protobuf fields. */
    uint32_t *cell_fields[PUBLISH_BMS_DETAIL_CELL_COUNT];
    int32_t *temp_fields[8U];

    if (module == NULL || module_index >= 6U)
    {
        return;
    }

    *module = (fsae_BatteryModule)fsae_BatteryModule_init_zero;
    module->module_id = (uint32_t)(module_index + 1U);

    cell_fields[0] = &module->v01;
    cell_fields[1] = &module->v02;
    cell_fields[2] = &module->v03;
    cell_fields[3] = &module->v04;
    cell_fields[4] = &module->v05;
    cell_fields[5] = &module->v06;
    cell_fields[6] = &module->v07;
    cell_fields[7] = &module->v08;
    cell_fields[8] = &module->v09;
    cell_fields[9] = &module->v10;
    cell_fields[10] = &module->v11;
    cell_fields[11] = &module->v12;
    cell_fields[12] = &module->v13;
    cell_fields[13] = &module->v14;
    cell_fields[14] = &module->v15;
    cell_fields[15] = &module->v16;
    cell_fields[16] = &module->v17;
    cell_fields[17] = &module->v18;
    cell_fields[18] = &module->v19;
    cell_fields[19] = &module->v20;
    cell_fields[20] = &module->v21;
    cell_fields[21] = &module->v22;
    cell_fields[22] = &module->v23;

    for (uint8_t cell_index = 0U; cell_index < PUBLISH_BMS_DETAIL_CELL_COUNT; cell_index++)
    {
        *cell_fields[cell_index] = (uint32_t)g_BatteryInfo.CellVolt[module_index][cell_index];
    }

    temp_fields[0] = &module->t1;
    temp_fields[1] = &module->t2;
    temp_fields[2] = &module->t3;
    temp_fields[3] = &module->t4;
    temp_fields[4] = &module->t5;
    temp_fields[5] = &module->t6;
    temp_fields[6] = &module->t7;
    temp_fields[7] = &module->t8;

    for (uint8_t temp_index = 0U; temp_index < 8U; temp_index++)
    {
        *temp_fields[temp_index] = (int32_t)g_BatteryInfo.ModTemp[module_index][temp_index];
    }
}

static void Publish_MapThermalSummary(fsae_ThermalSummary *thermal_summary)
{
    /* Thermal summary exposes the MLX90640 regions in a compact protobuf form. */
    if (thermal_summary == NULL)
    {
        return;
    }

    *thermal_summary = (fsae_ThermalSummary)fsae_ThermalSummary_init_zero;
    thermal_summary->sensors_count = MLX90640_SENSOR_COUNT;

    for (uint8_t sensor_index = 0U; sensor_index < thermal_summary->sensors_count; sensor_index++)
    {
        fsae_ThermalSensorSummary *sensor = &thermal_summary->sensors[sensor_index];
        uint8_t source_sensor_index = Publish_MapMotorOrderToThermalSensorIndex(sensor_index);
        int32_t temp_sum = 0;
        int32_t temp_min = 0;
        int32_t temp_max = 0;

        sensor->position = Publish_MapCanbMotorPosition(sensor_index);

        if ((source_sensor_index >= MLX90640_SENSOR_COUNT) ||
            ((g_MLX90640_Frame.ValidMask & (uint8_t)(1U << source_sensor_index)) == 0U))
        {
            sensor->chunks_count = 0U;
            continue;
        }

        sensor->chunks_count = MLX90640_REGION_COUNT;
        temp_min = g_MLX90640_Frame.RegionTemp[source_sensor_index][0];
        temp_max = g_MLX90640_Frame.RegionTemp[source_sensor_index][0];

        for (uint8_t region_index = 0U; region_index < MLX90640_REGION_COUNT; region_index++)
        {
            int32_t region_temp = g_MLX90640_Frame.RegionTemp[source_sensor_index][region_index];
            fsae_ThermalRawChunk *chunk = &sensor->chunks[region_index];

            if (region_temp < temp_min)
            {
                temp_min = region_temp;
            }

            if (region_temp > temp_max)
            {
                temp_max = region_temp;
            }

            temp_sum += region_temp;

            *chunk = (fsae_ThermalRawChunk)fsae_ThermalRawChunk_init_zero;
            chunk->position = sensor->position;
            chunk->frame_id = g_MLX90640_Frame.FrameCounter[source_sensor_index];
            chunk->chunk_index = region_index;
            chunk->chunk_count = MLX90640_REGION_COUNT;
            chunk->pixel_temp_centi_c = region_temp;
        }

        sensor->min_temp_centi_c = temp_min;
        sensor->max_temp_centi_c = temp_max;
        sensor->avg_temp_centi_c = temp_sum / (int32_t)MLX90640_REGION_COUNT;
    }
}

static bool Publish_EncodeTopicPayload(PublishTopic_t topic, uint8_t *payload_buffer, size_t payload_capacity, size_t *payload_size)
{
    /* Encode only the selected topic payload; the frame header is added later. */
    pb_ostream_t stream;
    bool encode_status = false;

    if (payload_buffer == NULL || payload_size == NULL)
    {
        return false;
    }

    stream = pb_ostream_from_buffer(payload_buffer, payload_capacity);

    switch (topic)
    {
        case PUBLISH_TOPIC_FAST_TELEMETRY:
        {
            g_publishFastTelemetry = (fsae_FastTelemetry)fsae_FastTelemetry_init_zero;
            Publish_MapFastTelemetry(&g_publishFastTelemetry);
            encode_status = pb_encode(&stream, fsae_FastTelemetry_fields, &g_publishFastTelemetry);
            break;
        }

        case PUBLISH_TOPIC_BMS_SUMMARY:
        {
            g_publishTelemetryFrame = (fsae_TelemetryFrame)fsae_TelemetryFrame_init_zero;
            Publish_MapBmsSummaryFrame(&g_publishTelemetryFrame);
            encode_status = pb_encode(&stream, fsae_TelemetryFrame_fields, &g_publishTelemetryFrame);
            break;
        }

        case PUBLISH_TOPIC_VEHICLE_STATE:
        {
            g_publishTelemetryFrame = (fsae_TelemetryFrame)fsae_TelemetryFrame_init_zero;
            Publish_MapCommonTelemetryFrame(&g_publishTelemetryFrame);
            encode_status = pb_encode(&stream, fsae_TelemetryFrame_fields, &g_publishTelemetryFrame);
            break;
        }

        case PUBLISH_TOPIC_BMS_DETAIL:
        {
            g_publishTelemetryFrame = (fsae_TelemetryFrame)fsae_TelemetryFrame_init_zero;
            Publish_MapBmsDetailFrame(&g_publishTelemetryFrame);
            encode_status = pb_encode(&stream, fsae_TelemetryFrame_fields, &g_publishTelemetryFrame);
            break;
        }

        case PUBLISH_TOPIC_THERMAL_SUMMARY:
        {
            g_publishThermalSummary = (fsae_ThermalSummary)fsae_ThermalSummary_init_zero;
            Publish_MapThermalSummary(&g_publishThermalSummary);
            encode_status = pb_encode(&stream, fsae_ThermalSummary_fields, &g_publishThermalSummary);
            break;
        }

        default:
            encode_status = false;
            break;
    }

    if (!encode_status)
    {
        return false;
    }

    *payload_size = stream.bytes_written;
    return true;
}
static const char *Publish_GetTopicName(PublishTopic_t topic)
{
    /* Keep the topic names as plain text for the downstream TCP side. */
    switch (topic)
    {
        case PUBLISH_TOPIC_FAST_TELEMETRY:
            return "fast_telemetry";
        case PUBLISH_TOPIC_BMS_SUMMARY:
            return "bms_summary";
        case PUBLISH_TOPIC_VEHICLE_STATE:
            return "vehicle_state";
        case PUBLISH_TOPIC_BMS_DETAIL:
            return "bms_detail";
        case PUBLISH_TOPIC_THERMAL_SUMMARY:
            return "thermal_summary";
        default:
            return NULL;
    }
}

static bool Publish_IsTopicValid(PublishTopic_t topic)
{
    /* Reject unsupported topic IDs before they hit the queue. */
    return ((uint32_t)topic < (uint32_t)PUBLISH_TOPIC_COUNT);
}
