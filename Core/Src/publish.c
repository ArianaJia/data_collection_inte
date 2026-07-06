#include "publish.h"

#include <stdbool.h>
#include <string.h>

#include "battery.h"
#include "pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "usart.h"

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

extern osMessageQueueId_t myQueue04Handle;

static fsae_DrivingMode Publish_MapDrivingMode(uint8_t raw_mode);
static fsae_ImdState Publish_MapImdState(uint8_t raw_state);
static fsae_WorkState Publish_MapWorkState(uint8_t raw_state);
static fsae_MotorPosition Publish_MapCanbMotorPosition(uint8_t motor_index);
static void Publish_MapCanbMotorState(fsae_MotorState *motor_state, uint8_t motor_index);
static void Publish_MapFastTelemetry(fsae_FastTelemetry *fast_telemetry);
static void Publish_MapBmsSummary(fsae_BmsSummary *bms_summary);
static void Publish_MapBmsDetail(fsae_BmsDetail *bms_detail);
static void Publish_MapThermalSummary(fsae_ThermalSummary *thermal_summary);
static bool Publish_EncodeTopicPayload(PublishTopic_t topic, uint8_t *payload_buffer, size_t payload_capacity, size_t *payload_size);
static bool Publish_SendFrame(PublishTopic_t topic, const uint8_t *payload, uint16_t payload_size);
static const char *Publish_GetTopicName(PublishTopic_t topic);

void Publish_Init(void)
{
}

void Publish_Process(void)
{
    static bool first_run = true;
    static uint32_t last_fast_tick = 0U;
    static uint32_t last_medium_tick = 0U;
    static uint32_t last_slow_tick = 0U;
    uint32_t now = HAL_GetTick();

    if (first_run)
    {
        last_fast_tick = now;
        last_medium_tick = now;
        last_slow_tick = now;
        first_run = false;

        (void)Publish_QueueTopic(PUBLISH_TOPIC_FAST_TELEMETRY);
        (void)Publish_QueueTopic(PUBLISH_TOPIC_BMS_SUMMARY);
        (void)Publish_QueueTopic(PUBLISH_TOPIC_VEHICLE_STATE);
        (void)Publish_QueueTopic(PUBLISH_TOPIC_BMS_DETAIL);
        (void)Publish_QueueTopic(PUBLISH_TOPIC_THERMAL_SUMMARY);
        return;
    }

    if ((now - last_fast_tick) >= PUBLISH_FAST_PERIOD_MS)
    {
        last_fast_tick = now;
        (void)Publish_QueueTopic(PUBLISH_TOPIC_FAST_TELEMETRY);
    }

    if ((now - last_medium_tick) >= PUBLISH_MEDIUM_PERIOD_MS)
    {
        last_medium_tick = now;
        (void)Publish_QueueTopic(PUBLISH_TOPIC_BMS_SUMMARY);
        (void)Publish_QueueTopic(PUBLISH_TOPIC_VEHICLE_STATE);
    }

    if ((now - last_slow_tick) >= PUBLISH_SLOW_PERIOD_MS)
    {
        last_slow_tick = now;
        (void)Publish_QueueTopic(PUBLISH_TOPIC_BMS_DETAIL);
        (void)Publish_QueueTopic(PUBLISH_TOPIC_THERMAL_SUMMARY);
    }
}

osStatus_t Publish_QueueTopic(PublishTopic_t topic)
{
    PublishQueueItem_t item;

    if (myQueue04Handle == NULL)
    {
        return osErrorResource;
    }

    item.topic = topic;
    return osMessageQueuePut(myQueue04Handle, &item, 0U, 0U);
}

void Publish_TxTaskStep(uint32_t timeout_ms)
{
    PublishQueueItem_t item;
    uint8_t payload_buffer[PUBLISH_MAX_PAYLOAD_SIZE];
    size_t payload_size = 0U;

    if (myQueue04Handle == NULL)
    {
        return;
    }

    if (osMessageQueueGet(myQueue04Handle, &item, NULL, timeout_ms) != osOK)
    {
        return;
    }

    if (!Publish_EncodeTopicPayload(item.topic, payload_buffer, sizeof(payload_buffer), &payload_size))
    {
        return;
    }

    (void)Publish_SendFrame(item.topic, payload_buffer, (uint16_t)payload_size);
}

void Publish_MapCanbVehicleState(fsae_VehicleState *vehicle_state)
{
    if (vehicle_state == NULL)
    {
        return;
    }

    *vehicle_state = (fsae_VehicleState)fsae_VehicleState_init_zero;
    vehicle_state->speed_kmh = g_CANB_LoopData.ECU.Vehicle_Speed;
    vehicle_state->driving_mode = Publish_MapDrivingMode(g_CANB_LoopData.ECU.driving_mode);
    vehicle_state->motors_count = 4U;

    /* Current CANB loop data does not provide throttle/brake/VCU/RTD fields yet. */
    vehicle_state->throttle_position = 0U;
    vehicle_state->brake_position = 0U;
    vehicle_state->ready_to_drive = false;
    vehicle_state->vcu_status = fsae_VcuStatus_VCU_STATUS_UNSPECIFIED;

    for (uint8_t i = 0U; i < vehicle_state->motors_count; i++)
    {
        Publish_MapCanbMotorState(&vehicle_state->motors[i], i);
    }
}

static fsae_DrivingMode Publish_MapDrivingMode(uint8_t raw_mode)
{
    switch (raw_mode)
    {
        case 1U:
            return fsae_DrivingMode_DRIVING_MODE_STANDBY;
        case 2U:
            return fsae_DrivingMode_DRIVING_MODE_READY;
        case 3U:
            return fsae_DrivingMode_DRIVING_MODE_DRIVE;
        case 4U:
            return fsae_DrivingMode_DRIVING_MODE_FAULT;
        default:
            return fsae_DrivingMode_DRIVING_MODE_UNSPECIFIED;
    }
}

static fsae_ImdState Publish_MapImdState(uint8_t raw_state)
{
    switch (raw_state)
    {
        case 1U:
            return fsae_ImdState_IMD_STATE_NORMAL;
        case 2U:
            return fsae_ImdState_IMD_STATE_WARNING;
        case 3U:
            return fsae_ImdState_IMD_STATE_FAULT;
        default:
            return fsae_ImdState_IMD_STATE_UNSPECIFIED;
    }
}

static fsae_WorkState Publish_MapWorkState(uint8_t raw_state)
{
    switch (raw_state)
    {
        case 1U:
            return fsae_WorkState_WORK_STATE_OFF;
        case 2U:
            return fsae_WorkState_WORK_STATE_PRECHARGE;
        case 3U:
            return fsae_WorkState_WORK_STATE_ON;
        case 4U:
            return fsae_WorkState_WORK_STATE_FAULT;
        default:
            return fsae_WorkState_WORK_STATE_UNSPECIFIED;
    }
}

static fsae_MotorPosition Publish_MapCanbMotorPosition(uint8_t motor_index)
{
    switch (motor_index)
    {
        case 0U:
            return fsae_MotorPosition_MOTOR_POSITION_REAR_LEFT;
        case 1U:
            return fsae_MotorPosition_MOTOR_POSITION_FRONT_LEFT;
        case 2U:
            return fsae_MotorPosition_MOTOR_POSITION_REAR_RIGHT;
        case 3U:
            return fsae_MotorPosition_MOTOR_POSITION_FRONT_RIGHT;
        default:
            return fsae_MotorPosition_MOTOR_POSITION_UNSPECIFIED;
    }
}

static void Publish_MapCanbMotorState(fsae_MotorState *motor_state, uint8_t motor_index)
{
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

    /* Current CANB loop data has no motor/inverter temperature fields yet. */
    motor_state->motor_temp_dc = 0;
    motor_state->inverter_temp_dc = 0;
}

static void Publish_MapFastTelemetry(fsae_FastTelemetry *fast_telemetry)
{
    if (fast_telemetry == NULL)
    {
        return;
    }

    *fast_telemetry = (fsae_FastTelemetry)fsae_FastTelemetry_init_zero;
    fast_telemetry->hv_voltage_dv = (int32_t)((g_BatteryInfo.TotalVolt + 50U) / 100U);
    fast_telemetry->hv_current_ma = (int32_t)g_BatteryInfo.TotalCurrent;
    fast_telemetry->battery_temp_max_c = (int32_t)g_BatteryInfo.MaxTemp;
    fast_telemetry->driving_mode = Publish_MapDrivingMode(g_CANB_LoopData.ECU.driving_mode);
    fast_telemetry->speed_kmh = (uint32_t)g_CANB_LoopData.ECU.Vehicle_Speed;
}

static void Publish_MapBmsSummary(fsae_BmsSummary *bms_summary)
{
    if (bms_summary == NULL)
    {
        return;
    }

    *bms_summary = (fsae_BmsSummary)fsae_BmsSummary_init_zero;
    bms_summary->total_voltage = (uint32_t)((g_BatteryInfo.TotalVolt + 50U) / 100U);
    bms_summary->total_current = (int32_t)g_BatteryInfo.TotalCurrent;
    bms_summary->soc_pct = (uint32_t)g_BatteryInfo.SOC;
    bms_summary->max_temp_c = (int32_t)g_BatteryInfo.MaxTemp;
    bms_summary->min_temp_c = (int32_t)g_BatteryInfo.MinTemp;
    bms_summary->max_cell_mv = (uint32_t)g_BatteryInfo.MaxCellVolt;
    bms_summary->min_cell_mv = (uint32_t)g_BatteryInfo.MinCellVolt;
    bms_summary->imd_state = Publish_MapImdState(g_BatteryInfo.IMD_State);
    bms_summary->work_state = Publish_MapWorkState(g_BatteryInfo.Work_State);
}

static void Publish_MapBmsDetail(fsae_BmsDetail *bms_detail)
{
    if (bms_detail == NULL)
    {
        return;
    }

    *bms_detail = (fsae_BmsDetail)fsae_BmsDetail_init_zero;
    bms_detail->modules_count = 6U;

    for (uint8_t module_index = 0U; module_index < bms_detail->modules_count; module_index++)
    {
        fsae_BatteryModule *module = &bms_detail->modules[module_index];
        module->module_id = (uint32_t)(module_index + 1U);
        module->cell_mv_count = PUBLISH_BMS_DETAIL_CELL_COUNT;
        module->temp_dc_count = 8U;

        /*
         * Local battery struct keeps 24 cells, while the current protobuf schema
         * reserves 23 values. Keep the first 23 entries to match the schema.
         */
        for (uint8_t cell_index = 0U; cell_index < module->cell_mv_count; cell_index++)
        {
            module->cell_mv[cell_index] = (uint32_t)g_BatteryInfo.CellVolt[module_index][cell_index];
        }

        for (uint8_t temp_index = 0U; temp_index < module->temp_dc_count; temp_index++)
        {
            module->temp_dc[temp_index] = (int32_t)g_BatteryInfo.ModTemp[module_index][temp_index] * 10;
        }
    }
}

static void Publish_MapThermalSummary(fsae_ThermalSummary *thermal_summary)
{
    if (thermal_summary == NULL)
    {
        return;
    }

    *thermal_summary = (fsae_ThermalSummary)fsae_ThermalSummary_init_zero;
    thermal_summary->sensors_count = MLX90640_SENSOR_COUNT;

    for (uint8_t sensor_index = 0U; sensor_index < thermal_summary->sensors_count; sensor_index++)
    {
        fsae_ThermalSensorSummary *sensor = &thermal_summary->sensors[sensor_index];
        int32_t temp_sum = 0;
        int32_t temp_min = 0;
        int32_t temp_max = 0;

        /* Assume sensor order follows the same wheel order used by CANB motor arrays. */
        sensor->position = Publish_MapCanbMotorPosition(sensor_index);

        if ((g_MLX90640_Frame.ValidMask & (uint8_t)(1U << sensor_index)) == 0U)
        {
            sensor->chunks_count = 0U;
            continue;
        }

        sensor->chunks_count = MLX90640_REGION_COUNT;
        temp_min = g_MLX90640_Frame.RegionTemp[sensor_index][0];
        temp_max = g_MLX90640_Frame.RegionTemp[sensor_index][0];

        for (uint8_t region_index = 0U; region_index < MLX90640_REGION_COUNT; region_index++)
        {
            int32_t region_temp = g_MLX90640_Frame.RegionTemp[sensor_index][region_index];
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
            chunk->frame_id = g_MLX90640_Frame.FrameCounter[sensor_index];
            chunk->chunk_index = region_index;
            chunk->chunk_count = MLX90640_REGION_COUNT;
            chunk->pixel_temp_dc = region_temp;
        }

        sensor->min_temp_dc = temp_min;
        sensor->max_temp_dc = temp_max;
        sensor->avg_temp_dc = temp_sum / (int32_t)MLX90640_REGION_COUNT;
    }
}

static bool Publish_EncodeTopicPayload(PublishTopic_t topic, uint8_t *payload_buffer, size_t payload_capacity, size_t *payload_size)
{
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
            fsae_FastTelemetry fast_telemetry = fsae_FastTelemetry_init_zero;
            Publish_MapFastTelemetry(&fast_telemetry);
            encode_status = pb_encode(&stream, fsae_FastTelemetry_fields, &fast_telemetry);
            break;
        }

        case PUBLISH_TOPIC_BMS_SUMMARY:
        {
            fsae_BmsSummary bms_summary = fsae_BmsSummary_init_zero;
            Publish_MapBmsSummary(&bms_summary);
            encode_status = pb_encode(&stream, fsae_BmsSummary_fields, &bms_summary);
            break;
        }

        case PUBLISH_TOPIC_VEHICLE_STATE:
        {
            fsae_VehicleState vehicle_state = fsae_VehicleState_init_zero;
            Publish_MapCanbVehicleState(&vehicle_state);
            encode_status = pb_encode(&stream, fsae_VehicleState_fields, &vehicle_state);
            break;
        }

        case PUBLISH_TOPIC_BMS_DETAIL:
        {
            fsae_BmsDetail bms_detail = fsae_BmsDetail_init_zero;
            Publish_MapBmsDetail(&bms_detail);
            encode_status = pb_encode(&stream, fsae_BmsDetail_fields, &bms_detail);
            break;
        }

        case PUBLISH_TOPIC_THERMAL_SUMMARY:
        {
            fsae_ThermalSummary thermal_summary = fsae_ThermalSummary_init_zero;
            Publish_MapThermalSummary(&thermal_summary);
            encode_status = pb_encode(&stream, fsae_ThermalSummary_fields, &thermal_summary);
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

static bool Publish_SendFrame(PublishTopic_t topic, const uint8_t *payload, uint16_t payload_size)
{
    uint8_t frame_buffer[PUBLISH_MAX_FRAME_SIZE];
    const char *topic_name = Publish_GetTopicName(topic);
    size_t topic_len;
    size_t frame_size = 0U;

    if (topic_name == NULL || payload == NULL)
    {
        return false;
    }

    topic_len = strlen(topic_name);
    if (topic_len > PUBLISH_MAX_TOPIC_LEN)
    {
        return false;
    }

    if ((PUBLISH_FRAME_HEADER_SIZE + topic_len + payload_size) > sizeof(frame_buffer))
    {
        return false;
    }

    frame_buffer[frame_size++] = PUBLISH_FRAME_MAGIC_0;
    frame_buffer[frame_size++] = PUBLISH_FRAME_MAGIC_1;
    frame_buffer[frame_size++] = (uint8_t)topic_len;
    frame_buffer[frame_size++] = (uint8_t)(payload_size & 0xFFU);
    frame_buffer[frame_size++] = (uint8_t)((payload_size >> 8) & 0xFFU);

    memcpy(&frame_buffer[frame_size], topic_name, topic_len);
    frame_size += topic_len;

    memcpy(&frame_buffer[frame_size], payload, payload_size);
    frame_size += payload_size;

    return (HAL_UART_Transmit(&huart1, frame_buffer, (uint16_t)frame_size, 1000U) == HAL_OK);
}

static const char *Publish_GetTopicName(PublishTopic_t topic)
{
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
