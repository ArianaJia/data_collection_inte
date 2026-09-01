/* CAN decoding converts raw bus payloads into the shared telemetry caches.
 * The decoder keeps source arbitration and byte-order handling here so the
 * rest of the application can work with normalized values.
 */
#include "can_decode.h"

#include "main.h"
#include "telemetry_data.h"

#define CAN_DECODE_MOTOR_RL 0U
#define CAN_DECODE_MOTOR_RR 1U
#define CAN_DECODE_MOTOR_FL 2U
#define CAN_DECODE_MOTOR_FR 3U

#define CAN_DECODE_LEN_2 2U
#define CAN_DECODE_LEN_4 4U
#define CAN_DECODE_LEN_5 5U
#define CAN_DECODE_LEN_6 6U
#define CAN_DECODE_LEN_7 7U
#define CAN_DECODE_LEN_8 8U

static uint16_t CAN_DecodeReadLe16(const uint8_t *data);
static int16_t CAN_DecodeReadLe16Signed(const uint8_t *data);
static uint32_t CAN_DecodeReadLe32(const uint8_t *data);
static int32_t CAN_DecodeReadLe32Signed(const uint8_t *data);
static uint16_t CAN_DecodeReadBe16(const uint8_t *data);
static uint32_t CAN_DecodeReadBe32(const uint8_t *data);
static int32_t CAN_DecodeReadBe32Signed(const uint8_t *data);
static int8_t CAN_DecodeSignExtend4(uint8_t raw);
static void CAN_DecodeStoreWheelLe16(int16_t values[4], const uint8_t *data);
static uint8_t CAN_DecodeCompetitionResult(const CAN_Msg_Queue_t *recv_data);
static void CAN_DecodeBatteryBoxStdMessage(const CAN_Msg_Queue_t *recv_data);
static void CAN_DecodeBatteryBoxExtMessage(const CAN_Msg_Queue_t *recv_data);
static uint8_t CAN_DecodeControllerMotorIndex(uint32_t msg_id, uint8_t *motor_index);

static uint16_t CAN_DecodeReadLe16(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int16_t CAN_DecodeReadLe16Signed(const uint8_t *data)
{
  return (int16_t)CAN_DecodeReadLe16(data);
}

static uint32_t CAN_DecodeReadLe32(const uint8_t *data)
{
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static int32_t CAN_DecodeReadLe32Signed(const uint8_t *data)
{
  return (int32_t)CAN_DecodeReadLe32(data);
}

static uint16_t CAN_DecodeReadBe16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static uint32_t CAN_DecodeReadBe32(const uint8_t *data)
{
  return ((uint32_t)data[0] << 24) |
         ((uint32_t)data[1] << 16) |
         ((uint32_t)data[2] << 8) |
         ((uint32_t)data[3]);
}

static int32_t CAN_DecodeReadBe32Signed(const uint8_t *data)
{
  return (int32_t)CAN_DecodeReadBe32(data);
}

static int8_t CAN_DecodeSignExtend4(uint8_t raw)
{
  raw &= 0x0FU;
  return (raw & 0x08U) ? (int8_t)(raw | 0xF0U) : (int8_t)raw;
}

static void CAN_DecodeStoreWheelLe16(int16_t values[4], const uint8_t *data)
{
  values[CAN_DECODE_MOTOR_RL] = CAN_DecodeReadLe16Signed(&data[0]);
  values[CAN_DECODE_MOTOR_RR] = CAN_DecodeReadLe16Signed(&data[2]);
  values[CAN_DECODE_MOTOR_FL] = CAN_DecodeReadLe16Signed(&data[4]);
  values[CAN_DECODE_MOTOR_FR] = CAN_DecodeReadLe16Signed(&data[6]);
}

/* Sources are identified by physical bus and ID, never numeric plausibility. */
static void CAN_DecodeBatteryIvt(const CAN_Msg_Queue_t *frame)
{
  Battery_IvtData *ivt = &g_BatteryInfo.IVT;
  uint8_t mux = (uint8_t)(frame->msg_id - CAN_ID_BATTERY_IVT_CURRENT);
  uint8_t bit;
  int32_t value;
  if ((frame->dlc != 6U) || (mux >= 8U) || (frame->msg_data[0] != mux)) return;
  bit = (uint8_t)(1U << mux);
  value = CAN_DecodeReadLe32Signed(&frame->msg_data[2]);
  ivt->State[mux] = frame->msg_data[1] >> 4;
  ivt->Counter[mux] = frame->msg_data[1] & 0x0FU;
  ivt->LastUpdateMs[mux] = HAL_GetTick();
  ivt->ReceivedFlags |= bit;
  ivt->ValidFlags &= (uint8_t)~bit;
  if ((ivt->State[mux] & 0x0EU) == 0U) ivt->ValidFlags |= bit;
  switch (mux)
  {
    case 0U: ivt->CurrentMa = value; ivt->CurrentState = ivt->State[mux]; break;
    case 1U: ivt->VoltageU1Mv = value; ivt->VoltageU1State = ivt->State[mux]; break;
    case 2U: ivt->VoltageU2Mv = value; ivt->VoltageU2State = ivt->State[mux]; break;
    case 3U: ivt->VoltageU3Mv = value; break;
    case 4U: ivt->TemperatureDc = value; break;
    case 5U: ivt->PowerW = value; ivt->PowerState = ivt->State[mux]; break;
    case 6U: ivt->ChargeAs = value; break;
    case 7U: ivt->EnergyWh = value; ivt->EnergyState = ivt->State[mux]; break;
    default: break;
  }
}

static uint8_t CAN_DecodeCompetitionResult(const CAN_Msg_Queue_t *frame)
{
  uint8_t mux, index, bit, state;
  int32_t value;
  if ((frame == NULL) || (frame->dlc != 6U)) return 0U;
  switch (frame->msg_id)
  {
    case CAN_ID_COMPETITION_CURRENT: mux = 0U; index = 0U; break;
    case CAN_ID_COMPETITION_U1: mux = 1U; index = 1U; break;
    case CAN_ID_COMPETITION_POWER: mux = 5U; index = 2U; break;
    case CAN_ID_COMPETITION_WH: mux = 7U; index = 3U; break;
    default: return 0U;
  }
  if (frame->msg_data[0] != mux) return 0U;
  state = frame->msg_data[1] >> 4;
  bit = (uint8_t)(1U << index);
  value = CAN_DecodeReadBe32Signed(&frame->msg_data[2]);
  g_CANB_LoopData.EnergyMeter.ResultCounter[index] = frame->msg_data[1] & 0x0FU;
  if (g_CANB_LoopData.EnergyMeter.AutoSource != CANB_ENERGY_METER_SOURCE_FS)
  {
    g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_COMPETITION_IVT;
    g_CANB_LoopData.EnergyMeter.MsgCounter = frame->msg_data[1] & 0x0FU;
  }
  g_CANB_LoopData.EnergyMeter.LastUpdateMs[index] = HAL_GetTick();
  g_CANB_LoopData.EnergyMeter.ReceivedFlags |= bit;
  g_CANB_LoopData.EnergyMeter.ValidFlags &= (uint8_t)~bit;
  if ((state & 0x0EU) == 0U) g_CANB_LoopData.EnergyMeter.ValidFlags |= bit;
  switch (index)
  {
    case 0U: g_CANB_LoopData.EnergyMeter.CurrentMa = value; g_CANB_LoopData.EnergyMeter.CurrentState = state; break;
    case 1U: g_CANB_LoopData.EnergyMeter.VoltageMv = value; g_CANB_LoopData.EnergyMeter.VoltageState = state; break;
    case 2U: g_CANB_LoopData.EnergyMeter.PowerW = value; g_CANB_LoopData.EnergyMeter.PowerState = state; break;
    case 3U: g_CANB_LoopData.EnergyMeter.EnergyWh = value; g_CANB_LoopData.EnergyMeter.EnergyState = state; break;
    default: break;
  }
  return 1U;
}

static uint8_t CAN_DecodeControllerMotorIndex(uint32_t msg_id, uint8_t *motor_index)
{
  if (motor_index == NULL)
  {
    return 0U;
  }

  switch (msg_id)
  {
    case CAN_ID_CANA_FR_STATUS_SPEED_TORQUE_CURRENT:
    case CAN_ID_CANA_FR_DIAGNOSTIC_INFO:
    case CAN_ID_CANA_FR_MOTOR_INVERTER_IGBT_TEMP_DCBUS:
    case CAN_ID_CANA_FR_TORQUE_LIMIT_POWER:
      *motor_index = CAN_DECODE_MOTOR_FR;
      return 1U;

    case CAN_ID_CANA_FL_STATUS_SPEED_TORQUE_CURRENT:
    case CAN_ID_CANA_FL_DIAGNOSTIC_INFO:
    case CAN_ID_CANA_FL_MOTOR_INVERTER_IGBT_TEMP_DCBUS:
    case CAN_ID_CANA_FL_TORQUE_LIMIT_POWER:
      *motor_index = CAN_DECODE_MOTOR_FL;
      return 1U;

    case CAN_ID_CANA_RR_STATUS_SPEED_TORQUE_CURRENT:
    case CAN_ID_CANA_RR_DIAGNOSTIC_INFO:
    case CAN_ID_CANA_RR_MOTOR_INVERTER_IGBT_TEMP_DCBUS:
    case CAN_ID_CANA_RR_TORQUE_LIMIT_POWER:
      *motor_index = CAN_DECODE_MOTOR_RR;
      return 1U;

    case CAN_ID_CANA_RL_STATUS_SPEED_TORQUE_CURRENT:
    case CAN_ID_CANA_RL_DIAGNOSTIC_INFO:
    case CAN_ID_CANA_RL_MOTOR_INVERTER_IGBT_TEMP_DCBUS:
    case CAN_ID_CANA_RL_TORQUE_LIMIT_POWER:
      *motor_index = CAN_DECODE_MOTOR_RL;
      return 1U;

    default:
      return 0U;
  }
}

void CAN_DecodeVehicleCanbMessage(const CAN_Msg_Queue_t *recv_data)
{
  /* Parse vehicle-side CANB frames and update the shared vehicle telemetry. */
  if (recv_data == NULL || recv_data->can_channel != CAN_CHANNEL_VEHICLE_CANB || recv_data->is_ext_id != 0U || recv_data->dlc > 8U)
  {
    return;
  }

  switch (recv_data->msg_id)
  {
    case CAN_ID_CANB_GPS_SPEED:
      if (recv_data->dlc == CAN_DECODE_LEN_2)
      {
        uint16_t speed = CAN_DecodeReadLe16(recv_data->msg_data);
        if (speed > 3000U) { g_CANB_LoopData.IMU.ValidFlags &= (uint8_t)~CANB_MOTION_VALID_GPS; break; }
        g_CANB_LoopData.IMU.GpsSpeedDeciKmh = speed;
        g_CANB_LoopData.IMU.GpsSpeedKmh = (uint16_t)((speed + 5U) / 10U);
        g_CANB_LoopData.IMU.ValidFlags |= CANB_MOTION_VALID_GPS;
      }
      break;

    case CAN_ID_CANB_DRIVER_INPUTS_OIL_PRESSURE:
      if (recv_data->dlc == CAN_DECODE_LEN_6)
      {
        g_CANB_LoopData.ECU.SteeringAngleDeciDeg = CAN_DecodeReadLe16Signed(&recv_data->msg_data[0]);
        g_CANB_LoopData.ECU.ThrottlePositionDeciPct = CAN_DecodeReadLe16(&recv_data->msg_data[2]);
        g_CANB_LoopData.ECU.OilPressureMilliKpa = CAN_DecodeReadLe16(&recv_data->msg_data[4]);
      }
      break;

    case CAN_ID_CANB_ENERGY_METER_STATUS:
      if (recv_data->dlc == CAN_DECODE_LEN_6)
      {
        uint8_t status_bits = recv_data->msg_data[1];

        /* This frame is unique to FS and seeds the generic energy meter view
         * with status, pack voltage, and pack current immediately.
         */
        g_CANB_LoopData.EnergyMeter.Status = 0UL;
        if ((status_bits & (1U << 0)) != 0U) g_CANB_LoopData.EnergyMeter.Status |= CANB_FS_STATUS_READY_BIT;
        if ((status_bits & (1U << 1)) != 0U) g_CANB_LoopData.EnergyMeter.Status |= CANB_FS_STATUS_LOGGING_BIT;
        if ((status_bits & (1U << 2)) != 0U) g_CANB_LoopData.EnergyMeter.Status |= CANB_FS_STATUS_TRIG_VOLTAGE_BIT;
        if ((status_bits & (1U << 3)) != 0U) g_CANB_LoopData.EnergyMeter.Status |= CANB_FS_STATUS_TRIG_CURRENT_BIT;

        g_CANB_LoopData.EnergyMeter.MsgCounter = recv_data->msg_data[0];
        g_CANB_LoopData.EnergyMeter.ReceivedFlags |= CANB_ENERGY_VALID_STATUS;
        g_CANB_LoopData.EnergyMeter.LastUpdateMs[4] = HAL_GetTick();
        g_CANB_LoopData.EnergyMeter.LastUpdateMs[0] = HAL_GetTick();
        g_CANB_LoopData.EnergyMeter.LastUpdateMs[1] = HAL_GetTick();
        g_CANB_LoopData.EnergyMeter.VoltageMv = (int32_t)CAN_DecodeReadBe16(&recv_data->msg_data[2]) * 16;
        g_CANB_LoopData.EnergyMeter.CurrentMa = (int32_t)CAN_DecodeReadBe16(&recv_data->msg_data[4]) * 64;
        g_CANB_LoopData.EnergyMeter.ValidFlags |= (CANB_ENERGY_VALID_STATUS |
                                                   CANB_ENERGY_VALID_VOLTAGE |
                                                   CANB_ENERGY_VALID_CURRENT);
        g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_FS;
      }
      break;

    case CAN_ID_CANB_WHEEL_ACTUAL_TORQUE:
      if (recv_data->dlc == CAN_DECODE_LEN_8)
      {
        CAN_DecodeStoreWheelLe16(g_CANB_LoopData.ECU.Motor_Torque, recv_data->msg_data);
        }
      break;

    case CAN_ID_CANB_REAR_MOTOR_DIAGNOSTIC_CODE:
      if (recv_data->dlc == CAN_DECODE_LEN_8)
      {
        g_CANB_LoopData.ECU.DiagnosticNumber[CAN_DECODE_MOTOR_RL] = CAN_DecodeReadLe32(&recv_data->msg_data[0]);
        g_CANB_LoopData.ECU.DiagnosticNumber[CAN_DECODE_MOTOR_RR] = CAN_DecodeReadLe32(&recv_data->msg_data[4]);
      }
      break;

    case CAN_ID_CANB_FRONT_MOTOR_DIAGNOSTIC_CODE:
      if (recv_data->dlc == CAN_DECODE_LEN_8)
      {
        g_CANB_LoopData.ECU.DiagnosticNumber[CAN_DECODE_MOTOR_FL] = CAN_DecodeReadLe32(&recv_data->msg_data[0]);
        g_CANB_LoopData.ECU.DiagnosticNumber[CAN_DECODE_MOTOR_FR] = CAN_DecodeReadLe32(&recv_data->msg_data[4]);
      }
      break;

    case CAN_ID_CANB_WHEEL_ACTUAL_SPEED:
      if (recv_data->dlc == CAN_DECODE_LEN_8)
      {
        CAN_DecodeStoreWheelLe16(g_CANB_LoopData.ECU.Motor_RPM, recv_data->msg_data);
        }
      break;

    case CAN_ID_CANB_MOTOR_TEMPERATURE:
      if (recv_data->dlc == CAN_DECODE_LEN_8)
      {
        CAN_DecodeStoreWheelLe16(g_CANB_LoopData.ECU.MotorTempDc, recv_data->msg_data);
      }
      break;

    case CAN_ID_CANB_INVERTER_TEMPERATURE:
      if (recv_data->dlc == CAN_DECODE_LEN_8)
      {
        CAN_DecodeStoreWheelLe16(g_CANB_LoopData.ECU.InverterTempDc, recv_data->msg_data);
      }
      break;

    case CAN_ID_CANB_IGBT_TEMPERATURE:
      if (recv_data->dlc == CAN_DECODE_LEN_8)
      {
        CAN_DecodeStoreWheelLe16(g_CANB_LoopData.ECU.IgbtTempDc, recv_data->msg_data);
      }
      break;

    case CAN_ID_CANB_AMK_STATUS_MODE_LOGIC:
      if (recv_data->dlc == CAN_DECODE_LEN_5)
      {
        int8_t mode_flag = CAN_DecodeSignExtend4((uint8_t)(recv_data->msg_data[0] >> 4));
        g_CANB_LoopData.ECU.driving_mode = (int8_t)(mode_flag + 2);
        g_CANB_LoopData.ECU.ERRO[CAN_DECODE_MOTOR_FR] = (recv_data->msg_data[0] & 0x01U) ? 1 : 0;
        g_CANB_LoopData.ECU.ERRO[CAN_DECODE_MOTOR_FL] = (recv_data->msg_data[0] & 0x02U) ? 1 : 0;
        g_CANB_LoopData.ECU.ERRO[CAN_DECODE_MOTOR_RR] = (recv_data->msg_data[0] & 0x04U) ? 1 : 0;
        g_CANB_LoopData.ECU.ERRO[CAN_DECODE_MOTOR_RL] = (recv_data->msg_data[0] & 0x08U) ? 1 : 0;
        g_CANB_LoopData.ECU.LogicState[CAN_DECODE_MOTOR_RR] = (uint8_t)(recv_data->msg_data[3] & 0x0FU);
        g_CANB_LoopData.ECU.LogicState[CAN_DECODE_MOTOR_RL] = (uint8_t)((recv_data->msg_data[3] >> 4) & 0x0FU);
        g_CANB_LoopData.ECU.LogicState[CAN_DECODE_MOTOR_FR] = (uint8_t)(recv_data->msg_data[4] & 0x0FU);
        g_CANB_LoopData.ECU.LogicState[CAN_DECODE_MOTOR_FL] = (uint8_t)((recv_data->msg_data[4] >> 4) & 0x0FU);
      }
      break;

    case CAN_ID_COMPETITION_CURRENT:
    case CAN_ID_COMPETITION_U1:
    case CAN_ID_COMPETITION_POWER:
    case CAN_ID_COMPETITION_WH:
      /* Competition results on CANB always carry a big-endian int32. */
      (void)CAN_DecodeCompetitionResult(recv_data);
      break;

    case CAN_ID_IMU_RAW:
      if ((recv_data->dlc == 8U) && (recv_data->msg_data[0] == 0x55U))
      {
        /* Byte0/1 are header/subtype, not accelerometer samples. */
        if (recv_data->msg_data[1] == 0x51U)
        {
          g_CANB_LoopData.IMU.Acc_X_Raw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[2]);
          g_CANB_LoopData.IMU.Acc_Y_Raw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[4]);
          g_CANB_LoopData.IMU.Acc_Z_Raw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[6]);
          IMU_Acc_Calibrate();
          g_CANB_LoopData.IMU.ValidFlags |= CANB_MOTION_VALID_ACCEL;
        }
        else if (recv_data->msg_data[1] == 0x52U)
        {
          g_CANB_LoopData.IMU.YawRateRaw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[6]);
          g_CANB_LoopData.IMU.ValidFlags |= CANB_MOTION_VALID_GYRO;
        }
        else if (recv_data->msg_data[1] == 0x53U)
        {
          g_CANB_LoopData.IMU.YawRaw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[6]);
          g_CANB_LoopData.IMU.ValidFlags |= CANB_MOTION_VALID_YAW;
        }
      }
      break;

    case CAN_ID_IMU_ACCEL:
      if (recv_data->dlc == CAN_DECODE_LEN_6)
      {
        /* Dedicated accel frame from the forwarded IMU stream. */
        g_CANB_LoopData.IMU.Acc_X_Raw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[0]);
        g_CANB_LoopData.IMU.Acc_Y_Raw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[2]);
        g_CANB_LoopData.IMU.Acc_Z_Raw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[4]);
        IMU_Acc_Calibrate();
        g_CANB_LoopData.IMU.ValidFlags |= CANB_MOTION_VALID_ACCEL;
      }
      break;

    case CAN_ID_IMU_GYRO:
      if (recv_data->dlc == CAN_DECODE_LEN_6)
      {
        /* Only yaw rate is currently consumed by protobuf motion telemetry. */
        g_CANB_LoopData.IMU.YawRateRaw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[4]);
        g_CANB_LoopData.IMU.ValidFlags |= CANB_MOTION_VALID_GYRO;
      }
      break;

    case CAN_ID_IMU_YAW:
      if (recv_data->dlc == CAN_DECODE_LEN_2)
      {
        /* Yaw angle is stored raw here and scaled later in publish. */
        g_CANB_LoopData.IMU.YawRaw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[0]);
        g_CANB_LoopData.IMU.ValidFlags |= CANB_MOTION_VALID_YAW;
      }
      break;

    default:
      break;
  }
}

uint8_t CAN_IsBatteryCanbMessage(const CAN_Msg_Queue_t *frame)
{
  return (frame != NULL) && (frame->can_channel == CAN_CHANNEL_VEHICLE_CANB) &&
      (frame->is_ext_id == 0U) &&
      ((frame->msg_id == CAN_ID_CANB_BATTERY_POWER_STATUS) ||
       (frame->msg_id == CAN_ID_CANB_BATTERY_FAULT_STATUS) ||
       (frame->msg_id == CAN_ID_CANB_BATTERY_ALARM_LEVELS));
}

void CAN_DecodeBatteryBoxMessage(const CAN_Msg_Queue_t *recv_data)
{
  if ((recv_data == NULL) || (recv_data->dlc > 8U)) return;
  if (CAN_IsBatteryCanbMessage(recv_data))
  {
    CAN_Msg_Queue_t mirror = *recv_data;
    mirror.is_ext_id = 1U;
    switch (recv_data->msg_id)
    {
      case CAN_ID_CANB_BATTERY_POWER_STATUS: mirror.msg_id = CAN_ID_BATTERY_PACK_SUMMARY; break;
      case CAN_ID_CANB_BATTERY_FAULT_STATUS: mirror.msg_id = CAN_ID_BATTERY_ALARM_STATUS; break;
      default: mirror.msg_id = CAN_ID_BATTERY_ALARM_LEVELS; break;
    }
    CAN_DecodeBatteryBoxExtMessage(&mirror);
  }
  else if (recv_data->can_channel == CAN_CHANNEL_BATTERY_BOX)
  {
    if (recv_data->is_ext_id == 0U) CAN_DecodeBatteryBoxStdMessage(recv_data);
    else CAN_DecodeBatteryBoxExtMessage(recv_data);
  }
}

void CAN_DecodeControllerCanaMessage(const CAN_Msg_Queue_t *recv_data)
{
  /* Decode the controller CANA stream into motor-specific telemetry slots. */
  uint8_t motor_index;

  if ((recv_data == NULL) ||
      (recv_data->can_channel != CAN_CHANNEL_CONTROLLER_CANA) ||
      (recv_data->is_ext_id != 0U) ||
      (recv_data->dlc < CAN_DECODE_LEN_8))
  {
    return;
  }

  if (CAN_DecodeControllerMotorIndex(recv_data->msg_id, &motor_index) == 0U)
  {
    return;
  }

  switch (recv_data->msg_id)
  {
    case CAN_ID_CANA_FR_STATUS_SPEED_TORQUE_CURRENT:
    case CAN_ID_CANA_FL_STATUS_SPEED_TORQUE_CURRENT:
    case CAN_ID_CANA_RR_STATUS_SPEED_TORQUE_CURRENT:
    case CAN_ID_CANA_RL_STATUS_SPEED_TORQUE_CURRENT:
      g_CANB_LoopData.ECU.StatusBits[motor_index] = recv_data->msg_data[1];
      g_CANB_LoopData.ECU.ERRO[motor_index] = (recv_data->msg_data[1] & 0x02U) ? 1 : 0;
      g_CANB_LoopData.ECU.Motor_RPM[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[2]);
      g_CANB_LoopData.ECU.Motor_Torque[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[4]);
      g_CANB_LoopData.ECU.MagnetizingCurrent[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[6]);
      break;

    case CAN_ID_CANA_FR_DIAGNOSTIC_INFO:
    case CAN_ID_CANA_FL_DIAGNOSTIC_INFO:
    case CAN_ID_CANA_RR_DIAGNOSTIC_INFO:
    case CAN_ID_CANA_RL_DIAGNOSTIC_INFO:
      g_CANB_LoopData.ECU.DiagnosticNumber[motor_index] = CAN_DecodeReadLe32(&recv_data->msg_data[0]);
      g_CANB_LoopData.ECU.ErrorInfo[motor_index] = CAN_DecodeReadLe32(&recv_data->msg_data[4]);
      break;

    case CAN_ID_CANA_FR_MOTOR_INVERTER_IGBT_TEMP_DCBUS:
    case CAN_ID_CANA_FL_MOTOR_INVERTER_IGBT_TEMP_DCBUS:
    case CAN_ID_CANA_RR_MOTOR_INVERTER_IGBT_TEMP_DCBUS:
    case CAN_ID_CANA_RL_MOTOR_INVERTER_IGBT_TEMP_DCBUS:
      g_CANB_LoopData.ECU.MotorTempDc[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[0]);
      g_CANB_LoopData.ECU.InverterTempDc[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[2]);
      g_CANB_LoopData.ECU.IgbtTempDc[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[4]);
      g_CANB_LoopData.ECU.DCBusVoltage[motor_index] = CAN_DecodeReadLe16(&recv_data->msg_data[6]);
      break;

    case CAN_ID_CANA_FR_TORQUE_LIMIT_POWER:
    case CAN_ID_CANA_FL_TORQUE_LIMIT_POWER:
    case CAN_ID_CANA_RR_TORQUE_LIMIT_POWER:
    case CAN_ID_CANA_RL_TORQUE_LIMIT_POWER:
      g_CANB_LoopData.ECU.TorqueLimitPositive[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[0]);
      g_CANB_LoopData.ECU.TorqueLimitNegative[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[2]);
      g_CANB_LoopData.ECU.MotorPowerW[motor_index] = CAN_DecodeReadLe32(&recv_data->msg_data[4]);
      g_CANB_LoopData.ECU.Motor_Power[motor_index] = (float)g_CANB_LoopData.ECU.MotorPowerW[motor_index] / 1000.0f;
      ECU_TotalPower_Calc();
      break;

    default:
      break;
  }
}

void CAN_DecodeCdcMonitorMessage(const CAN_Msg_Queue_t *recv_data)
{
  /* Keep CDC monitoring read-only so future mailbox semantics stay isolated. */
  if ((recv_data == NULL) ||
      (recv_data->can_channel != CAN_CHANNEL_CDC_MONITOR) ||
      (recv_data->is_ext_id != 0U))
  {
    return;
  }

  /* CDC on SPI3 is currently treated as a read-only monitoring bus.
   * A dedicated decoder entry point is kept here so future CDC mailbox logic
   * can be added without coupling it to battery-box or CANA handling.
   */
  g_CDCInfo.FrameCounter++;
  g_CDCInfo.LastMsgId = recv_data->msg_id;
  g_CDCInfo.LastError = 0UL;
  g_CDCInfo.LastDlc = recv_data->dlc;
  g_CDCInfo.LastIsExtId = recv_data->is_ext_id;
  g_CDCInfo.Valid = 1U;
  (void)memset(g_CDCInfo.LastData, 0, sizeof(g_CDCInfo.LastData));
  (void)memcpy(g_CDCInfo.LastData, recv_data->msg_data,
               (recv_data->dlc <= sizeof(g_CDCInfo.LastData)) ? recv_data->dlc : sizeof(g_CDCInfo.LastData));

  switch (recv_data->msg_id)
  {
    default:
      break;
  }
}

static void CAN_DecodeBatteryBoxStdMessage(const CAN_Msg_Queue_t *recv_data)
{
  if ((recv_data != NULL) && (recv_data->is_ext_id == 0U) &&
      (recv_data->msg_id >= CAN_ID_BATTERY_IVT_CURRENT) &&
      (recv_data->msg_id <= CAN_ID_BATTERY_IVT_WH))
  {
    CAN_DecodeBatteryIvt(recv_data);
  }
}

static void CAN_DecodeBatteryBoxExtMessage(const CAN_Msg_Queue_t *recv_data)
{
  /* Extended battery-box frames carry the richer pack-level telemetry set. */
  if (recv_data == NULL || recv_data->is_ext_id == 0U)
  {
    return;
  }

  if ((recv_data->msg_id >= CAN_ID_BATTERY_CELL_VOLTAGE_BASE) &&
      (recv_data->msg_id <= CAN_ID_BATTERY_CELL_VOLTAGE_LAST) &&
      (((recv_data->msg_id - CAN_ID_BATTERY_CELL_VOLTAGE_BASE) & 0xFFFFUL) == 0UL) &&
      (recv_data->dlc == CAN_DECODE_LEN_8))
  {
    uint32_t frame_no = (recv_data->msg_id - CAN_ID_BATTERY_CELL_VOLTAGE_BASE) >> 16;
    uint8_t module_id = (uint8_t)(frame_no / CAN_BATTERY_CELL_VOLTAGE_FRAMES_PER_MODULE);
    uint8_t frame_in_module = (uint8_t)(frame_no % CAN_BATTERY_CELL_VOLTAGE_FRAMES_PER_MODULE);

    if (module_id < CAN_BATTERY_MODULE_COUNT)
    {
      g_BatteryInfo.CellFrameTick[frame_no] = HAL_GetTick();
      g_BatteryInfo.CellFramesReceived |= (1ULL << frame_no);
      if (frame_in_module == 0U)
      {
        for (uint8_t i = 0U; i < 3U; i++)
        {
          uint16_t value = CAN_DecodeReadLe16(&recv_data->msg_data[2U + (2U * i)]);
          uint32_t bit = 1UL << (i);
          g_BatteryInfo.CellValidMask[module_id] &= ~bit;
          g_BatteryInfo.CellVolt[module_id][i] = 0U;
          if ((value >= 500U) && (value <= 5000U))
          {
            g_BatteryInfo.CellVolt[module_id][i] = value;
            g_BatteryInfo.CellValidMask[module_id] |= bit;
          }
        }
      }
      else
      {
        uint8_t cell_base = (uint8_t)(3U + ((frame_in_module - 1U) * 4U));
        for (uint8_t i = 0U; i < 4U; i++)
        {
          uint16_t value = CAN_DecodeReadLe16(&recv_data->msg_data[2U * i]);
          uint32_t bit = 1UL << (cell_base + i);
          g_BatteryInfo.CellValidMask[module_id] &= ~bit;
          g_BatteryInfo.CellVolt[module_id][cell_base + i] = 0U;
          if ((value >= 500U) && (value <= 5000U))
          {
            g_BatteryInfo.CellVolt[module_id][cell_base + i] = value;
            g_BatteryInfo.CellValidMask[module_id] |= bit;
          }
        }
      }
    }
    return;
  }

  if ((recv_data->msg_id >= CAN_ID_BATTERY_CELL_TEMP_BASE) &&
      (recv_data->msg_id <= CAN_ID_BATTERY_CELL_TEMP_LAST) &&
      (((recv_data->msg_id - CAN_ID_BATTERY_CELL_TEMP_BASE) & 0xFFFFUL) == 0UL) &&
      (recv_data->dlc == CAN_DECODE_LEN_8))
  {
    uint8_t module_id = (uint8_t)((recv_data->msg_id - CAN_ID_BATTERY_CELL_TEMP_BASE) >> 16);

    if (module_id < CAN_BATTERY_MODULE_COUNT)
    {
      g_BatteryInfo.TempFrameTick[module_id] = HAL_GetTick();
      g_BatteryInfo.TempFramesReceived |= (uint8_t)(1U << module_id);
      g_BatteryInfo.TempValidMask[module_id] = 0U;
      for (uint8_t i = 0U; i < BAT_TEMP_POINT_PER_MOD; i++)
      {
        uint8_t raw = recv_data->msg_data[i];
        g_BatteryInfo.ModTemp[module_id][i] = 0;
        if (raw <= 129U)
        {
          g_BatteryInfo.ModTemp[module_id][i] = (int8_t)((int16_t)raw - BAT_TEMP_OFFSET);
          g_BatteryInfo.TempValidMask[module_id] |= (uint8_t)(1U << i);
        }
      }
    }
    return;
  }

  switch (recv_data->msg_id)
  {
    case CAN_ID_BATTERY_PACK_SUMMARY:
      if (recv_data->dlc == CAN_DECODE_LEN_7)
      {
        uint16_t pack_voltage_deci_v = CAN_DecodeReadBe16(&recv_data->msg_data[0]);
        int16_t pack_current_raw = (int16_t)CAN_DecodeReadBe16(&recv_data->msg_data[2]);
        uint8_t flags = recv_data->msg_data[5] & 0x1FU;
        if (recv_data->msg_data[4] > 100U) flags &= (uint8_t)~BATTERY_SOC_VALID;
        g_BatteryInfo.PackValidFlags = flags;
        g_BatteryInfo.PackReceived = 1U;
        g_BatteryInfo.PackLastUpdateMs = HAL_GetTick();
        g_BatteryInfo.TotalVolt = (flags & BATTERY_PACK_VOLT_VALID) ? (uint32_t)pack_voltage_deci_v * 100U : 0U;
        g_BatteryInfo.TotalCurrent = (flags & BATTERY_PACK_CURR_VALID) ? (int32_t)pack_current_raw * 100 : 0;
        g_BatteryInfo.SOC = (flags & BATTERY_SOC_VALID) ? recv_data->msg_data[4] : 0U;
        g_BatteryInfo.Battery_State = (uint8_t)((recv_data->msg_data[6] >> 4) & 0x0FU);
        g_BatteryInfo.Battery_AlarmLevel = (uint8_t)(recv_data->msg_data[6] & 0x0FU);
        g_BatteryInfo.Work_State = g_BatteryInfo.Battery_State;
      }
      break;

    case CAN_ID_BATTERY_CELL_EXTREMA:
      if (recv_data->dlc == CAN_DECODE_LEN_6)
      {
        g_BatteryInfo.CellExtremaTick = HAL_GetTick();
        g_BatteryInfo.CellExtremaValid = 1U;
        g_BatteryInfo.MaxCellVolt = CAN_DecodeReadBe16(&recv_data->msg_data[0]);
        g_BatteryInfo.MinCellVolt = CAN_DecodeReadBe16(&recv_data->msg_data[2]);
        g_BatteryInfo.MaxCellVoltNo = recv_data->msg_data[4];
        g_BatteryInfo.MinCellVoltNo = recv_data->msg_data[5];
        if ((g_BatteryInfo.MaxCellVolt == 0xFFFFU) || (g_BatteryInfo.MinCellVolt == 0xFFFFU) ||
            (g_BatteryInfo.MaxCellVoltNo >= 138U) || (g_BatteryInfo.MinCellVoltNo >= 138U))
        {
          g_BatteryInfo.CellExtremaValid = 0U;
          g_BatteryInfo.MaxCellVolt = g_BatteryInfo.MinCellVolt = 0U;
        }
      }
      break;

    case CAN_ID_BATTERY_TEMP_EXTREMA:
      if (recv_data->dlc == CAN_DECODE_LEN_8)
      {
        g_BatteryInfo.TempExtremaTick = HAL_GetTick();
        g_BatteryInfo.TempExtremaValid = (recv_data->msg_data[0] <= 129U) && (recv_data->msg_data[1] <= 129U) &&
            (recv_data->msg_data[2] < 48U) && (recv_data->msg_data[3] < 48U);
        g_BatteryInfo.MaxTemp = g_BatteryInfo.TempExtremaValid ? (int8_t)(recv_data->msg_data[0] - BAT_TEMP_OFFSET) : 0;
        g_BatteryInfo.MinTemp = g_BatteryInfo.TempExtremaValid ? (int8_t)(recv_data->msg_data[1] - BAT_TEMP_OFFSET) : 0;
        g_BatteryInfo.CoolingControl = recv_data->msg_data[4];
        g_BatteryInfo.FanTargetDuty = recv_data->msg_data[5];
        g_BatteryInfo.FanSpeedRpm = (uint16_t)recv_data->msg_data[6] * 100U;
        g_BatteryInfo.FanStatusFlags = recv_data->msg_data[7];
        g_BatteryInfo.MaxTempNo = recv_data->msg_data[2];
        g_BatteryInfo.MinTempNo = recv_data->msg_data[3];
      }
      break;

    case CAN_ID_BATTERY_STATUS:
      if (recv_data->dlc == CAN_DECODE_LEN_8)
      {
        g_BatteryInfo.Air1_State = (uint8_t)((recv_data->msg_data[0] >> 6) & 0x03U);
        g_BatteryInfo.Air2_State = (uint8_t)((recv_data->msg_data[0] >> 4) & 0x03U);
        g_BatteryInfo.Air3_State = (uint8_t)((recv_data->msg_data[0] >> 2) & 0x03U);
        g_BatteryInfo.Charge_Enable = (uint8_t)((recv_data->msg_data[1] >> 4) & 0x01U);
        g_BatteryInfo.ChargeCommValid = (recv_data->msg_data[1] >> 3) & 1U;
        g_BatteryInfo.Charge_ReqVolt = CAN_DecodeReadBe16(&recv_data->msg_data[2]);
        g_BatteryInfo.Charge_ReqCurr = CAN_DecodeReadBe16(&recv_data->msg_data[4]);
        g_BatteryInfo.PrechargeVolt = CAN_DecodeReadBe16(&recv_data->msg_data[6]);
        Battery_Charger_Calibrate();
      }
      break;

    case CAN_ID_BATTERY_CELL_SUM:
      if (recv_data->dlc == CAN_DECODE_LEN_2)
      {
        g_BatteryInfo.CellSumTick = HAL_GetTick();
        g_BatteryInfo.CellVoltSum = CAN_DecodeReadBe16(&recv_data->msg_data[0]);
        g_BatteryInfo.CellSumValid = g_BatteryInfo.CellVoltSum != 0xFFFFU;
        if (!g_BatteryInfo.CellSumValid) g_BatteryInfo.CellVoltSum = 0U;
      }
      break;

    case CAN_ID_BATTERY_ALARM_STATUS:
      if (recv_data->dlc == CAN_DECODE_LEN_8)
      {
        g_BatteryInfo.Battery_State = recv_data->msg_data[0] >> 4;
        g_BatteryInfo.Work_State = g_BatteryInfo.Battery_State;
        g_BatteryInfo.Battery_AlarmLevel = recv_data->msg_data[0] & 0x0FU;
        g_BatteryInfo.BatteryFaultCode = CAN_DecodeReadBe32(&recv_data->msg_data[1]);
        g_BatteryInfo.FaultFlags = recv_data->msg_data[5];
        g_BatteryInfo.SlaveOfflineMask = recv_data->msg_data[6] & 0x3FU;
        g_BatteryInfo.FaultFrameVersion = recv_data->msg_data[7];
        g_BatteryInfo.FaultValid = 1U;
        g_BatteryInfo.FaultTick = HAL_GetTick();
      }
      break;

    case CAN_ID_BATTERY_ALARM_LEVELS:
      if (recv_data->dlc == 8U)
      {
        for (uint8_t i = 0U; i < 32U; ++i)
          g_BatteryInfo.AlarmLevels[i] = (recv_data->msg_data[i / 4U] >> ((i % 4U) * 2U)) & 3U;
        g_BatteryInfo.AlarmLevelsValid = 1U;
        g_BatteryInfo.AlarmLevelsTick = HAL_GetTick();
      }
      break;

    case CAN_ID_BATTERY_IMD_DIAG:
      if (recv_data->dlc == 8U)
      {
        g_BatteryInfo.ImdReceived = 1U;
        g_BatteryInfo.ImdFrequencyClass = recv_data->msg_data[0] & 0x0FU;
        g_BatteryInfo.IMD_State = recv_data->msg_data[0] >> 4;
        g_BatteryInfo.ImdFlags = recv_data->msg_data[1];
        g_BatteryInfo.ImdDutyDeciPct = CAN_DecodeReadBe16(&recv_data->msg_data[2]);
        g_BatteryInfo.ImdResistanceKohm = CAN_DecodeReadBe16(&recv_data->msg_data[4]);
        g_BatteryInfo.ImdFrequencyCentiHz = CAN_DecodeReadBe16(&recv_data->msg_data[6]);
      }
      break;

    case CAN_ID_BATTERY_HV_STATUS:
      if (recv_data->dlc == 8U)
      {
        g_BatteryInfo.HvRequestFlags = recv_data->msg_data[0];
        g_BatteryInfo.LastPrechargeResult = (recv_data->msg_data[0] >> 2) & 3U;
        g_BatteryInfo.LastPrechargeSuccessMs = CAN_DecodeReadBe16(&recv_data->msg_data[1]);
        g_BatteryInfo.LastPrechargeFailureMs = CAN_DecodeReadBe16(&recv_data->msg_data[3]);
      }
      break;

    case CAN_ID_BATTERY_SOP_MIRROR:
      if (recv_data->dlc == 8U)
      {
        g_BatteryInfo.SopDischargeCurrentDa = CAN_DecodeReadBe16(&recv_data->msg_data[0]);
        g_BatteryInfo.SopChargeCurrentDa = CAN_DecodeReadBe16(&recv_data->msg_data[2]);
        g_BatteryInfo.SopDischargePowerDkw = CAN_DecodeReadBe16(&recv_data->msg_data[4]);
        g_BatteryInfo.SopChargePowerDkw = CAN_DecodeReadBe16(&recv_data->msg_data[6]);
      }
      break;

    case CAN_ID_BATTERY_CHARGER_FEEDBACK:
      if (recv_data->dlc >= CAN_DECODE_LEN_5)
      {
        g_BatteryInfo.Charger_OutVolt = CAN_DecodeReadBe16(&recv_data->msg_data[0]);
        g_BatteryInfo.Charger_OutCurr = (int16_t)CAN_DecodeReadBe16(&recv_data->msg_data[2]);
        g_BatteryInfo.Charger_Cmd = recv_data->msg_data[4];
        Battery_Charger_Calibrate();
      }
      break;

    default:
      break;
  }
}

/* Called by the battery owner task, including while its receive queue is idle.
 * Unsigned subtraction intentionally handles HAL tick wraparound.
 */
void CAN_DecodeBatteryService(uint32_t now)
{
  for (uint8_t i = 0U; i < 8U; ++i)
  {
    if ((uint32_t)(now - g_BatteryInfo.IVT.LastUpdateMs[i]) >= BATTERY_IVT_TIMEOUT_MS)
      g_BatteryInfo.IVT.ValidFlags &= (uint8_t)~(1U << i);
  }
  for (uint8_t i = 0U; i < 36U; ++i)
  {
    if (((g_BatteryInfo.CellFramesReceived >> i) & 1U) &&
        ((uint32_t)(now - g_BatteryInfo.CellFrameTick[i]) >= BATTERY_SAMPLE_TIMEOUT_MS))
    {
      uint8_t module = i / 6U, frame = i % 6U;
      uint8_t first = frame == 0U ? 0U : (uint8_t)(3U + (frame - 1U) * 4U);
      uint8_t count = frame == 0U ? 3U : 4U;
      for (uint8_t cell = first; cell < first + count; ++cell)
      {
        g_BatteryInfo.CellValidMask[module] &= ~(1UL << cell);
        g_BatteryInfo.CellVolt[module][cell] = 0U;
      }
      g_BatteryInfo.CellFramesReceived &= ~(1ULL << i);
    }
  }
  for (uint8_t i = 0U; i < 6U; ++i)
  {
    if (((g_BatteryInfo.TempFramesReceived >> i) & 1U) &&
        ((uint32_t)(now - g_BatteryInfo.TempFrameTick[i]) >= BATTERY_SAMPLE_TIMEOUT_MS))
    {
      g_BatteryInfo.TempValidMask[i] = 0U;
      memset(g_BatteryInfo.ModTemp[i], 0, sizeof(g_BatteryInfo.ModTemp[i]));
      g_BatteryInfo.TempFramesReceived &= (uint8_t)~(1U << i);
    }
  }
  if ((uint32_t)(now - g_BatteryInfo.PackLastUpdateMs) >= BATTERY_STATUS_TIMEOUT_MS)
  {
    g_BatteryInfo.PackValidFlags = 0U;
    g_BatteryInfo.TotalVolt = 0U;
    g_BatteryInfo.TotalCurrent = 0;
    g_BatteryInfo.SOC = 0U;
  }
  if ((uint32_t)(now - g_BatteryInfo.CellExtremaTick) >= BATTERY_STATUS_TIMEOUT_MS)
  {
    g_BatteryInfo.CellExtremaValid = 0U;
    g_BatteryInfo.MaxCellVolt = g_BatteryInfo.MinCellVolt = 0U;
  }
  if ((uint32_t)(now - g_BatteryInfo.TempExtremaTick) >= BATTERY_STATUS_TIMEOUT_MS)
  {
    g_BatteryInfo.TempExtremaValid = 0U;
    g_BatteryInfo.MaxTemp = g_BatteryInfo.MinTemp = 0;
  }
  if ((uint32_t)(now - g_BatteryInfo.CellSumTick) >= BATTERY_STATUS_TIMEOUT_MS)
  {
    g_BatteryInfo.CellSumValid = 0U;
    g_BatteryInfo.CellVoltSum = 0U;
  }
  /* Retain the last fault bitmap for diagnosis, but expose that it is stale. */
  if ((uint32_t)(now - g_BatteryInfo.FaultTick) >= BATTERY_STATUS_TIMEOUT_MS)
    g_BatteryInfo.FaultValid = 0U;
  if ((uint32_t)(now - g_BatteryInfo.AlarmLevelsTick) >= 3000U)
    g_BatteryInfo.AlarmLevelsValid = 0U;
}

void CAN_DecodeVehicleService(uint32_t now)
{
  /* Competition timing is provisional; use the gateway's 2 s freshness policy. */
  for (uint8_t i = 0U; i < 5U; ++i)
  {
    if ((uint32_t)(now - g_CANB_LoopData.EnergyMeter.LastUpdateMs[i]) >= 2000U)
      g_CANB_LoopData.EnergyMeter.ValidFlags &= (uint8_t)~(1U << i);
  }
  if ((g_CANB_LoopData.EnergyMeter.ValidFlags & CANB_ENERGY_VALID_STATUS) == 0U)
  {
    g_CANB_LoopData.EnergyMeter.Status = 0U;
    g_CANB_LoopData.EnergyMeter.AutoSource =
        (g_CANB_LoopData.EnergyMeter.ValidFlags != 0U) ? CANB_ENERGY_METER_SOURCE_COMPETITION_IVT : CANB_ENERGY_METER_SOURCE_UNKNOWN;
  }
}
