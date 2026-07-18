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
static void CAN_DecodeRecalculateMotorPower(void);
static uint32_t CAN_DecodeBuildBatteryFaultCode(const uint8_t *data);
static uint8_t CAN_DecodeIvtValueIsPlausible(uint32_t msg_id, int32_t value);
/* Decode one IVT numeric payload while tolerating either byte order.
 * The selected byte order is cached once a plausible result is found.
 */
static uint8_t CAN_DecodeIvtResultValue(uint32_t msg_id, const uint8_t *data, int32_t *value);
/* Team IVT path: same message IDs, little-endian payload semantics after the
 * byte-order detector has decided the frame is IVT-like.
 */
static uint8_t CAN_DecodeVehicleCanbIvtResult(const CAN_Msg_Queue_t *recv_data);
/* FS path: the competition logger reuses the IVT-like IDs but stores the
 * result payload in big-endian format and exposes its own status/mux meaning.
 */
static uint8_t CAN_DecodeVehicleCanbFsResult(const CAN_Msg_Queue_t *recv_data);
/* Decide whether a shared CANB energy frame currently belongs to IVT or FS.
 * The user has clarified the car will only carry one of them at a time, but
 * the decoder keeps this arbitration logic for compatibility and robustness.
 */
static uint8_t CAN_DecodeVehicleCanbEnergyResult(const CAN_Msg_Queue_t *recv_data);
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

static void CAN_DecodeRecalculateMotorPower(void)
{
  for (uint8_t index = 0U; index < 4U; index++)
  {
    g_CANB_LoopData.ECU.Motor_Power[index] = ((float)g_CANB_LoopData.ECU.Motor_RPM[index] *
                                              (float)g_CANB_LoopData.ECU.Motor_Torque[index]) / 9549.0f;
  }

  ECU_TotalPower_Calc();
}

static uint32_t CAN_DecodeBuildBatteryFaultCode(const uint8_t *data)
{
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static uint8_t CAN_DecodeIvtValueIsPlausible(uint32_t msg_id, int32_t value)
{
  int64_t abs_value = value;

  if (abs_value < 0)
  {
    abs_value = -abs_value;
  }

  switch (msg_id)
  {
    case CAN_ID_IVT_CURRENT:
      return (abs_value <= 600000LL) ? 1U : 0U;

    case CAN_ID_IVT_U1:
      return (abs_value <= 1000000LL) ? 1U : 0U;

    case CAN_ID_IVT_POWER:
    case CAN_ID_IVT_WH:
      return (abs_value <= 100000000LL) ? 1U : 0U;

    default:
      return 0U;
  }
}

static uint8_t CAN_DecodeIvtResultValue(uint32_t msg_id, const uint8_t *data, int32_t *value)
{
  int32_t be_value;
  int32_t le_value;
  uint8_t be_plausible;
  uint8_t le_plausible;

  if ((data == NULL) || (value == NULL))
  {
    return 0U;
  }

  if (g_CANB_LoopData.IVT.ByteOrder == CANB_IVT_BYTE_ORDER_BE)
  {
    *value = CAN_DecodeReadBe32Signed(data);
    if (CAN_DecodeIvtValueIsPlausible(msg_id, *value) != 0U)
    {
      return 1U;
    }
    g_CANB_LoopData.IVT.ByteOrder = CANB_IVT_BYTE_ORDER_UNKNOWN;
  }

  if (g_CANB_LoopData.IVT.ByteOrder == CANB_IVT_BYTE_ORDER_LE)
  {
    *value = CAN_DecodeReadLe32Signed(data);
    if (CAN_DecodeIvtValueIsPlausible(msg_id, *value) != 0U)
    {
      return 1U;
    }
    g_CANB_LoopData.IVT.ByteOrder = CANB_IVT_BYTE_ORDER_UNKNOWN;
  }

  be_value = CAN_DecodeReadBe32Signed(data);
  le_value = CAN_DecodeReadLe32Signed(data);
  be_plausible = CAN_DecodeIvtValueIsPlausible(msg_id, be_value);
  le_plausible = CAN_DecodeIvtValueIsPlausible(msg_id, le_value);

  if ((be_plausible != 0U) && (le_plausible == 0U))
  {
    g_CANB_LoopData.IVT.ByteOrder = CANB_IVT_BYTE_ORDER_BE;
    *value = be_value;
    return 1U;
  }

  if ((le_plausible != 0U) && (be_plausible == 0U))
  {
    g_CANB_LoopData.IVT.ByteOrder = CANB_IVT_BYTE_ORDER_LE;
    *value = le_value;
    return 1U;
  }

  if ((be_plausible != 0U) && (le_plausible != 0U) && (be_value == le_value))
  {
    *value = be_value;
    return 1U;
  }

  return 0U;
}

static uint8_t CAN_DecodeVehicleCanbIvtResult(const CAN_Msg_Queue_t *recv_data)
{
  uint8_t mux;
  uint32_t state;
  int32_t value;

  if ((recv_data == NULL) || (recv_data->dlc < CAN_DECODE_LEN_6))
  {
    return 0U;
  }

  mux = recv_data->msg_data[0];
  state = (uint32_t)((recv_data->msg_data[1] >> 4) & 0x0FU);

  if (CAN_DecodeIvtResultValue(recv_data->msg_id, &recv_data->msg_data[2], &value) == 0U)
  {
    return 0U;
  }

  switch (recv_data->msg_id)
  {
    case CAN_ID_IVT_CURRENT:
      if (mux != 0x00U)
      {
        return 0U;
      }
      g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_IVT;
      g_CANB_LoopData.IVT.CurrentMa = value;
      g_CANB_LoopData.IVT.CurrentState = state;
      g_CANB_LoopData.IVT.ValidFlags |= CANB_IVT_VALID_CURRENT;
      return 1U;

    case CAN_ID_IVT_U1:
      if (mux != 0x01U)
      {
        return 0U;
      }
      g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_IVT;
      g_CANB_LoopData.IVT.VoltageU1Mv = value;
      g_CANB_LoopData.IVT.VoltageU1State = state;
      g_CANB_LoopData.IVT.ValidFlags |= CANB_IVT_VALID_U1;
      return 1U;

    case CAN_ID_IVT_POWER:
      if (mux != 0x05U)
      {
        return 0U;
      }
      g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_IVT;
      g_CANB_LoopData.IVT.PowerW = value;
      g_CANB_LoopData.IVT.PowerState = state;
      g_CANB_LoopData.IVT.ValidFlags |= CANB_IVT_VALID_POWER;
      return 1U;

    case CAN_ID_IVT_WH:
      if (mux != 0x07U)
      {
        return 0U;
      }
      g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_IVT;
      g_CANB_LoopData.IVT.EnergyWh = value;
      g_CANB_LoopData.IVT.EnergyState = state;
      g_CANB_LoopData.IVT.ValidFlags |= CANB_IVT_VALID_ENERGY;
      return 1U;

    default:
      return 0U;
  }
}

static uint8_t CAN_DecodeVehicleCanbFsResult(const CAN_Msg_Queue_t *recv_data)
{
  uint8_t mux;
  uint32_t state;
  int32_t value;
  uint8_t msg_counter;

  if ((recv_data == NULL) || (recv_data->dlc < CAN_DECODE_LEN_6))
  {
    return 0U;
  }

  mux = recv_data->msg_data[0];
  state = (uint32_t)((recv_data->msg_data[1] >> 4) & 0x0FU);
  msg_counter = (uint8_t)(recv_data->msg_data[1] & 0x0FU);
  value = CAN_DecodeReadBe32Signed(&recv_data->msg_data[2]);

  switch (recv_data->msg_id)
  {
    case CAN_ID_IVT_CURRENT:
      if (mux != 0x00U)
      {
        return 0U;
      }
      g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_FS;
      g_CANB_LoopData.EnergyMeter.CurrentMa = value;
      g_CANB_LoopData.EnergyMeter.Status = state;
      g_CANB_LoopData.EnergyMeter.MsgCounter = msg_counter;
      g_CANB_LoopData.EnergyMeter.ValidFlags |= (CANB_ENERGY_VALID_CURRENT | CANB_ENERGY_VALID_STATUS);
      return 1U;

    case CAN_ID_IVT_U1:
      if (mux != 0x01U)
      {
        return 0U;
      }
      g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_FS;
      g_CANB_LoopData.EnergyMeter.VoltageMv = value;
      g_CANB_LoopData.EnergyMeter.Status = state;
      g_CANB_LoopData.EnergyMeter.MsgCounter = msg_counter;
      g_CANB_LoopData.EnergyMeter.ValidFlags |= (CANB_ENERGY_VALID_VOLTAGE | CANB_ENERGY_VALID_STATUS);
      return 1U;

    case CAN_ID_IVT_POWER:
      if (mux != 0x05U)
      {
        return 0U;
      }
      g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_FS;
      g_CANB_LoopData.EnergyMeter.PowerW = value;
      g_CANB_LoopData.EnergyMeter.Status = state;
      g_CANB_LoopData.EnergyMeter.MsgCounter = msg_counter;
      g_CANB_LoopData.EnergyMeter.ValidFlags |= (CANB_ENERGY_VALID_POWER | CANB_ENERGY_VALID_STATUS);
      return 1U;

    case CAN_ID_IVT_WH:
      if (mux != 0x07U)
      {
        return 0U;
      }
      g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_FS;
      g_CANB_LoopData.EnergyMeter.EnergyWh = value;
      g_CANB_LoopData.EnergyMeter.Status = state;
      g_CANB_LoopData.EnergyMeter.MsgCounter = msg_counter;
      g_CANB_LoopData.EnergyMeter.ValidFlags |= (CANB_ENERGY_VALID_ENERGY | CANB_ENERGY_VALID_STATUS);
      return 1U;

    default:
      return 0U;
  }
}

static uint8_t CAN_DecodeVehicleCanbEnergyResult(const CAN_Msg_Queue_t *recv_data)
{
  int32_t be_value;
  int32_t le_value;
  uint8_t be_plausible;
  uint8_t le_plausible;

  if ((recv_data == NULL) || (recv_data->dlc < CAN_DECODE_LEN_6))
  {
    return 0U;
  }

  if (g_CANB_LoopData.EnergyMeter.AutoSource == CANB_ENERGY_METER_SOURCE_FS)
  {
    /* FS result frames are interpreted as big-endian first. */
    be_value = CAN_DecodeReadBe32Signed(&recv_data->msg_data[2]);
    if (CAN_DecodeIvtValueIsPlausible(recv_data->msg_id, be_value) != 0U)
    {
      return CAN_DecodeVehicleCanbFsResult(recv_data);
    }
    g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_UNKNOWN;
  }

  if (g_CANB_LoopData.EnergyMeter.AutoSource == CANB_ENERGY_METER_SOURCE_IVT)
  {
    /* IVT result frames reuse the same IDs but must pass IVT plausibility. */
    if (CAN_DecodeVehicleCanbIvtResult(recv_data) != 0U)
    {
      return 1U;
    }
    g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_UNKNOWN;
  }

  be_value = CAN_DecodeReadBe32Signed(&recv_data->msg_data[2]);
  le_value = CAN_DecodeReadLe32Signed(&recv_data->msg_data[2]);
  be_plausible = CAN_DecodeIvtValueIsPlausible(recv_data->msg_id, be_value);
  le_plausible = CAN_DecodeIvtValueIsPlausible(recv_data->msg_id, le_value);

  if ((be_plausible != 0U) && (le_plausible == 0U))
  {
    g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_FS;
    return CAN_DecodeVehicleCanbFsResult(recv_data);
  }

  if ((le_plausible != 0U) && (be_plausible == 0U))
  {
    g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_IVT;
    g_CANB_LoopData.IVT.ByteOrder = CANB_IVT_BYTE_ORDER_LE;
    return CAN_DecodeVehicleCanbIvtResult(recv_data);
  }

  return 0U;
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
    case CAN_ID_CANA_FR_TORQUE_CURRENT_LIMIT_COUNTER:
      *motor_index = CAN_DECODE_MOTOR_FR;
      return 1U;

    case CAN_ID_CANA_FL_STATUS_SPEED_TORQUE_CURRENT:
    case CAN_ID_CANA_FL_DIAGNOSTIC_INFO:
    case CAN_ID_CANA_FL_MOTOR_INVERTER_IGBT_TEMP_DCBUS:
    case CAN_ID_CANA_FL_TORQUE_CURRENT_LIMIT_COUNTER:
      *motor_index = CAN_DECODE_MOTOR_FL;
      return 1U;

    case CAN_ID_CANA_RR_STATUS_SPEED_TORQUE_CURRENT:
    case CAN_ID_CANA_RR_DIAGNOSTIC_INFO:
    case CAN_ID_CANA_RR_MOTOR_INVERTER_IGBT_TEMP_DCBUS:
    case CAN_ID_CANA_RR_TORQUE_CURRENT_LIMIT_COUNTER:
      *motor_index = CAN_DECODE_MOTOR_RR;
      return 1U;

    case CAN_ID_CANA_RL_STATUS_SPEED_TORQUE_CURRENT:
    case CAN_ID_CANA_RL_DIAGNOSTIC_INFO:
    case CAN_ID_CANA_RL_MOTOR_INVERTER_IGBT_TEMP_DCBUS:
    case CAN_ID_CANA_RL_TORQUE_CURRENT_LIMIT_COUNTER:
      *motor_index = CAN_DECODE_MOTOR_RL;
      return 1U;

    default:
      return 0U;
  }
}

void CAN_DecodeVehicleCanbMessage(const CAN_Msg_Queue_t *recv_data)
{
  /* Parse vehicle-side CANB frames and update the shared vehicle telemetry. */
  if (recv_data == NULL || recv_data->can_channel != CAN_CHANNEL_VEHICLE_CANB || recv_data->is_ext_id != 0U)
  {
    return;
  }

  switch (recv_data->msg_id)
  {
    case CAN_ID_CANB_GPS_SPEED:
      if (recv_data->dlc >= CAN_DECODE_LEN_2)
      {
        g_CANB_LoopData.IMU.GpsSpeedKmh = CAN_DecodeReadLe16(recv_data->msg_data);
        g_CANB_LoopData.IMU.ValidFlags |= CANB_MOTION_VALID_GPS;
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
      }
      break;

    case CAN_ID_CANB_DRIVER_INPUTS_OIL_PRESSURE:
      if (recv_data->dlc >= CAN_DECODE_LEN_6)
      {
        g_CANB_LoopData.ECU.SteeringAngleDeciDeg = CAN_DecodeReadLe16Signed(&recv_data->msg_data[0]);
        g_CANB_LoopData.ECU.ThrottlePositionDeciPct = CAN_DecodeReadLe16(&recv_data->msg_data[2]);
        g_CANB_LoopData.ECU.OilPressureMilliKpa = CAN_DecodeReadLe16(&recv_data->msg_data[4]);
      }
      break;

    case CAN_ID_CANB_ENERGY_METER_STATUS:
      if (recv_data->dlc >= CAN_DECODE_LEN_6)
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
        g_CANB_LoopData.EnergyMeter.VoltageMv = (int32_t)CAN_DecodeReadBe16(&recv_data->msg_data[2]) * 16;
        g_CANB_LoopData.EnergyMeter.CurrentMa = (int32_t)CAN_DecodeReadBe16(&recv_data->msg_data[4]) * 64;
        g_CANB_LoopData.EnergyMeter.ValidFlags |= (CANB_ENERGY_VALID_STATUS |
                                                   CANB_ENERGY_VALID_VOLTAGE |
                                                   CANB_ENERGY_VALID_CURRENT);
        g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_FS;
      }
      break;

    case CAN_ID_CANB_LEGACY_VEHICLE_STATE:
      if (recv_data->dlc >= CAN_DECODE_LEN_8)
      {
        g_CANB_LoopData.ECU.Vehicle_Speed = recv_data->msg_data[0];
        g_CANB_LoopData.IMU.Long_Accel = (float)((int8_t)recv_data->msg_data[1]);
        g_CANB_LoopData.IMU.Lat_Accel = (float)((int8_t)recv_data->msg_data[2]);
        for (uint8_t i = 0U; i < 4U; i++)
        {
          g_CANB_LoopData.ECU.Motor_Torque[i] = (int16_t)recv_data->msg_data[i + 3U];
        }
        g_CANB_LoopData.ECU.driving_mode = (int8_t)recv_data->msg_data[7];
      }
      break;

    case CAN_ID_CANB_WHEEL_ACTUAL_TORQUE:
      if (recv_data->dlc >= CAN_DECODE_LEN_8)
      {
        CAN_DecodeStoreWheelLe16(g_CANB_LoopData.ECU.Motor_Torque, recv_data->msg_data);
        CAN_DecodeRecalculateMotorPower();
      }
      break;

    case CAN_ID_CANB_REAR_MOTOR_DIAGNOSTIC_CODE:
      if (recv_data->dlc >= CAN_DECODE_LEN_8)
      {
        g_CANB_LoopData.ECU.DiagnosticNumber[CAN_DECODE_MOTOR_RL] = CAN_DecodeReadLe32(&recv_data->msg_data[0]);
        g_CANB_LoopData.ECU.DiagnosticNumber[CAN_DECODE_MOTOR_RR] = CAN_DecodeReadLe32(&recv_data->msg_data[4]);
      }
      break;

    case CAN_ID_CANB_FRONT_MOTOR_DIAGNOSTIC_CODE:
      if (recv_data->dlc >= CAN_DECODE_LEN_8)
      {
        g_CANB_LoopData.ECU.DiagnosticNumber[CAN_DECODE_MOTOR_FL] = CAN_DecodeReadLe32(&recv_data->msg_data[0]);
        g_CANB_LoopData.ECU.DiagnosticNumber[CAN_DECODE_MOTOR_FR] = CAN_DecodeReadLe32(&recv_data->msg_data[4]);
      }
      break;

    case CAN_ID_CANB_WHEEL_ACTUAL_SPEED:
      if (recv_data->dlc >= CAN_DECODE_LEN_8)
      {
        CAN_DecodeStoreWheelLe16(g_CANB_LoopData.ECU.Motor_RPM, recv_data->msg_data);
        CAN_DecodeRecalculateMotorPower();
      }
      break;

    case CAN_ID_CANB_MOTOR_TEMPERATURE:
      if (recv_data->dlc >= CAN_DECODE_LEN_8)
      {
        CAN_DecodeStoreWheelLe16(g_CANB_LoopData.ECU.MotorTempDc, recv_data->msg_data);
      }
      break;

    case CAN_ID_CANB_INVERTER_TEMPERATURE:
      if (recv_data->dlc >= CAN_DECODE_LEN_8)
      {
        CAN_DecodeStoreWheelLe16(g_CANB_LoopData.ECU.InverterTempDc, recv_data->msg_data);
      }
      break;

    case CAN_ID_CANB_IGBT_TEMPERATURE:
      if (recv_data->dlc >= CAN_DECODE_LEN_8)
      {
        CAN_DecodeStoreWheelLe16(g_CANB_LoopData.ECU.IgbtTempDc, recv_data->msg_data);
      }
      break;

    case CAN_ID_CANB_AMK_STATUS_MODE_LOGIC:
      if (recv_data->dlc >= CAN_DECODE_LEN_5)
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

    case CAN_ID_IVT_CURRENT:
    case CAN_ID_IVT_U1:
    case CAN_ID_IVT_POWER:
    case CAN_ID_IVT_WH:
      /* IVT and FS may reuse these standard IDs, so the helper resolves the
       * currently active source before updating publish-facing caches.
       */
      (void)CAN_DecodeVehicleCanbEnergyResult(recv_data);
      break;

    case CAN_ID_IMU_RAW:
      if (recv_data->dlc >= CAN_DECODE_LEN_6)
      {
        g_CANB_LoopData.IMU.Acc_X_Raw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[0]);
        g_CANB_LoopData.IMU.Acc_Y_Raw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[2]);
        g_CANB_LoopData.IMU.Acc_Z_Raw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[4]);
        IMU_Acc_Calibrate();
        g_CANB_LoopData.IMU.ValidFlags |= CANB_MOTION_VALID_ACCEL;
      }
      break;

    case CAN_ID_IMU_ACCEL:
      if (recv_data->dlc >= CAN_DECODE_LEN_6)
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
      if (recv_data->dlc >= CAN_DECODE_LEN_6)
      {
        /* Only yaw rate is currently consumed by protobuf motion telemetry. */
        g_CANB_LoopData.IMU.YawRateRaw = CAN_DecodeReadLe16Signed(&recv_data->msg_data[4]);
        g_CANB_LoopData.IMU.ValidFlags |= CANB_MOTION_VALID_GYRO;
      }
      break;

    case CAN_ID_IMU_YAW:
      if (recv_data->dlc >= CAN_DECODE_LEN_2)
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

void CAN_DecodeBatteryBoxMessage(const CAN_Msg_Queue_t *recv_data)
{
  /* Decode battery-box frames in the format selected by the bus owner. */
  if (recv_data == NULL || recv_data->can_channel != CAN_CHANNEL_BATTERY_BOX)
  {
    return;
  }

  if (recv_data->is_ext_id == 0U)
  {
    CAN_DecodeBatteryBoxStdMessage(recv_data);
  }
  else
  {
    CAN_DecodeBatteryBoxExtMessage(recv_data);
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
      CAN_DecodeRecalculateMotorPower();
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

    case CAN_ID_CANA_FR_TORQUE_CURRENT_LIMIT_COUNTER:
    case CAN_ID_CANA_FL_TORQUE_CURRENT_LIMIT_COUNTER:
    case CAN_ID_CANA_RR_TORQUE_CURRENT_LIMIT_COUNTER:
    case CAN_ID_CANA_RL_TORQUE_CURRENT_LIMIT_COUNTER:
      g_CANB_LoopData.ECU.TorqueCurrent[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[0]);
      g_CANB_LoopData.ECU.TorqueLimitPositive[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[2]);
      g_CANB_LoopData.ECU.TorqueLimitNegative[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[4]);
      g_CANB_LoopData.ECU.MessageCounter[motor_index] = CAN_DecodeReadLe16(&recv_data->msg_data[6]);
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
  /* Standard battery-box frames carry hall current and a compact status byte. */
  if (recv_data == NULL || recv_data->is_ext_id != 0U)
  {
    return;
  }

  if (recv_data->msg_id == CAN_ID_BATTERY_HALL_CURRENT && recv_data->dlc >= CAN_DECODE_LEN_5)
  {
    uint32_t raw_current = CAN_DecodeReadBe32(&recv_data->msg_data[0]);
    g_BatteryInfo.HallCurrent = (int32_t)(raw_current - 0x80000000UL);
    g_BatteryInfo.HallError = (uint8_t)(recv_data->msg_data[4] & 0x01U);
    g_BatteryInfo.HallErrorCode = (uint8_t)(recv_data->msg_data[4] >> 1);
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
      (recv_data->dlc >= CAN_DECODE_LEN_8))
  {
    uint32_t frame_no = (recv_data->msg_id - CAN_ID_BATTERY_CELL_VOLTAGE_BASE) >> 16;
    uint8_t module_id = (uint8_t)(frame_no / CAN_BATTERY_CELL_VOLTAGE_FRAMES_PER_MODULE);
    uint8_t frame_in_module = (uint8_t)(frame_no % CAN_BATTERY_CELL_VOLTAGE_FRAMES_PER_MODULE);

    if (module_id < CAN_BATTERY_MODULE_COUNT)
    {
      if (frame_in_module == 0U)
      {
        for (uint8_t i = 0U; i < 3U; i++)
        {
          g_BatteryInfo.CellVolt[module_id][i] = CAN_DecodeReadLe16(&recv_data->msg_data[2U + (2U * i)]);
        }
      }
      else
      {
        uint8_t cell_base = (uint8_t)(3U + ((frame_in_module - 1U) * 4U));
        for (uint8_t i = 0U; i < 4U; i++)
        {
          g_BatteryInfo.CellVolt[module_id][cell_base + i] = CAN_DecodeReadLe16(&recv_data->msg_data[2U * i]);
        }
      }
    }
    return;
  }

  if ((recv_data->msg_id >= CAN_ID_BATTERY_CELL_TEMP_BASE) &&
      (recv_data->msg_id <= CAN_ID_BATTERY_CELL_TEMP_LAST) &&
      (((recv_data->msg_id - CAN_ID_BATTERY_CELL_TEMP_BASE) & 0xFFFFUL) == 0UL) &&
      (recv_data->dlc >= CAN_DECODE_LEN_8))
  {
    uint8_t module_id = (uint8_t)((recv_data->msg_id - CAN_ID_BATTERY_CELL_TEMP_BASE) >> 16);

    if (module_id < CAN_BATTERY_MODULE_COUNT)
    {
      for (uint8_t i = 0U; i < BAT_TEMP_POINT_PER_MOD; i++)
      {
        g_BatteryInfo.ModTemp[module_id][i] = (int8_t)(recv_data->msg_data[i] - BAT_TEMP_OFFSET);
      }
    }
    return;
  }

  switch (recv_data->msg_id)
  {
    case CAN_ID_BATTERY_PACK_SUMMARY:
      if (recv_data->dlc >= CAN_DECODE_LEN_7)
      {
        uint16_t pack_voltage_deci_v = CAN_DecodeReadBe16(&recv_data->msg_data[0]);
        uint16_t pack_current_raw = CAN_DecodeReadBe16(&recv_data->msg_data[2]);
        g_BatteryInfo.TotalVolt = (uint32_t)pack_voltage_deci_v * 100U;
        g_BatteryInfo.TotalCurrent = ((int32_t)pack_current_raw - 10000) * 100;
        g_BatteryInfo.SOC = recv_data->msg_data[4];
        g_BatteryInfo.IMD_State = recv_data->msg_data[5];
        g_BatteryInfo.Battery_State = (uint8_t)((recv_data->msg_data[6] >> 4) & 0x0FU);
        g_BatteryInfo.Battery_AlarmLevel = (uint8_t)(recv_data->msg_data[6] & 0x0FU);
        g_BatteryInfo.Work_State = g_BatteryInfo.Battery_State;
      }
      break;

    case CAN_ID_BATTERY_CELL_EXTREMA:
      if (recv_data->dlc >= CAN_DECODE_LEN_6)
      {
        g_BatteryInfo.MaxCellVolt = CAN_DecodeReadBe16(&recv_data->msg_data[0]);
        g_BatteryInfo.MinCellVolt = CAN_DecodeReadBe16(&recv_data->msg_data[2]);
        g_BatteryInfo.MaxCellVoltNo = recv_data->msg_data[4];
        g_BatteryInfo.MinCellVoltNo = recv_data->msg_data[5];
      }
      break;

    case CAN_ID_BATTERY_TEMP_EXTREMA:
      if (recv_data->dlc >= CAN_DECODE_LEN_4)
      {
        g_BatteryInfo.MaxTemp = (int8_t)(recv_data->msg_data[0] - BAT_TEMP_OFFSET);
        g_BatteryInfo.MinTemp = (int8_t)(recv_data->msg_data[1] - BAT_TEMP_OFFSET);
        g_BatteryInfo.MaxTempNo = recv_data->msg_data[2];
        g_BatteryInfo.MinTempNo = recv_data->msg_data[3];
      }
      break;

    case CAN_ID_BATTERY_STATUS:
      if (recv_data->dlc >= CAN_DECODE_LEN_8)
      {
        g_BatteryInfo.Air1_State = (uint8_t)((recv_data->msg_data[0] >> 6) & 0x03U);
        g_BatteryInfo.Air2_State = (uint8_t)((recv_data->msg_data[0] >> 4) & 0x03U);
        g_BatteryInfo.Air3_State = (uint8_t)((recv_data->msg_data[0] >> 2) & 0x03U);
        g_BatteryInfo.Charge_Enable = (uint8_t)((recv_data->msg_data[1] >> 4) & 0x0FU);
        g_BatteryInfo.Charge_ReqVolt = CAN_DecodeReadBe16(&recv_data->msg_data[2]);
        g_BatteryInfo.Charge_ReqCurr = CAN_DecodeReadBe16(&recv_data->msg_data[4]);
        g_BatteryInfo.PrechargeVolt = CAN_DecodeReadBe16(&recv_data->msg_data[6]);
        Battery_Charger_Calibrate();
      }
      break;

    case CAN_ID_BATTERY_CELL_SUM:
      if (recv_data->dlc >= CAN_DECODE_LEN_2)
      {
        g_BatteryInfo.CellVoltSum = CAN_DecodeReadBe16(&recv_data->msg_data[0]);
      }
      break;

    case CAN_ID_BATTERY_ALARM_STATUS:
      if (recv_data->dlc >= CAN_DECODE_LEN_4)
      {
        g_BatteryInfo.BatteryFaultCode = CAN_DecodeBuildBatteryFaultCode(recv_data->msg_data);
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
