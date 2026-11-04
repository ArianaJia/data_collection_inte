exec(open('test_and_fix/migrations/update_can_protocol.py', encoding='utf-8').read().split("p='Core/Inc/can.h'")[0])
p='Core/Src/can_decode.c'
s=read(p)
a=s.index('static void CAN_DecodeRecalculateMotorPower(void);'); b=s.index('static void CAN_DecodeBatteryBoxStdMessage',a)
s=s[:a]+'''static uint8_t CAN_DecodeCompetitionResult(const CAN_Msg_Queue_t *recv_data);
''' +s[b:]
a=s.index('static void CAN_DecodeRecalculateMotorPower(void)\n'); b=s.index('static uint8_t CAN_DecodeControllerMotorIndex(uint32_t msg_id, uint8_t *motor_index)\n',a)
s=s[:a]+'''/* Sources are identified by physical bus and ID, never numeric plausibility. */
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

''' +s[b:]
for old,new in [('CAN_ID_IVT_CURRENT','CAN_ID_COMPETITION_CURRENT'),('CAN_ID_IVT_U1','CAN_ID_COMPETITION_U1'),('CAN_ID_IVT_POWER','CAN_ID_COMPETITION_POWER'),('CAN_ID_IVT_WH','CAN_ID_COMPETITION_WH')]: s=s.replace(old,new)
s=s.replace('CAN_DecodeVehicleCanbEnergyResult(recv_data)','CAN_DecodeCompetitionResult(recv_data)')
s=s.replace('      /* IVT and FS may reuse these standard IDs, so the helper resolves the\n       * currently active source before updating publish-facing caches.\n       */','      /* Competition results on CANB always carry a big-endian int32. */')
s=s.replace('      CAN_DecodeRecalculateMotorPower();\n','')
s=s.replace('recv_data->is_ext_id != 0U)\n  {\n    return;\n  }\n\n  switch (recv_data->msg_id)', 'recv_data->is_ext_id != 0U || recv_data->dlc > 8U)\n  {\n    return;\n  }\n\n  switch (recv_data->msg_id)',1)
s=s.replace('        g_CANB_LoopData.IMU.GpsSpeedKmh = CAN_DecodeReadLe16(recv_data->msg_data);', '''        uint16_t speed = CAN_DecodeReadLe16(recv_data->msg_data);
        if (speed > 3000U) { g_CANB_LoopData.IMU.ValidFlags &= (uint8_t)~CANB_MOTION_VALID_GPS; break; }
        g_CANB_LoopData.IMU.GpsSpeedDeciKmh = speed;
        g_CANB_LoopData.IMU.GpsSpeedKmh = (uint16_t)((speed + 5U) / 10U);''')
s=s.replace('        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);\n','')
a=s.index('    case CAN_ID_CANB_LEGACY_VEHICLE_STATE:'); b=s.index('    case CAN_ID_CANB_WHEEL_ACTUAL_TORQUE:',a)
s=s[:a]+s[b:]
a=s.index('    case CAN_ID_IMU_RAW:'); b=s.index('    case CAN_ID_IMU_ACCEL:',a)
s=s[:a]+'''    case CAN_ID_IMU_RAW:
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

''' +s[b:]
# Retain the FS status message and its counter independently of result states.
s=s.replace('        g_CANB_LoopData.EnergyMeter.MsgCounter = recv_data->msg_data[0];','''        g_CANB_LoopData.EnergyMeter.MsgCounter = recv_data->msg_data[0];
        g_CANB_LoopData.EnergyMeter.ReceivedFlags |= CANB_ENERGY_VALID_STATUS;
        g_CANB_LoopData.EnergyMeter.LastUpdateMs[4] = HAL_GetTick();
        g_CANB_LoopData.EnergyMeter.LastUpdateMs[0] = HAL_GetTick();
        g_CANB_LoopData.EnergyMeter.LastUpdateMs[1] = HAL_GetTick();''')
# BMS mirrors are processed by the battery task, retaining physical channel metadata.
a=s.index('void CAN_DecodeBatteryBoxMessage('); b=s.index('void CAN_DecodeControllerCanaMessage(',a)
s=s[:a]+'''uint8_t CAN_IsBatteryCanbMessage(const CAN_Msg_Queue_t *frame)
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

''' +s[b:]
for name in ['FR','FL','RR','RL']: s=s.replace(f'CAN_ID_CANA_{name}_TORQUE_CURRENT_LIMIT_COUNTER',f'CAN_ID_CANA_{name}_TORQUE_LIMIT_POWER')
s=s.replace('      g_CANB_LoopData.ECU.TorqueCurrent[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[0]);\n      g_CANB_LoopData.ECU.TorqueLimitPositive[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[2]);\n      g_CANB_LoopData.ECU.TorqueLimitNegative[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[4]);\n      g_CANB_LoopData.ECU.MessageCounter[motor_index] = CAN_DecodeReadLe16(&recv_data->msg_data[6]);', '''      g_CANB_LoopData.ECU.TorqueLimitPositive[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[0]);
      g_CANB_LoopData.ECU.TorqueLimitNegative[motor_index] = CAN_DecodeReadLe16Signed(&recv_data->msg_data[2]);
      g_CANB_LoopData.ECU.MotorPowerW[motor_index] = CAN_DecodeReadLe32(&recv_data->msg_data[4]);
      g_CANB_LoopData.ECU.Motor_Power[motor_index] = (float)g_CANB_LoopData.ECU.MotorPowerW[motor_index] / 1000.0f;
      ECU_TotalPower_Calc();''')
a=s.index('  /* Standard battery-box frames'); b=s.index('\n}\n',a)
s=s[:a]+'''  if ((recv_data != NULL) && (recv_data->is_ext_id == 0U) &&
      (recv_data->msg_id >= CAN_ID_BATTERY_IVT_CURRENT) &&
      (recv_data->msg_id <= CAN_ID_BATTERY_IVT_WH))
  {
    CAN_DecodeBatteryIvt(recv_data);
  }''' +s[b:]
# Validity follows individual samples, including missing/invalid sentinel values.
s=s.replace('    if (module_id < CAN_BATTERY_MODULE_COUNT)\n    {\n      if (frame_in_module', '''    if (module_id < CAN_BATTERY_MODULE_COUNT)
    {
      g_BatteryInfo.CellFrameTick[frame_no] = HAL_GetTick();
      g_BatteryInfo.CellFramesReceived |= (1ULL << frame_no);
      if (frame_in_module''',1)
for expr,offset in [('i','2U + (2U * i)'),('cell_base + i','2U * i')]:
    old=f'          g_BatteryInfo.CellVolt[module_id][{expr}] = CAN_DecodeReadLe16(&recv_data->msg_data[{offset}]);'
    new=f'''          uint16_t value = CAN_DecodeReadLe16(&recv_data->msg_data[{offset}]);
          uint32_t bit = 1UL << ({expr});
          g_BatteryInfo.CellValidMask[module_id] &= ~bit;
          g_BatteryInfo.CellVolt[module_id][{expr}] = 0U;
          if ((value >= 500U) && (value <= 5000U))
          {{
            g_BatteryInfo.CellVolt[module_id][{expr}] = value;
            g_BatteryInfo.CellValidMask[module_id] |= bit;
          }}'''
    assert old in s
    s=s.replace(old,new)
s=s.replace('      for (uint8_t i = 0U; i < BAT_TEMP_POINT_PER_MOD; i++)\n      {\n        g_BatteryInfo.ModTemp[module_id][i] = (int8_t)(recv_data->msg_data[i] - BAT_TEMP_OFFSET);\n      }', '''      g_BatteryInfo.TempFrameTick[module_id] = HAL_GetTick();
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
      }''')
s=s.replace('        uint16_t pack_current_raw = CAN_DecodeReadBe16(&recv_data->msg_data[2]);\n        g_BatteryInfo.TotalVolt = (uint32_t)pack_voltage_deci_v * 100U;\n        g_BatteryInfo.TotalCurrent = ((int32_t)pack_current_raw - 10000) * 100;\n        g_BatteryInfo.SOC = recv_data->msg_data[4];\n        g_BatteryInfo.IMD_State = recv_data->msg_data[5];', '''        int16_t pack_current_raw = (int16_t)CAN_DecodeReadBe16(&recv_data->msg_data[2]);
        uint8_t flags = recv_data->msg_data[5] & 0x1FU;
        if (recv_data->msg_data[4] > 100U) flags &= (uint8_t)~BATTERY_SOC_VALID;
        g_BatteryInfo.PackValidFlags = flags;
        g_BatteryInfo.PackReceived = 1U;
        g_BatteryInfo.PackLastUpdateMs = HAL_GetTick();
        g_BatteryInfo.TotalVolt = (flags & BATTERY_PACK_VOLT_VALID) ? (uint32_t)pack_voltage_deci_v * 100U : 0U;
        g_BatteryInfo.TotalCurrent = (flags & BATTERY_PACK_CURR_VALID) ? (int32_t)pack_current_raw * 100 : 0;
        g_BatteryInfo.SOC = (flags & BATTERY_SOC_VALID) ? recv_data->msg_data[4] : 0U;''')
s=s.replace('        g_BatteryInfo.MaxCellVolt = CAN_DecodeReadBe16', '        g_BatteryInfo.CellExtremaTick = HAL_GetTick();\n        g_BatteryInfo.CellExtremaValid = 1U;\n        g_BatteryInfo.MaxCellVolt = CAN_DecodeReadBe16')
s=s.replace('        g_BatteryInfo.MinCellVoltNo = recv_data->msg_data[5];','''        g_BatteryInfo.MinCellVoltNo = recv_data->msg_data[5];
        if ((g_BatteryInfo.MaxCellVolt == 0xFFFFU) || (g_BatteryInfo.MinCellVolt == 0xFFFFU) ||
            (g_BatteryInfo.MaxCellVoltNo >= 138U) || (g_BatteryInfo.MinCellVoltNo >= 138U))
        {
          g_BatteryInfo.CellExtremaValid = 0U;
          g_BatteryInfo.MaxCellVolt = g_BatteryInfo.MinCellVolt = 0U;
        }''')
s=s.replace('    case CAN_ID_BATTERY_TEMP_EXTREMA:\n      if (recv_data->dlc >= CAN_DECODE_LEN_4)', '    case CAN_ID_BATTERY_TEMP_EXTREMA:\n      if (recv_data->dlc == CAN_DECODE_LEN_8)')
s=s.replace('        g_BatteryInfo.MaxTemp = (int8_t)(recv_data->msg_data[0] - BAT_TEMP_OFFSET);\n        g_BatteryInfo.MinTemp = (int8_t)(recv_data->msg_data[1] - BAT_TEMP_OFFSET);','''        g_BatteryInfo.TempExtremaTick = HAL_GetTick();
        g_BatteryInfo.TempExtremaValid = (recv_data->msg_data[0] <= 129U) && (recv_data->msg_data[1] <= 129U) &&
            (recv_data->msg_data[2] < 48U) && (recv_data->msg_data[3] < 48U);
        g_BatteryInfo.MaxTemp = g_BatteryInfo.TempExtremaValid ? (int8_t)(recv_data->msg_data[0] - BAT_TEMP_OFFSET) : 0;
        g_BatteryInfo.MinTemp = g_BatteryInfo.TempExtremaValid ? (int8_t)(recv_data->msg_data[1] - BAT_TEMP_OFFSET) : 0;
        g_BatteryInfo.CoolingControl = recv_data->msg_data[4];
        g_BatteryInfo.FanTargetDuty = recv_data->msg_data[5];
        g_BatteryInfo.FanSpeedRpm = (uint16_t)recv_data->msg_data[6] * 100U;
        g_BatteryInfo.FanStatusFlags = recv_data->msg_data[7];''')
s=s.replace('(uint8_t)((recv_data->msg_data[1] >> 4) & 0x0FU);','(uint8_t)((recv_data->msg_data[1] >> 4) & 0x01U);\n        g_BatteryInfo.ChargeCommValid = (recv_data->msg_data[1] >> 3) & 1U;')
s=s.replace('        g_BatteryInfo.CellVoltSum = CAN_DecodeReadBe16(&recv_data->msg_data[0]);','''        g_BatteryInfo.CellSumTick = HAL_GetTick();
        g_BatteryInfo.CellVoltSum = CAN_DecodeReadBe16(&recv_data->msg_data[0]);
        g_BatteryInfo.CellSumValid = g_BatteryInfo.CellVoltSum != 0xFFFFU;
        if (!g_BatteryInfo.CellSumValid) g_BatteryInfo.CellVoltSum = 0U;''')
s=s.replace('    case CAN_ID_BATTERY_ALARM_STATUS:\n      if (recv_data->dlc >= CAN_DECODE_LEN_4)\n      {\n        g_BatteryInfo.BatteryFaultCode = CAN_DecodeBuildBatteryFaultCode(recv_data->msg_data);\n      }\n      break;', '''    case CAN_ID_BATTERY_ALARM_STATUS:
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
      break;''')
# Exact DLC for the supported classic protocol messages (not oversized aliases).
s=s.replace('recv_data->dlc >= CAN_DECODE_LEN_', 'recv_data->dlc == CAN_DECODE_LEN_')
# Preserve the existing charger decoder unchanged: no charger protocol migration in this task.
s=s.replace('case CAN_ID_BATTERY_CHARGER_FEEDBACK:\n      if (recv_data->dlc == CAN_DECODE_LEN_5)', 'case CAN_ID_BATTERY_CHARGER_FEEDBACK:\n      if (recv_data->dlc >= CAN_DECODE_LEN_5)')
write(p,s)
p='Core/Inc/telemetry_data.h'; s=read(p).replace('    float    Motor_Power[4];', '    float    Motor_Power[4]; /* Compatibility kW view of ActualValue4. */\n    uint32_t MotorPowerW[4]; /* ActualValue4 raw value is W. */'); write(p,s)
p='Core/Src/fdcan.c'; s=read(p)
for name in ['FR','FL','RR','RL']: s=s.replace(f'CAN_ID_CANA_{name}_TORQUE_CURRENT_LIMIT_COUNTER',f'CAN_ID_CANA_{name}_TORQUE_LIMIT_POWER')
write(p,s)
p='Core/Inc/can_decode.h'; s=read(p).replace('void CAN_DecodeVehicleCanbMessage', 'uint8_t CAN_IsBatteryCanbMessage(const CAN_Msg_Queue_t *frame);\nvoid CAN_DecodeBatteryService(uint32_t now);\nvoid CAN_DecodeVehicleService(uint32_t now);\nvoid CAN_DecodeVehicleCanbMessage'); write(p,s)
