exec(open('test_and_fix/migrations/update_can_protocol.py', encoding='utf-8').read().split("p='Core/Inc/can.h'")[0])
p='Core/Src/can_decode.c'; s=read(p)
s+='''
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
'''
write(p,s)
p='Core/Src/freertos.c'; s=read(p)
# Service freshness while idle; a queue-driven receiver must not sleep per frame.
for func,nextfunc,service in [('CanForViehcleTask','CanForBMSTask','CAN_DecodeVehicleService'),('CanForBMSTask','MLX90640Task','CAN_DecodeBatteryService')]:
    a=s.index('void '+func+'(void *argument)\n{'); b=s.index('void '+nextfunc+'(void *argument)\n{',a)
    part=s[a:b].replace('NULL, osWaitForever)', 'NULL, pdMS_TO_TICKS(20U))').replace('    osDelay(200);',f'    {service}(HAL_GetTick());')
    s=s[:a]+part+s[b:]
s=s.replace('  if(pData->can_channel == CAN_CHANNEL_VEHICLE_CANB)\n  {', '''  if(CAN_IsBatteryCanbMessage(pData))
  {
    queue_handle = CAN_Msg_Queue_2Handle;
  }
  else if(pData->can_channel == CAN_CHANNEL_VEHICLE_CANB)
  {''')
write(p,s)
p='Core/Src/publish.c'; s=read(p)
s=s.replace('g_CANB_LoopData.IVT','g_BatteryInfo.IVT').replace('CANB_IVT_VALID_','BATTERY_IVT_VALID_')
a=s.index('    if (source == CANB_ENERGY_METER_SOURCE_IVT)'); b=s.index('    /* FS source reads',a)
s=s[:a]+s[b:]
# Power is device-reported, not reconstructed from independent U/I samples.
s=re.sub(r'    else if \(\(valid_flags & \(BATTERY_IVT_VALID_CURRENT.*?\n    }\n', '', s, flags=re.S)
s=re.sub(r'    else if \(\(valid_flags & \(CANB_ENERGY_VALID_CURRENT.*?\n    }\n', '', s, flags=re.S)
s=s.replace('/* Present one stable energy-meter protobuf regardless of physical source. */','/* Competition-only cache. The team IVT belongs to g_BatteryInfo. */')
s=s.replace('/* Export the legacy IVT cache into its protobuf representation. */','/* Adapt the CAN1 battery IVT cache to the existing protobuf schema. */')
s=s.replace('        energy_meter->msg_counter = (uint32_t)g_CANB_LoopData.EnergyMeter.MsgCounter;\n','')
s=s.replace('    energy_meter->source = (uint32_t)source;', '    energy_meter->source = (uint32_t)source;\n    energy_meter->msg_counter = g_CANB_LoopData.EnergyMeter.MsgCounter;')
s=s.replace('    motor_state->power_w = (int32_t)(g_CANB_LoopData.ECU.Motor_Power[motor_index] * 1000.0f);', '    motor_state->power_w = (int32_t)((g_CANB_LoopData.ECU.MotorPowerW[motor_index] > INT32_MAX) ? INT32_MAX : g_CANB_LoopData.ECU.MotorPowerW[motor_index]);')
write(p,s)
