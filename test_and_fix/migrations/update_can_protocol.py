from pathlib import Path
import re

def read(name):
    return Path(name).read_text(encoding='utf-8-sig')

def write(name, text):
    Path(name).write_bytes(text.replace('\r\n', '\n').replace('\n', '\r\n').encode('utf-8'))

p='Core/Inc/can.h'
s=read(p)
s=s.replace('0x0401U','0x04B0U').replace('0x0402U','0x04B1U')
s=s.replace('#define CAN_ID_CANB_ENERGY_METER_STATUS', '#define CAN_ID_CANB_BATTERY_ALARM_LEVELS  0x04B2U\n#define CAN_ID_CANB_ENERGY_METER_STATUS')
s=s.replace('#define CAN_ID_BATTERY_ALARM_THRESHOLD', '#define CAN_ID_BATTERY_ALARM_LEVELS       0x187850F4UL\n#define CAN_ID_BATTERY_HV_STATUS          0x186950F4UL\n#define CAN_ID_BATTERY_SOP_MIRROR         0x186A50F4UL\n#define CAN_ID_BATTERY_ALARM_THRESHOLD')
for old,new in [('CAN_ID_IVT_CURRENT','CAN_ID_COMPETITION_CURRENT'),('CAN_ID_IVT_U1','CAN_ID_COMPETITION_U1'),('CAN_ID_IVT_POWER','CAN_ID_COMPETITION_POWER'),('CAN_ID_IVT_WH','CAN_ID_COMPETITION_WH')]: s=s.replace(old,new)
s=s.replace('/************************** Vehicle CANB mailbox IDs', '''/* Team-owned IVT: CAN1 battery-box bus, DLC 6, little-endian int32. */
#define CAN_ID_BATTERY_IVT_CURRENT       0x0512U
#define CAN_ID_BATTERY_IVT_U1            0x0513U
#define CAN_ID_BATTERY_IVT_U2            0x0514U
#define CAN_ID_BATTERY_IVT_U3            0x0515U
#define CAN_ID_BATTERY_IVT_TEMPERATURE   0x0516U
#define CAN_ID_BATTERY_IVT_POWER         0x0517U
#define CAN_ID_BATTERY_IVT_AS            0x0518U
#define CAN_ID_BATTERY_IVT_WH            0x0519U

/************************** Vehicle CANB mailbox IDs''')
for name in ['FR','FL','RR','RL']:
    s=s.replace(f'CAN_ID_CANA_{name}_TORQUE_CURRENT_LIMIT_COUNTER',f'CAN_ID_CANA_{name}_TORQUE_LIMIT_POWER')
write(p,s)

p='Core/Inc/telemetry_data.h'
s=read(p)
a=s.index('/* IVT valid bits:'); b=s.index('/* FS status bits',a)
s=s[:a]+'''/* IVT flags follow CAN1 MUX 0..7. Received and usable are distinct. */
#define BATTERY_IVT_VALID_CURRENT (1U << 0)
#define BATTERY_IVT_VALID_U1      (1U << 1)
#define BATTERY_IVT_VALID_U2      (1U << 2)
#define BATTERY_IVT_VALID_U3      (1U << 3)
#define BATTERY_IVT_VALID_TEMP    (1U << 4)
#define BATTERY_IVT_VALID_POWER   (1U << 5)
#define BATTERY_IVT_VALID_AS      (1U << 6)
#define BATTERY_IVT_VALID_ENERGY  (1U << 7)
#define BATTERY_IVT_TIMEOUT_MS   360U
#define BATTERY_SAMPLE_TIMEOUT_MS 350U
#define BATTERY_STATUS_TIMEOUT_MS 2000U
#define BATTERY_PACK_VOLT_VALID  (1U << 0)
#define BATTERY_PACK_CURR_VALID  (1U << 1)
#define BATTERY_SOC_VALID        (1U << 2)

#define CANB_ENERGY_VALID_CURRENT (1U << 0)
#define CANB_ENERGY_VALID_VOLTAGE (1U << 1)
#define CANB_ENERGY_VALID_POWER   (1U << 2)
#define CANB_ENERGY_VALID_ENERGY  (1U << 3)
#define CANB_ENERGY_VALID_STATUS  (1U << 4)
#define CANB_ENERGY_METER_SOURCE_UNKNOWN 0U
#define CANB_ENERGY_METER_SOURCE_COMPETITION_IVT 1U
#define CANB_ENERGY_METER_SOURCE_FS      2U

typedef struct
{
    int32_t CurrentMa, VoltageU1Mv, VoltageU2Mv, VoltageU3Mv;
    int32_t TemperatureDc, PowerW, ChargeAs, EnergyWh;
    uint8_t CurrentState, VoltageU1State, VoltageU2State, PowerState, EnergyState;
    uint8_t State[8], Counter[8];
    uint32_t LastUpdateMs[8];
    uint8_t ReceivedFlags, ValidFlags;
} Battery_IvtData;

''' +s[b:]
a=s.index('    /* Raw IVT cache.'); b=s.index('    /* Unified energy meter cache',a)
s=s[:a]+s[b:]
s=s.replace('    /* Cell voltage matrix:', '''    /* All battery-domain sources, including CANB BMS mirrors, live here. */
    Battery_IvtData IVT;
    uint8_t PackValidFlags, PackReceived;
    uint32_t PackLastUpdateMs;
    uint32_t CellValidMask[6]; /* Bits 0..22; raw invalid values are not measurements. */
    uint8_t TempValidMask[6];
    uint32_t CellFrameTick[36], TempFrameTick[6];
    uint64_t CellFramesReceived;
    uint8_t TempFramesReceived;
    uint8_t CellExtremaValid, TempExtremaValid, CellSumValid;
    uint32_t CellExtremaTick, TempExtremaTick, CellSumTick;
    uint8_t FaultFlags, SlaveOfflineMask, FaultFrameVersion, FaultValid;
    uint8_t AlarmLevels[32], AlarmLevelsValid;
    uint32_t FaultTick, AlarmLevelsTick;
    uint8_t ChargeCommValid;
    uint8_t CoolingControl, FanTargetDuty, FanStatusFlags;
    uint16_t FanSpeedRpm;
    uint8_t ImdFrequencyClass, ImdFlags, ImdReceived;
    uint16_t ImdDutyDeciPct, ImdResistanceKohm, ImdFrequencyCentiHz;
    uint8_t HvRequestFlags, LastPrechargeResult;
    uint16_t LastPrechargeSuccessMs, LastPrechargeFailureMs;
    uint16_t SopDischargeCurrentDa, SopChargeCurrentDa;
    uint16_t SopDischargePowerDkw, SopChargePowerDkw;
    /* Cell voltage matrix:''')
s=s.replace('Pack total voltage in the raw integer unit used by the BMS frame.','Pack total voltage in mV; zero when invalid.')
s=s.replace('Pack total current in the raw integer unit used by the BMS frame.','Pack current in mA; positive = charging, negative = discharging.')
s=s.replace('    uint16_t GpsSpeedKmh;', '    uint16_t GpsSpeedDeciKmh; /* CAN raw unit: 0.1 km/h. */\n    uint16_t GpsSpeedKmh; /* Rounded compatibility view for the existing protobuf. */')
s=s.replace('        uint32_t Status;', '        uint32_t Status; /* Only the FS 0x430 status byte, never result state. */\n        uint8_t CurrentState, VoltageState, PowerState, EnergyState;\n        uint8_t ResultCounter[4];\n        uint8_t ReceivedFlags;\n        uint32_t LastUpdateMs[5];')
s=s.replace('IVT or FS.', 'competition IVT or FS.').replace('unknown, IVT, or FS.', 'unknown, competition IVT, or FS.')
write(p,s)

p='Core/Src/can.c'
s=read(p)
s=s.replace('  CAN_ID_CANB_LEGACY_VEHICLE_STATE,\n','')
s=s.replace('  CAN_ID_CANB_BATTERY_FAULT_STATUS,','  CAN_ID_CANB_BATTERY_FAULT_STATUS,\n  CAN_ID_CANB_BATTERY_ALARM_LEVELS,')
for old,new in [('CAN_ID_IVT_CURRENT','CAN_ID_COMPETITION_CURRENT'),('CAN_ID_IVT_U1','CAN_ID_COMPETITION_U1'),('CAN_ID_IVT_POWER','CAN_ID_COMPETITION_POWER'),('CAN_ID_IVT_WH','CAN_ID_COMPETITION_WH')]: s=s.replace(old,new)
a=s.index('static const uint32_t g_can1_battery_filter_ids[]'); b=s.index('static CAN_ControllerContext_t *CAN_GetContext',a)
s=s[:a]+s[b:]
a=s.index('static HAL_StatusTypeDef CAN_ConfigCan1Filters'); b=s.index('static HAL_StatusTypeDef CAN_ConfigCan2Filters',a)
s=s[:a]+'''/* 32-bit bxCAN masks include IDE and RTR. Coarse frame groups are
 * narrowed again by the decoder; bank 0..5 belongs exclusively to CAN1.
 */
static HAL_StatusTypeDef CAN_ConfigCan1Filters(CAN_HandleTypeDef *canHandle)
{
  static const struct { uint32_t id, mask; uint8_t extended; } groups[] = {
    {CAN_ID_BATTERY_CELL_VOLTAGE_BASE, 0x1FC0FFFFUL, 1U},
    {CAN_ID_BATTERY_CELL_TEMP_BASE,    0x1FF8FFFFUL, 1U},
    {CAN_ID_BATTERY_PACK_SUMMARY,      0x1FE0FFFFUL, 1U},
    {CAN_ID_BATTERY_IVT_CURRENT,       0x7FEU, 0U},
    {CAN_ID_BATTERY_IVT_U2,            0x7FCU, 0U},
    {CAN_ID_BATTERY_IVT_AS,            0x7FEU, 0U}
  };
  CAN_FilterTypeDef filter = {0};
  if (canHandle == NULL) return HAL_ERROR;
  filter.FilterActivation = ENABLE;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.SlaveStartFilterBank = 14U;
  for (uint32_t i = 0U; i < sizeof(groups) / sizeof(groups[0]); ++i)
  {
    uint32_t id = groups[i].extended ? (groups[i].id << 3) | CAN_ID_EXT : groups[i].id << 21;
    uint32_t mask = (groups[i].extended ? groups[i].mask << 3 : groups[i].mask << 21) | 6U;
    filter.FilterBank = i;
    filter.FilterIdHigh = (uint16_t)(id >> 16);
    filter.FilterIdLow = (uint16_t)id;
    filter.FilterMaskIdHigh = (uint16_t)(mask >> 16);
    filter.FilterMaskIdLow = (uint16_t)mask;
    if (HAL_CAN_ConfigFilter(canHandle, &filter) != HAL_OK) return HAL_ERROR;
  }
  return HAL_OK;
}

''' +s[b:]
s=s.replace('  if (canHandle == NULL)\n  {\n    return HAL_ERROR;\n  }\n\n  filter.FilterActivation', '  if ((canHandle == NULL) || (bank_count > 14U))\n  {\n    return HAL_ERROR;\n  }\n\n  filter.FilterActivation')
s=s.replace('  ctx->rx_count++;','  if ((ctx->rx_header.RTR != CAN_RTR_DATA) || (ctx->rx_header.DLC > 8U))\n  {\n    return;\n  }\n\n  ctx->rx_count++;')
s=s.replace('(canHandle->Instance == CAN1) && (CAN_ConfigCan2Filters(canHandle)', '(canHandle->Instance == CAN1) && (CAN_ConfigCan1Filters(canHandle)')
s=s.replace('(canHandle->Instance == CAN2) && (CAN_ConfigCan1Filters(canHandle)', '(canHandle->Instance == CAN2) && (CAN_ConfigCan2Filters(canHandle)')
s=s.replace('void CAN_Filter_Config(CAN_HandleTypeDef *canHandle)\n{','void CAN_Filter_Config(CAN_HandleTypeDef *canHandle)\n{\n  if (canHandle == NULL) { Error_Handler(); return; }')
write(p,s)
