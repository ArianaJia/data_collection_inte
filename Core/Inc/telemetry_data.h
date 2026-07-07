#ifndef __TELEMETRY_DATA_H
#define __TELEMETRY_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CHARGER_VOLT_SCALE     0.1f
#define CHARGER_CURR_SCALE     0.1f
#define IMU_ACC_SCALE          (16.0f / 32768.0f)
#define BAT_TEMP_POINT_PER_MOD 8
#define BAT_TEMP_OFFSET        30
#define MLX90640_SENSOR_COUNT  4U
#define MLX90640_REGION_COUNT  4U
#define MLX90640_TEMP_SCALE    100L

/* Motion valid bits for CANB-derived GPS/IMU data. */
#define CANB_MOTION_VALID_GPS  (1U << 0)
#define CANB_MOTION_VALID_ACCEL (1U << 1)
#define CANB_MOTION_VALID_GYRO (1U << 2)
#define CANB_MOTION_VALID_YAW  (1U << 3)

/* IVT valid bits: private team energy meter values decoded from CANB. */
#define CANB_IVT_VALID_CURRENT (1U << 0)
#define CANB_IVT_VALID_U1      (1U << 1)
#define CANB_IVT_VALID_POWER   (1U << 2)
#define CANB_IVT_VALID_ENERGY  (1U << 3)

/* Unified energy meter valid bits exposed to publish/protobuf. */
#define CANB_ENERGY_VALID_CURRENT (1U << 0)
#define CANB_ENERGY_VALID_VOLTAGE (1U << 1)
#define CANB_ENERGY_VALID_POWER   (1U << 2)
#define CANB_ENERGY_VALID_ENERGY  (1U << 3)
#define CANB_ENERGY_VALID_STATUS  (1U << 4)

/* Energy meter source tags.
 * IVT = team-owned energy meter.
 * FS  = competition/Formula Student logger-side energy meter data.
 */
#define CANB_ENERGY_METER_SOURCE_UNKNOWN 0U
#define CANB_ENERGY_METER_SOURCE_IVT     1U
#define CANB_ENERGY_METER_SOURCE_FS      2U

/* IVT payload endianness may differ between devices/bridges, so we remember
 * the most recently plausible byte order after decoding.
 */
#define CANB_IVT_BYTE_ORDER_UNKNOWN      0U
#define CANB_IVT_BYTE_ORDER_BE           1U
#define CANB_IVT_BYTE_ORDER_LE           2U

/* FS status bits reconstructed from the CANB status frame. */
#define CANB_FS_STATUS_READY_BIT         (1UL << 0)
#define CANB_FS_STATUS_LOGGING_BIT       (1UL << 1)
#define CANB_FS_STATUS_TRIG_VOLTAGE_BIT  (1UL << 2)
#define CANB_FS_STATUS_TRIG_CURRENT_BIT  (1UL << 3)

typedef struct
{
    uint16_t CellVolt[6][24];
    int8_t   ModTemp[6][8];
    uint32_t TotalVolt;
    int32_t  TotalCurrent;
    int8_t   MaxTemp;
    int8_t   MinTemp;
    uint16_t MaxCellVolt;
    uint16_t MinCellVolt;
    uint8_t  Air1_State;
    uint8_t  Air2_State;
    uint8_t  Air3_State;
    uint8_t  Charge_Enable;
    uint8_t  IMD_State;
    uint8_t  Battery_State;
    uint8_t  Battery_AlarmLevel;
    uint8_t  MaxCellVoltNo;
    uint8_t  MinCellVoltNo;
    uint8_t  MaxTempNo;
    uint8_t  MinTempNo;
    uint16_t CellVoltSum;
    uint16_t PrechargeVolt;
    uint32_t BatteryFaultCode;
    int32_t  HallCurrent;
    uint8_t  HallError;
    uint8_t  HallErrorCode;
    uint8_t  SOC;
    uint16_t Charge_ReqVolt;
    uint16_t Charge_ReqCurr;
    uint8_t  Charger_Cmd;
    uint16_t Charger_OutVolt;
    int16_t  Charger_OutCurr;
    uint8_t  Work_State;
    float    Charger_ReqVolt_Act;
    float    Charger_ReqCurr_Act;
    float    Charger_OutVolt_Act;
    float    Charger_OutCurr_Act;
} Battery_InfoDef;

typedef struct
{
    uint8_t  Vehicle_Speed;
    float    Motor_Power[4];
    float    Total_Power;
    int8_t   driving_mode;
    int16_t  Motor_RPM[4];
    int16_t  Motor_Torque[4];
    int16_t  MotorTempDc[4];
    int16_t  InverterTempDc[4];
    int16_t  IgbtTempDc[4];
    int16_t  MagnetizingCurrent[4];
    int16_t  TorqueCurrent[4];
    int16_t  TorqueLimitPositive[4];
    int16_t  TorqueLimitNegative[4];
    uint16_t DCBusVoltage[4];
    uint16_t MessageCounter[4];
    uint32_t DiagnosticNumber[4];
    uint32_t ErrorInfo[4];
    uint8_t  LogicState[4];
    uint8_t  StatusBits[4];
    int16_t  SteeringAngleDeciDeg;
    uint16_t ThrottlePositionDeciPct;
    uint16_t OilPressureMilliKpa;
    int      ERRO[4];
} CANB_ECU_VehicleTypeDef;

typedef struct
{
    int16_t  Acc_X_Raw;
    int16_t  Acc_Y_Raw;
    int16_t  Acc_Z_Raw;
    float    Acc_X_Act;
    float    Acc_Y_Act;
    float    Acc_Z_Act;
    float    Long_Accel;
    float    Lat_Accel;
    uint16_t GpsSpeedKmh;
    int16_t  YawRateRaw;
    int16_t  YawRaw;
    uint8_t  ValidFlags;
} CANB_IMU_TypeDef;

typedef struct
{
    CANB_ECU_VehicleTypeDef ECU;
    CANB_IMU_TypeDef        IMU;
    /* Raw IVT cache.
     * These fields represent the private team energy meter messages directly,
     * before they are folded into the generic EnergyMeter view.
     */
    struct
    {
        int32_t  CurrentMa;
        int32_t  VoltageU1Mv;
        int32_t  EnergyWh;
        int32_t  PowerW;
        uint32_t CurrentState;
        uint32_t VoltageU1State;
        uint32_t EnergyState;
        uint32_t PowerState;
        uint8_t  ValidFlags;
        uint8_t  ByteOrder;
    } IVT;
    /* Unified energy meter cache for publish.
     * This layer allows protobuf output to stay stable even if the physical
     * source is IVT or FS.
     */
    struct
    {
        int32_t  CurrentMa;
        int32_t  VoltageMv;
        int32_t  EnergyWh;
        int32_t  PowerW;
        uint32_t Status;
        uint8_t  MsgCounter;
        uint8_t  ValidFlags;
        uint8_t  AutoSource;
    } EnergyMeter;
} CANB_LoopAllData;

typedef struct
{
    int32_t  RegionTemp[MLX90640_SENSOR_COUNT][MLX90640_REGION_COUNT];
    int32_t  Ta[MLX90640_SENSOR_COUNT];
    int32_t  Tr[MLX90640_SENSOR_COUNT];
    int32_t  LastError[MLX90640_SENSOR_COUNT];
    uint32_t FrameCounter[MLX90640_SENSOR_COUNT];
    uint8_t  ValidMask;
    uint8_t  ActiveSensorId;
} MLX90640_FrameTypeDef;

typedef struct
{
    /* Generic CDC monitor cache populated by SPI3 polling.
     * This is intentionally protocol-agnostic for now: the bus is treated as a
     * read-only monitoring path until CDC mailbox semantics are finalized.
     */
    uint32_t FrameCounter;
    uint32_t LastMsgId;
    uint32_t LastError;
    uint8_t  LastData[8];
    uint8_t  LastDlc;
    uint8_t  LastIsExtId;
    uint8_t  Valid;
} CDC_InfoDef;

extern Battery_InfoDef g_BatteryInfo;
extern CANB_LoopAllData g_CANB_LoopData;
extern MLX90640_FrameTypeDef g_MLX90640_Frame;
extern CDC_InfoDef g_CDCInfo;

void Battery_Charger_Calibrate(void);
void IMU_Acc_Calibrate(void);
void ECU_TotalPower_Calc(void);
void Battery_Data_Clear(void);
void CANB_Loop_Data_Clear(void);
void CDC_Data_Clear(void);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_DATA_H */
