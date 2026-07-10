#ifndef __TELEMETRY_DATA_H
#define __TELEMETRY_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Charger voltage raw value -> volts. */
#define CHARGER_VOLT_SCALE     0.1f
/* Charger current raw value -> amps. */
#define CHARGER_CURR_SCALE     0.1f
/* IMU raw acceleration count -> g. */
#define IMU_ACC_SCALE          (16.0f / 32768.0f)
/* Number of temperature points stored for one battery module. */
#define BAT_TEMP_POINT_PER_MOD 8
/* Battery temperature index offset used by the original sensor mapping. */
#define BAT_TEMP_OFFSET        30
/* Number of MLX90640 sensors on the board. */
#define MLX90640_SENSOR_COUNT  4U
/* Number of averaged temperature regions per MLX90640 sensor. */
#define MLX90640_REGION_COUNT  4U
/* Floating-point Celsius is stored as centi-Celsius in telemetry caches. */
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
    /* Cell voltage matrix: 6 packs/modules, each with 24 cell taps. */
    uint16_t CellVolt[6][24];
    /* Module temperature matrix: 6 packs/modules, each with 8 temperature points. */
    int8_t   ModTemp[6][8];
    /* Pack total voltage in the raw integer unit used by the BMS frame. */
    uint32_t TotalVolt;
    /* Pack total current in the raw integer unit used by the BMS frame. */
    int32_t  TotalCurrent;
    /* Highest module/cell temperature observed in the pack. */
    int8_t   MaxTemp;
    /* Lowest module/cell temperature observed in the pack. */
    int8_t   MinTemp;
    /* Highest single-cell voltage observed in the pack. */
    uint16_t MaxCellVolt;
    /* Lowest single-cell voltage observed in the pack. */
    uint16_t MinCellVolt;
    /* AIR1 relay/contact state. */
    uint8_t  Air1_State;
    /* AIR2 relay/contact state. */
    uint8_t  Air2_State;
    /* AIR3 relay/contact state. */
    uint8_t  Air3_State;
    /* Charger enable request or actual enable state. */
    uint8_t  Charge_Enable;
    /* Insulation monitoring device state. */
    uint8_t  IMD_State;
    /* Overall battery state machine value. */
    uint8_t  Battery_State;
    /* Alarm severity level for the pack. */
    uint8_t  Battery_AlarmLevel;
    /* Index of the hottest cell. */
    uint8_t  MaxCellVoltNo;
    /* Index of the coldest cell. */
    uint8_t  MinCellVoltNo;
    /* Index of the hottest temperature probe. */
    uint8_t  MaxTempNo;
    /* Index of the coldest temperature probe. */
    uint8_t  MinTempNo;
    /* Sum of all cell voltages in the current frame. */
    uint16_t CellVoltSum;
    /* Precharge voltage used by the BMS state machine. */
    uint16_t PrechargeVolt;
    /* Battery fault bitmap or vendor-specific fault word. */
    uint32_t BatteryFaultCode;
    /* Hall sensor current reading in raw units. */
    int32_t  HallCurrent;
    /* Hall sensor error flag. */
    uint8_t  HallError;
    /* Hall sensor error code. */
    uint8_t  HallErrorCode;
    /* State of charge percentage. */
    uint8_t  SOC;
    /* Charger requested output voltage. */
    uint16_t Charge_ReqVolt;
    /* Charger requested output current. */
    uint16_t Charge_ReqCurr;
    /* Charger command byte or command state. */
    uint8_t  Charger_Cmd;
    /* Charger measured output voltage in raw units. */
    uint16_t Charger_OutVolt;
    /* Charger measured output current in raw units. */
    int16_t  Charger_OutCurr;
    /* Charger/BMS working state used by higher-level logic. */
    uint8_t  Work_State;
    /* Charger requested voltage converted to engineering units. */
    float    Charger_ReqVolt_Act;
    /* Charger requested current converted to engineering units. */
    float    Charger_ReqCurr_Act;
    /* Charger measured voltage converted to engineering units. */
    float    Charger_OutVolt_Act;
    /* Charger measured current converted to engineering units. */
    float    Charger_OutCurr_Act;
} Battery_InfoDef;

typedef struct
{
    /* Vehicle speed in km/h or another project-defined integer unit. */
    uint8_t  Vehicle_Speed;
    /* Per-motor power estimate for four drive units. */
    float    Motor_Power[4];
    /* Total vehicle power aggregated from all drive units. */
    float    Total_Power;
    /* Driving mode or torque-map mode reported by the ECU. */
    int8_t   driving_mode;
    /* Per-motor speed in RPM. */
    int16_t  Motor_RPM[4];
    /* Per-motor torque request or measured torque. */
    int16_t  Motor_Torque[4];
    /* Motor case temperature, converted to deci-degrees or raw project units. */
    int16_t  MotorTempDc[4];
    /* Inverter temperature. */
    int16_t  InverterTempDc[4];
    /* IGBT temperature. */
    int16_t  IgbtTempDc[4];
    /* Magnetizing current for each motor controller. */
    int16_t  MagnetizingCurrent[4];
    /* Torque-producing current for each motor controller. */
    int16_t  TorqueCurrent[4];
    /* Positive torque limit for each motor controller. */
    int16_t  TorqueLimitPositive[4];
    /* Negative torque limit for each motor controller. */
    int16_t  TorqueLimitNegative[4];
    /* DC bus voltage for each motor controller. */
    uint16_t DCBusVoltage[4];
    /* Rolling message counter for each motor controller. */
    uint16_t MessageCounter[4];
    /* Diagnostic word or fault number for each motor controller. */
    uint32_t DiagnosticNumber[4];
    /* Extended error information for each motor controller. */
    uint32_t ErrorInfo[4];
    /* Logic state bitmap for each motor controller. */
    uint8_t  LogicState[4];
    /* Status bitfield for each motor controller. */
    uint8_t  StatusBits[4];
    /* Steering angle in 0.1 degree units. */
    int16_t  SteeringAngleDeciDeg;
    /* Throttle pedal position in 0.1 percent units. */
    uint16_t ThrottlePositionDeciPct;
    /* Oil pressure in milli-kPa. */
    uint16_t OilPressureMilliKpa;
    /* Legacy per-channel error cache kept for compatibility. */
    int      ERRO[4];
} CANB_ECU_VehicleTypeDef;

typedef struct
{
    /* Raw accelerometer X-axis reading. */
    int16_t  Acc_X_Raw;
    /* Raw accelerometer Y-axis reading. */
    int16_t  Acc_Y_Raw;
    /* Raw accelerometer Z-axis reading. */
    int16_t  Acc_Z_Raw;
    /* Calibrated X-axis acceleration in g. */
    float    Acc_X_Act;
    /* Calibrated Y-axis acceleration in g. */
    float    Acc_Y_Act;
    /* Calibrated Z-axis acceleration in g. */
    float    Acc_Z_Act;
    /* Longitudinal acceleration used by vehicle dynamics logic. */
    float    Long_Accel;
    /* Lateral acceleration used by vehicle dynamics logic. */
    float    Lat_Accel;
    /* GPS or wheel-speed value in km/h. */
    uint16_t GpsSpeedKmh;
    /* Raw yaw-rate reading. */
    int16_t  YawRateRaw;
    /* Raw yaw angle reading. */
    int16_t  YawRaw;
    /* Validity bits for the IMU data channels. */
    uint8_t  ValidFlags;
} CANB_IMU_TypeDef;

typedef struct
{
    /* Vehicle ECU telemetry cache. */
    CANB_ECU_VehicleTypeDef ECU;
    /* IMU telemetry cache. */
    CANB_IMU_TypeDef        IMU;
    /* Raw IVT cache.
     * These fields represent the private team energy meter messages directly,
     * before they are folded into the generic EnergyMeter view.
     */
    struct
    {
        /* Battery current in milliamps. */
        int32_t  CurrentMa;
        /* U1 bus voltage in millivolts. */
        int32_t  VoltageU1Mv;
        /* Accumulated energy in watt-hours. */
        int32_t  EnergyWh;
        /* Instantaneous power in watts. */
        int32_t  PowerW;
        /* Validity/status field for the current value. */
        uint32_t CurrentState;
        /* Validity/status field for the U1 voltage value. */
        uint32_t VoltageU1State;
        /* Validity/status field for the energy value. */
        uint32_t EnergyState;
        /* Validity/status field for the power value. */
        uint32_t PowerState;
        /* Bitmask that marks which IVT fields have been decoded. */
        uint8_t  ValidFlags;
        /* Last decoded byte order for this IVT source. */
        uint8_t  ByteOrder;
    } IVT;
    /* Unified energy meter cache for publish.
     * This layer allows protobuf output to stay stable even if the physical
     * source is IVT or FS.
     */
    struct
    {
        /* Battery current in milliamps. */
        int32_t  CurrentMa;
        /* Pack voltage in millivolts. */
        int32_t  VoltageMv;
        /* Accumulated energy in watt-hours. */
        int32_t  EnergyWh;
        /* Instantaneous power in watts. */
        int32_t  PowerW;
        /* Unified status bits exposed to the publish layer. */
        uint32_t Status;
        /* Source message counter used for change detection. */
        uint8_t  MsgCounter;
        /* Validity bits for the unified energy meter view. */
        uint8_t  ValidFlags;
        /* Auto-selected source tag: unknown, IVT, or FS. */
        uint8_t  AutoSource;
    } EnergyMeter;
} CANB_LoopAllData;

typedef struct
{
    /* Region-average temperatures for each sensor, stored in centi-Celsius. */
    int32_t  RegionTemp[MLX90640_SENSOR_COUNT][MLX90640_REGION_COUNT];
    /* Ambient temperature for each sensor, stored in centi-Celsius. */
    int32_t  Ta[MLX90640_SENSOR_COUNT];
    /* Reflected temperature for each sensor, stored in centi-Celsius. */
    int32_t  Tr[MLX90640_SENSOR_COUNT];
    /* Last capture or recovery error code for each sensor. */
    int32_t  LastError[MLX90640_SENSOR_COUNT];
    /* Number of successful frames captured for each sensor. */
    uint32_t FrameCounter[MLX90640_SENSOR_COUNT];
    /* Bitmask of sensors that currently have valid data. */
    uint8_t  ValidMask;
    /* Sensor ID that produced the most recent valid frame. */
    uint8_t  ActiveSensorId;
} MLX90640_FrameTypeDef;

typedef struct
{
    /* Generic CDC monitor cache populated by SPI3 polling.
     * This is intentionally protocol-agnostic for now: the bus is treated as a
     * read-only monitoring path until CDC mailbox semantics are finalized.
     */
    /* Number of valid CDC frames observed. */
    uint32_t FrameCounter;
    /* ID of the last observed frame. */
    uint32_t LastMsgId;
    /* Last HAL or driver error code. */
    uint32_t LastError;
    /* Payload bytes of the last observed frame. */
    uint8_t  LastData[8];
    /* DLC of the last observed frame. */
    uint8_t  LastDlc;
    /* Nonzero when the last frame was an extended ID frame. */
    uint8_t  LastIsExtId;
    /* Nonzero when the CDC cache contains usable data. */
    uint8_t  Valid;
} CDC_InfoDef;

/* Shared battery telemetry cache. */
extern Battery_InfoDef g_BatteryInfo;
/* Shared CANB decoded telemetry cache. */
extern CANB_LoopAllData g_CANB_LoopData;
/* Shared MLX90640 thermal cache. */
extern MLX90640_FrameTypeDef g_MLX90640_Frame;
/* Shared CDC monitor cache. */
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
