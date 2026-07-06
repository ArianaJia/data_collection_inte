#ifndef __BATTERY_H
#define __BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>  // 启用uint8_t/uint16_t/int16_t等固定宽度类型

/************************** 协议校准宏定义（CANB环路约定） **************************/
#define CHARGER_VOLT_SCALE     0.1f    // 充电机电压缩放：原始值×0.1=实际电压(V)
#define CHARGER_CURR_SCALE     0.1f    // 充电机电流缩放：原始值×0.1=实际电流(A)
#define IMU_ACC_SCALE          (16.0f/32768.0f) // IMU加速度校准：组合值×16g/32768
#define BAT_TEMP_POINT_PER_MOD 23
#define BAT_TEMP_OFFSET        30
#define MLX90640_SENSOR_COUNT  4U
#define MLX90640_REGION_COUNT  4U
#define MLX90640_TEMP_SCALE    100L
/************************** 电池箱全量数据结构体（整合SOC+充电机所有交互数据） **************************/
typedef struct
{
    /********** 电池箱基础参数 **********/
    uint16_t CellVolt[6][24];   // 单体电压[模组][单体]，mV
    int8_t   ModTemp[6][8];     // 模组温度[模组][测点]，℃（已校准）
    uint16_t TotalVolt;          // 电池总电压，mV
    int16_t  TotalCurrent;       // 电池总电流，mA（正放电/负充电）
    int8_t   MaxTemp;            // 电池最高温度，℃
    int8_t   MinTemp;            // 电池最低温度，℃
    uint16_t MaxCellVolt;        // 单体最高电压，mV
    uint16_t MinCellVolt;        // 单体最低电压，mV
    uint8_t  Air1_State;         // 主继电器1（0断开/1闭合）
    uint8_t  Air2_State;         // 主继电器2（0断开/1闭合）
    uint8_t  Air3_State;         // 预充继电器（0断开/1闭合）
    uint8_t  Charge_Enable;      // 充电使能（0禁止/1允许）
    uint8_t  IMD_State;          // IMD状态

    /********** 电池核心状态-SOC **********/
    uint8_t  SOC;                // 剩余电量，0~100%（直接使用，无需校准）

    /********** BMS→充电机控制指令 **********/
    uint16_t Charge_ReqVolt;    // 请求充电机最高输出电压（原始值）
    uint16_t Charge_ReqCurr;    // 请求充电机最高输出电流（原始值）
    uint8_t  Charger_Cmd;        // 充电机控制指令：0=开启充电，1=关闭充电

    /********** 充电机→BMS状态反馈 **********/
    uint16_t Charger_OutVolt;    // 充电机实际输出电压（原始值）
    int16_t  Charger_OutCurr;    // 充电机实际输出电流（原始值，正充电/负放电）
    uint8_t  Work_State;     // 充电机工作状态（0=待机，1=充电，非0=故障码）

    /********** 校准后充电参数（全局复用，避免重复计算） **********/
    float    Charger_ReqVolt_Act;// 实际请求最高电压(V) = Charger_MaxVolt × CHARGER_VOLT_SCALE
    float    Charger_ReqCurr_Act;// 实际请求最高电流(A) = Charger_MaxCurr × CHARGER_CURR_SCALE
    float    Charger_OutVolt_Act;// 实际输出电压(V) = Charger_OutVolt × CHARGER_VOLT_SCALE
    float    Charger_OutCurr_Act;// 实际输出电流(A) = Charger_OutCurr × CHARGER_CURR_SCALE
} Battery_InfoDef;

/************************** CANB-ECU车辆数据结构体（整合车速/功率，无重复） **************************/
typedef struct
{
    /********** 核心行驶参数 **********/
    uint8_t  Vehicle_Speed;      // 整车车速，km/h
    float    Motor_Power[4];     // 四电机实时功率[0~3]，kW（对应IN0~IN3）
    float    Total_Power;        // 整车总功率，kW（正放电/负充电，电机功率总和）
    uint8_t  driving_mode;

    /********** 车轮转速（单位：rpm）与电机扭矩（N·M）**********/
    uint16_t Motor_RPM[4];           //电机转速
    uint16_t Motor_Torque[4];        //电机扭矩

    /********** 新增：0x503 故障/错误码（按协议4路） **********/
    int      ERRO[4];                // 4路整形错误码/故障码（解析方式同功率的低/高字节拼接）

    /********** 车轮附着系数利用率（0~100%） **********/
//    uint8_t  Adhesion_RL_Long;   // 左后-纵向附着系数利用率
//    uint8_t  Adhesion_RL_Lat;    // 左后-横向附着系数利用率
//    uint8_t  Adhesion_FL_Long;   // 左前-纵向附着系数利用率
//    uint8_t  Adhesion_FL_Lat;    // 左前-横向附着系数利用率
//    uint8_t  Adhesion_RR_Long;   // 右后-纵向附着系数利用率
//    uint8_t  Adhesion_RR_Lat;    // 右后-横向附着系数利用率
//    uint8_t  Adhesion_FR_Long;   // 右前-纵向附着系数利用率
//    uint8_t  Adhesion_FR_Lat;    // 右前-横向附着系数利用率
} CANB_ECU_VehicleTypeDef;

/************************** CANB-IMU惯导数据结构体（平铺字段，整合横/纵向加速度） **************************/
typedef struct
{
//    /********** 时间信息 **********/
//    uint8_t  Time_YY;            // 年份（例：24→2024年）
//    uint8_t  Time_MM;            // 月份（1~12）
//    uint8_t  Time_DD;            // 日期（1~31）
//    uint8_t  Time_HH;            // 小时（0~23）
//    uint8_t  Time_MN;            // 分钟（0~59）
//    uint8_t  Time_SS;            // 秒（0~59）
//
    /********** 原生三轴加速度（单位：g） **********/
    int16_t  Acc_X_Raw;          // X轴加速度原始值（低字节在前）
    int16_t  Acc_Y_Raw;          // Y轴加速度原始值
    int16_t  Acc_Z_Raw;          // Z轴加速度原始值
    float    Acc_X_Act;          // 实际X轴加速度 = Acc_X_Raw × IMU_ACC_SCALE
    float    Acc_Y_Act;          // 实际Y轴加速度
    float    Acc_Z_Act;          // 实际Z轴加速度

    /********** 车辆横/纵向加速度（直接使用，单位：g） **********/
    float    Long_Accel;         // 纵向加速度（前进为正）
    float    Lat_Accel;          // 横向加速度（右偏为正）

//    /********** 三轴角速度 **********/
//    int16_t  Gyro_X_Raw;         // X轴角速度原始值
//    int16_t  Gyro_Y_Raw;         // Y轴角速度原始值
//    int16_t  Gyro_Z_Raw;         // Z轴角速度原始值
//
//    /********** 姿态角（横滚/俯仰/航向） **********/
//    int32_t  Roll_Raw;           // 横滚角原始值
//    int32_t  Pitch_Raw;          // 俯仰角原始值
//    int32_t  Yaw_Raw;            // 航向角原始值
//
//    /********** 三轴磁场 **********/
//    int16_t  Mag_X_Raw;          // X轴磁场原始值
//    int16_t  Mag_Y_Raw;          // Y轴磁场原始值
//    int16_t  Mag_Z_Raw;          // Z轴磁场原始值
} CANB_IMU_TypeDef;

/************************** CANB环路全量数据结构体（聚合ECU+IMU，全局入口） **************************/
typedef struct
{
    CANB_ECU_VehicleTypeDef ECU; // ECU整车状态（车速/功率/轮速/附着系数）
    CANB_IMU_TypeDef        IMU; // IMU惯导数据（含横/纵向加速度）
} CANB_LoopAllData;

/************************** 全局变量声明（项目所有文件直接调用） **************************/
extern Battery_InfoDef    g_BatteryInfo;  // 电池箱+充电机核心数据
extern CANB_LoopAllData   g_CANB_LoopData;// CANB环路全量数据（ECU+IMU）

/************************** MLX90640 红外阵列数据（新增：FreeRTOS采集+UART1 DMA发送） **************************/
// 统一放入结构体，便于扩展与传递
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

extern MLX90640_FrameTypeDef g_MLX90640_Frame;

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_H */
