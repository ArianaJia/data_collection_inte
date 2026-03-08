#include "battery.h"

/************************** 全局变量定义（项目所有文件可直接调用） **************************/
// 电池箱+充电机核心数据（初始化全字段为0，避免随机值）
Battery_InfoDef g_BatteryInfo = {0};
// CANB环路全量数据（ECU+IMU，初始化全字段为0）
CANB_LoopAllData g_CANB_LoopData = {0};

/************************** 核心辅助函数实现 **************************/
/**
 * @brief 充电参数校准函数：将充电机原始值转换为实际物理量（V/A）
 * @note 调用时机：CAN解析到充电机控制指令/状态反馈报文后立即调用
 *       一次校准，全局复用，避免重复计算
 */
void Battery_Charger_Calibrate(void)
{
    // 校准BMS→充电机控制指令实际值
    g_BatteryInfo.Charger_ReqVolt_Act = (float)g_BatteryInfo.Charge_ReqVolt * CHARGER_VOLT_SCALE;
    g_BatteryInfo.Charger_ReqCurr_Act = (float)g_BatteryInfo.Charge_ReqCurr * CHARGER_CURR_SCALE;

    // 校准充电机→BMS状态反馈实际值
    g_BatteryInfo.Charger_OutVolt_Act = (float)g_BatteryInfo.Charger_OutVolt * CHARGER_VOLT_SCALE;
    g_BatteryInfo.Charger_OutCurr_Act = (float)g_BatteryInfo.Charger_OutCurr * CHARGER_CURR_SCALE;
}

/**
 * @brief IMU加速度原始值校准函数：转换为实际g值并映射车辆横/纵向加速度
 * @param None
 * @note 调用时机：CAN解析到IMU加速度报文（0x51）后调用
 *       车辆加速度映射可根据IMU硬件安装方向调整（当前默认X轴=纵向，Y轴=横向）
 */
void IMU_Acc_Calibrate(void)
{
    // 校准IMU原生三轴加速度实际值（g）
    g_CANB_LoopData.IMU.Acc_X_Act = (float)g_CANB_LoopData.IMU.Acc_X_Raw * IMU_ACC_SCALE;
    g_CANB_LoopData.IMU.Acc_Y_Act = (float)g_CANB_LoopData.IMU.Acc_Y_Raw * IMU_ACC_SCALE;
    g_CANB_LoopData.IMU.Acc_Z_Act = (float)g_CANB_LoopData.IMU.Acc_Z_Raw * IMU_ACC_SCALE;

    // 映射车辆横/纵向加速度（可根据硬件实际安装方向修改轴对应关系）
    g_CANB_LoopData.IMU.Long_Accel = g_CANB_LoopData.IMU.Acc_X_Act; // X轴→纵向（前进为正）
    g_CANB_LoopData.IMU.Lat_Accel  = g_CANB_LoopData.IMU.Acc_Y_Act; // Y轴→横向（右偏为正）
}

/**
 * @brief 整车总功率计算函数：根据四电机功率求和，更新总功率
 * @param None
 * @note 调用时机：CAN解析到电机功率数据后/主循环定时调用
 *       功率正负定义：正=放电（电机驱动），负=充电（能量回收）
 */
void ECU_TotalPower_Calc(void)
{
    uint8_t i;
    g_CANB_LoopData.ECU.Total_Power = 0.0f;

    // 累加四电机功率，得到整车总功率
    for(i = 0; i < 4; i++)
    {
        g_CANB_LoopData.ECU.Total_Power += g_CANB_LoopData.ECU.Motor_Power[i];
    }
}

/**
 * @brief 电池箱数据清零函数：系统初始化/故障复位时调用
 * @param None
 * @note 清零所有电池核心参数，避免故障后数据残留
 */
void Battery_Data_Clear(void)
{
    // 直接重新初始化全局变量，全字段置0
    g_BatteryInfo = (Battery_InfoDef){0};
}

/**
 * @brief CANB环路数据清零函数：系统初始化/通信复位时调用
 * @param None
 * @note 清零ECU+IMU所有数据，保证CAN通信异常恢复后数据有效性
 */
void CANB_Loop_Data_Clear(void)
{
    // 直接重新初始化全局变量，全字段置0
    g_CANB_LoopData = (CANB_LoopAllData){0};
}
