/* Shared telemetry caches live here so decoding, calibration and publishing
 * all operate on the same application-level state.
 */
#include "telemetry_data.h"

Battery_InfoDef g_BatteryInfo = {0};
CANB_LoopAllData g_CANB_LoopData = {0};
MLX90640_FrameTypeDef g_MLX90640_Frame = {0};
CDC_InfoDef g_CDCInfo = {0};

void Battery_Charger_Calibrate(void)
{
    /* Convert raw charger register values into engineering units for publish. */
    g_BatteryInfo.Charger_ReqVolt_Act = (float)g_BatteryInfo.Charge_ReqVolt * CHARGER_VOLT_SCALE;
    g_BatteryInfo.Charger_ReqCurr_Act = (float)g_BatteryInfo.Charge_ReqCurr * CHARGER_CURR_SCALE;
    g_BatteryInfo.Charger_OutVolt_Act = (float)g_BatteryInfo.Charger_OutVolt * CHARGER_VOLT_SCALE;
    g_BatteryInfo.Charger_OutCurr_Act = (float)g_BatteryInfo.Charger_OutCurr * CHARGER_CURR_SCALE;
}

void IMU_Acc_Calibrate(void)
{
    /* Keep both raw acceleration and derived longitudinal/lateral values. */
    g_CANB_LoopData.IMU.Acc_X_Act = (float)g_CANB_LoopData.IMU.Acc_X_Raw * IMU_ACC_SCALE;
    g_CANB_LoopData.IMU.Acc_Y_Act = (float)g_CANB_LoopData.IMU.Acc_Y_Raw * IMU_ACC_SCALE;
    g_CANB_LoopData.IMU.Acc_Z_Act = (float)g_CANB_LoopData.IMU.Acc_Z_Raw * IMU_ACC_SCALE;
    g_CANB_LoopData.IMU.Long_Accel = g_CANB_LoopData.IMU.Acc_X_Act;
    g_CANB_LoopData.IMU.Lat_Accel = g_CANB_LoopData.IMU.Acc_Y_Act;
}

void ECU_TotalPower_Calc(void)
{
    /* Aggregate motor power into a single vehicle-level total. */
    g_CANB_LoopData.ECU.Total_Power = 0.0f;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        g_CANB_LoopData.ECU.Total_Power += g_CANB_LoopData.ECU.Motor_Power[i];
    }
}

void Battery_Data_Clear(void)
{
    /* Reset the battery cache when a new decoding session starts. */
    g_BatteryInfo = (Battery_InfoDef){0};
}

void CANB_Loop_Data_Clear(void)
{
    /* Clear CANB-derived state in one shot to avoid stale telemetry. */
    g_CANB_LoopData = (CANB_LoopAllData){0};
}

void CDC_Data_Clear(void)
{
    /* Clear the generic CDC monitor cache together with the rest of telemetry. */
    g_CDCInfo = (CDC_InfoDef){0};
}
