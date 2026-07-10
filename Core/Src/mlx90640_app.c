#include "mlx90640_app.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "main.h"
#include "task.h"
#include "telemetry_data.h"

#include <stdio.h>


typedef struct
{
  uint16_t eeData[MLX90640_EEPROM_DUMP_NUM];
  uint16_t frameData[834];
  float tempMap[MLX90640_PIXEL_NUM];
  paramsMLX90640 params;
} MLX90640_RuntimeData_t;

static MLX90640_RuntimeData_t g_runtime;
static uint8_t g_sensorReady[MLX90640_SENSOR_COUNT];
static char g_logLine[MLX90640_LOG_BUFFER_SIZE];
static MLX90640_AppLogFn g_logCallback = NULL;
static volatile uint8_t g_mlx90640Busy = 0U;
static uint8_t g_mlx90640RuntimePrepared = 0U;

/* Check whether the FreeRTOS scheduler is already running. */
static uint8_t MLX90640_App_IsSchedulerRunning(void)
{
  return (osKernelGetState() == osKernelRunning) ? 1U : 0U;
}

/* Read the current millisecond tick used for timeout handling. */
static uint32_t MLX90640_App_GetTimeMs(void)
{
  return HAL_GetTick();
}

/* Sleep with the RTOS delay API when possible, otherwise fall back to HAL. */
static void MLX90640_App_SleepMs(uint32_t delay_ms)
{
  if (delay_ms == 0U)
  {
    return;
  }

  if (MLX90640_App_IsSchedulerRunning() != 0U)
  {
    osDelay(delay_ms);
    return;
  }

  HAL_Delay(delay_ms);
}

/* Try to acquire the shared MLX90640 lock without blocking the system forever. */
static uint8_t MLX90640_App_TryLock(uint32_t timeout_ms)
{
  uint32_t start_ms = MLX90640_App_GetTimeMs();

  for (;;)
  {
    uint8_t lock_acquired = 0U;

    taskENTER_CRITICAL();
    if (g_mlx90640Busy == 0U)
    {
      g_mlx90640Busy = 1U;
      lock_acquired = 1U;
    }
    taskEXIT_CRITICAL();

    if (lock_acquired != 0U)
    {
      return 1U;
    }

    if ((MLX90640_App_GetTimeMs() - start_ms) >= timeout_ms)
    {
      return 0U;
    }

    MLX90640_App_SleepMs(1U);
  }
}

/* Release the shared MLX90640 lock. */
static void MLX90640_App_Unlock(void)
{
  taskENTER_CRITICAL();
  g_mlx90640Busy = 0U;
  taskEXIT_CRITICAL();
}

/* Mark one sensor as offline and keep its last failure code for recovery. */
static void MLX90640_App_SetSensorOffline(uint8_t sensor_id, int32_t error)
{
  if (sensor_id >= MLX90640_SENSOR_COUNT)
  {
    return;
  }

  g_sensorReady[sensor_id] = 0U;
  g_MLX90640_Frame.LastError[sensor_id] = error;
  g_MLX90640_Frame.ValidMask &= (uint8_t)(~(1U << sensor_id));
}

/* Initialize the shared runtime buffers and I2C helpers once. */
static void MLX90640_App_PrepareRuntime(void)
{
  if (g_mlx90640RuntimePrepared != 0U)
  {
    return;
  }

  MLX90640_SetExtractScratch(g_runtime.tempMap);
  (void)MLX90640_I2CInit();
  g_mlx90640RuntimePrepared = 1U;
}

/* Emit a recovery log line for a sensor that failed to come back online. */
static void MLX90640_App_LogRecoverStatus(uint8_t sensor_id, int status)
{
  (void)snprintf(g_logLine, sizeof(g_logLine),
      "MLX90640 recover status, sensor=%u, channel=0x%02X, status=%d, hal=%d, err=0x%08lX\r\n",
      (unsigned int)sensor_id,
      (unsigned int)MLX90640_GetSensorChannel(sensor_id),
      status,
      (int)TCA9548A_GetLastHalStatus(),
      (unsigned long)TCA9548A_GetLastErrorCode());
  MLX90640_App_Log(g_logLine);
}

/* Convert Celsius to centi-Celsius for compact integer telemetry storage. */
static int32_t MLX90640_App_TempToCentiC(float temp_c)
{
  float scaled_temp = temp_c * (float)MLX90640_TEMP_SCALE;

  if (scaled_temp >= 0.0f)
  {
    return (int32_t)(scaled_temp + 0.5f);
  }

  return (int32_t)(scaled_temp - 0.5f);
}

/* Forward a log line to the caller-provided callback when logging is enabled. */
void MLX90640_App_Log(const char *text)
{
  if ((g_logCallback != NULL) && (text != NULL))
  {
    g_logCallback(text);
  }
}

/* Build a detailed log line for one failed MLX90640 initialization step. */
static void MLX90640_App_LogInitError(uint8_t sensor_id, const char *step, int error)
{
  (void)snprintf(g_logLine, sizeof(g_logLine),
      "MLX90640 init error, sensor=%u, channel=0x%02X, step=%s, status=%d, hal=%d, err=0x%08lX\r\n",
      (unsigned int)sensor_id,
      (unsigned int)MLX90640_GetSensorChannel(sensor_id),
      step,
      error,
      (int)TCA9548A_GetLastHalStatus(),
      (unsigned long)TCA9548A_GetLastErrorCode());
  MLX90640_App_Log(g_logLine);
}

/* Select one sensor channel, probe the device, and load its EEPROM parameters. */
static int MLX90640_App_LoadParametersOnChannel(uint8_t sensor_id)
{
  int status = MLX90640_SelectSensor(sensor_id);
  if (status != MLX90640_NO_ERROR)
  {
    MLX90640_App_LogInitError(sensor_id, "select TCA9548A channel", status);
    return status;
  }

  status = MLX90640_I2CProbe(MLX90640_ADDR);
  (void)snprintf(g_logLine, sizeof(g_logLine),
      "MLX90640 probe, sensor=%u, channel=0x%02X, status=%d, hal=%d, err=0x%08lX\r\n",
      (unsigned int)sensor_id,
      (unsigned int)MLX90640_GetSensorChannel(sensor_id),
      status,
      (int)MLX90640_GetLastHalStatus(),
      (unsigned long)MLX90640_GetLastErrorCode());
  MLX90640_App_Log(g_logLine);

  if (status != MLX90640_NO_ERROR)
  {
    return status;
  }

  status = MLX90640_DumpEE(MLX90640_ADDR, g_runtime.eeData);
  if (status != MLX90640_NO_ERROR)
  {
    MLX90640_App_LogInitError(sensor_id, "dump EEPROM", status);
    return status;
  }

  status = MLX90640_ExtractParameters(g_runtime.eeData, &g_runtime.params);
  if (status != MLX90640_NO_ERROR)
  {
    MLX90640_App_LogInitError(sensor_id, "extract parameters", status);
  }

  return status;
}

/* Finish the per-sensor hardware setup after parameters have been loaded. */
static int MLX90640_App_InitSensorOnChannel(uint8_t sensor_id)
{
  int status = MLX90640_App_LoadParametersOnChannel(sensor_id);
  if (status != MLX90640_NO_ERROR)
  {
    return status;
  }

  status = MLX90640_SetChessMode(MLX90640_ADDR);
  if (status != MLX90640_NO_ERROR)
  {
    MLX90640_App_LogInitError(sensor_id, "set chess mode", status);
    return status;
  }

  status = MLX90640_SetRefreshRate(MLX90640_ADDR, MLX90640_REF_1HZ);
  if (status != MLX90640_NO_ERROR)
  {
    MLX90640_App_LogInitError(sensor_id, "set refresh rate", status);
    return status;
  }

  return MLX90640_NO_ERROR;
}

/* Reduce the 32x24 pixel map into region-level temperatures for telemetry. */
static void MLX90640_App_UpdateRegionTemp(uint8_t sensor_id, const float *pixel_temp)
{
  float region_sum[MLX90640_REGION_COUNT] = {0.0f};
  uint16_t region_count[MLX90640_REGION_COUNT] = {0U};
  const uint16_t rows_per_region = (uint16_t)(MLX90640_LINE_NUM / MLX90640_REGION_COUNT);

  for (uint16_t row = 0U; row < MLX90640_LINE_NUM; row++)
  {
    uint8_t region_index = (uint8_t)(row / rows_per_region);

    if (region_index >= MLX90640_REGION_COUNT)
    {
      region_index = (uint8_t)(MLX90640_REGION_COUNT - 1U);
    }

    for (uint16_t col = 0U; col < MLX90640_COLUMN_NUM; col++)
    {
      uint16_t pixel_index = (uint16_t)(row * MLX90640_COLUMN_NUM + col);
      region_sum[region_index] += pixel_temp[pixel_index];
      region_count[region_index]++;
    }
  }

  for (uint8_t region_index = 0U; region_index < MLX90640_REGION_COUNT; region_index++)
  {
    if (region_count[region_index] > 0U)
    {
      g_MLX90640_Frame.RegionTemp[sensor_id][region_index] =
          MLX90640_App_TempToCentiC(region_sum[region_index] / (float)region_count[region_index]);
    }
    else
    {
      g_MLX90640_Frame.RegionTemp[sensor_id][region_index] = 0;
    }
  }
}

/* Register the external log sink used by this MLX90640 application layer. */
void MLX90640_App_SetLogCallback(MLX90640_AppLogFn callback)
{
  g_logCallback = callback;
}

/* Initialize every configured MLX90640 sensor and record which ones are alive. */
int MLX90640_App_Init(void)
{
  uint8_t initialized_count = 0U;

  if (MLX90640_App_TryLock(MLX90640_APP_LOCK_TIMEOUT_MS) == 0U)
  {
    return -MLX90640_I2C_WRITE_ERROR;
  }

  MLX90640_App_PrepareRuntime();

  for (uint8_t sensor_id = 0U; sensor_id < MLX90640_SENSOR_COUNT; sensor_id++)
  {
    int status = MLX90640_App_InitSensorOnChannel(sensor_id);
    if (status != MLX90640_NO_ERROR)
    {
      MLX90640_App_SetSensorOffline(sensor_id, status);
      continue;
    }

    g_sensorReady[sensor_id] = 1U;
    g_MLX90640_Frame.LastError[sensor_id] = 0;
    initialized_count++;

    (void)snprintf(g_logLine, sizeof(g_logLine),
        "MLX90640 init ok, sensor=%u, channel=0x%02X\r\n",
        (unsigned int)sensor_id,
        (unsigned int)MLX90640_GetSensorChannel(sensor_id));
    MLX90640_App_Log(g_logLine);
  }

  if (initialized_count == 0U)
  {
    MLX90640_App_Log("MLX90640 init pending, all sensors offline\r\n");
    MLX90640_App_Unlock();
    return MLX90640_NO_ERROR;
  }

  MLX90640_App_Log("MLX90640 initialized\r\n");
  MLX90640_App_Unlock();
  return MLX90640_NO_ERROR;
}

/* Capture one full frame from the selected sensor and update shared telemetry. */
int MLX90640_App_CaptureOnce(uint8_t sensor_id)
{
  if (sensor_id >= MLX90640_SENSOR_COUNT)
  {
    return -MLX90640_I2C_WRITE_ERROR;
  }

  if (g_sensorReady[sensor_id] == 0U)
  {
    return g_MLX90640_Frame.LastError[sensor_id];
  }

  if (MLX90640_App_TryLock(MLX90640_APP_LOCK_TIMEOUT_MS) == 0U)
  {
    return -MLX90640_I2C_WRITE_ERROR;
  }

  int status = MLX90640_App_LoadParametersOnChannel(sensor_id);
  if (status != MLX90640_NO_ERROR)
  {
    MLX90640_App_SetSensorOffline(sensor_id, status);
    MLX90640_App_Unlock();
    return status;
  }

  uint8_t subPageMask = 0U;
  uint8_t frameAttempts = 0U;
  float ta = 0.0f;
  float tr = 0.0f;

  while ((subPageMask != 0x03U) && (frameAttempts < MLX90640_FRAME_ATTEMPTS_MAX))
  {
    status = MLX90640_GetFrameData(MLX90640_ADDR, g_runtime.frameData);
    if (status != MLX90640_NO_ERROR)
    {
      (void)snprintf(g_logLine, sizeof(g_logLine), "Frame read error: %d\r\n", status);
      MLX90640_App_Log(g_logLine);
      MLX90640_App_SetSensorOffline(sensor_id, status);
      MLX90640_App_Unlock();
      return status;
    }

    subPageMask |= (uint8_t)(1U << (g_runtime.frameData[833] & 1U));
    ta = MLX90640_GetTa(g_runtime.frameData, &g_runtime.params);
    tr = ta - MLX90640_TA_SHIFT;
    MLX90640_CalculateTo(
        g_runtime.frameData,
        &g_runtime.params,
        MLX90640_EMISSIVITY,
        tr,
        g_runtime.tempMap);
    frameAttempts++;
  }

  if (subPageMask != 0x03U)
  {
    MLX90640_App_SetSensorOffline(sensor_id, -MLX90640_FRAME_DATA_ERROR);
    MLX90640_App_Unlock();
    return -MLX90640_FRAME_DATA_ERROR;
  }

  MLX90640_BadPixelsCorrection(g_runtime.params.brokenPixels, g_runtime.tempMap, 1, &g_runtime.params);
  MLX90640_BadPixelsCorrection(g_runtime.params.outlierPixels, g_runtime.tempMap, 1, &g_runtime.params);
  MLX90640_App_UpdateRegionTemp(sensor_id, g_runtime.tempMap);

  g_MLX90640_Frame.Ta[sensor_id] = MLX90640_App_TempToCentiC(ta);
  g_MLX90640_Frame.Tr[sensor_id] = MLX90640_App_TempToCentiC(tr);
  g_MLX90640_Frame.LastError[sensor_id] = 0;
  g_MLX90640_Frame.FrameCounter[sensor_id]++;
  g_MLX90640_Frame.ActiveSensorId = sensor_id;
  g_MLX90640_Frame.ValidMask |= (uint8_t)(1U << sensor_id);

  MLX90640_App_Unlock();
  return MLX90640_NO_ERROR;
}

/* Retry initialization for sensors that were previously marked offline. */
int MLX90640_App_ServiceHealth(void)
{
  if (MLX90640_App_TryLock(0U) == 0U)
  {
    return MLX90640_NO_ERROR;
  }

  MLX90640_App_PrepareRuntime();

  for (uint8_t sensor_id = 0U; sensor_id < MLX90640_SENSOR_COUNT; sensor_id++)
  {
    int status;

    if (g_sensorReady[sensor_id] != 0U)
    {
      continue;
    }

    status = MLX90640_App_InitSensorOnChannel(sensor_id);
    if (status == MLX90640_NO_ERROR)
    {
      g_sensorReady[sensor_id] = 1U;
      g_MLX90640_Frame.LastError[sensor_id] = 0;
      (void)snprintf(g_logLine, sizeof(g_logLine),
          "MLX90640 recover ok, sensor=%u, channel=0x%02X\r\n",
          (unsigned int)sensor_id,
          (unsigned int)MLX90640_GetSensorChannel(sensor_id));
      MLX90640_App_Log(g_logLine);
    }
    else
    {
      MLX90640_App_SetSensorOffline(sensor_id, status);
      MLX90640_App_LogRecoverStatus(sensor_id, status);
    }

    MLX90640_App_Unlock();
    return status;
  }

  MLX90640_App_Unlock();
  return MLX90640_NO_ERROR;
}

/* Report whether the selected sensor is currently considered ready. */
uint8_t MLX90640_App_IsSensorReady(uint8_t sensor_id)
{
  if (sensor_id >= MLX90640_SENSOR_COUNT)
  {
    return 0U;
  }

  return g_sensorReady[sensor_id];
}

/* Expose the latest floating-point temperature map buffer. */
const float *MLX90640_App_GetTempMap(void)
{
  return g_runtime.tempMap;
}





