#include "mlx90640_app.h"

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "battery.h"

#include <stdio.h>

#define MLX90640_FRAME_ATTEMPTS_MAX 4U
#define MLX90640_LOG_BUFFER_SIZE    160U

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

static int32_t MLX90640_App_TempToCentiC(float temp_c)
{
  float scaled_temp = temp_c * (float)MLX90640_TEMP_SCALE;

  if (scaled_temp >= 0.0f)
  {
    return (int32_t)(scaled_temp + 0.5f);
  }

  return (int32_t)(scaled_temp - 0.5f);
}

static void MLX90640_App_Log(const char *text)
{
  if ((g_logCallback != NULL) && (text != NULL))
  {
    g_logCallback(text);
  }
}

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

static void MLX90640_App_UpdateRegionTemp(uint8_t sensor_id, const float *pixel_temp)
{
  float region_sum[MLX90640_REGION_COUNT] = {0.0f};
  uint16_t region_count[MLX90640_REGION_COUNT] = {0U};

  for (uint16_t row = 0U; row < MLX90640_LINE_NUM; row++)
  {
    for (uint16_t col = 0U; col < MLX90640_COLUMN_NUM; col++)
    {
      uint16_t pixel_index = (uint16_t)(row * MLX90640_COLUMN_NUM + col);
      uint8_t region_index = (uint8_t)((col * MLX90640_REGION_COUNT) / MLX90640_COLUMN_NUM);

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

void MLX90640_App_SetLogCallback(MLX90640_AppLogFn callback)
{
  g_logCallback = callback;
}

int MLX90640_App_Init(void)
{
  uint8_t initialized_count = 0U;

  MLX90640_SetExtractScratch(g_runtime.tempMap);
  (void)MLX90640_I2CInit();

  for (uint8_t sensor_id = 0U; sensor_id < MLX90640_SENSOR_COUNT; sensor_id++)
  {
    int status = MLX90640_App_InitSensorOnChannel(sensor_id);
    if (status != MLX90640_NO_ERROR)
    {
      g_sensorReady[sensor_id] = 0U;
      g_MLX90640_Frame.LastError[sensor_id] = status;
      continue;
    }

    g_sensorReady[sensor_id] = 1U;
    initialized_count++;

    (void)snprintf(g_logLine, sizeof(g_logLine),
        "MLX90640 init ok, sensor=%u, channel=0x%02X\r\n",
        (unsigned int)sensor_id,
        (unsigned int)MLX90640_GetSensorChannel(sensor_id));
    MLX90640_App_Log(g_logLine);
  }

  if (initialized_count == 0U)
  {
    MLX90640_App_Log("MLX90640 init failed\r\n");
    return -MLX90640_I2C_WRITE_ERROR;
  }

  MLX90640_App_Log("MLX90640 initialized\r\n");
  return MLX90640_NO_ERROR;
}

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

  int status = MLX90640_App_LoadParametersOnChannel(sensor_id);
  if (status != MLX90640_NO_ERROR)
  {
    g_sensorReady[sensor_id] = 0U;
    g_MLX90640_Frame.LastError[sensor_id] = status;
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
      g_MLX90640_Frame.LastError[sensor_id] = status;
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
    g_MLX90640_Frame.LastError[sensor_id] = -MLX90640_FRAME_DATA_ERROR;
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

  return MLX90640_NO_ERROR;
}

uint8_t MLX90640_App_IsSensorReady(uint8_t sensor_id)
{
  if (sensor_id >= MLX90640_SENSOR_COUNT)
  {
    return 0U;
  }

  return g_sensorReady[sensor_id];
}

const float *MLX90640_App_GetTempMap(void)
{
  return g_runtime.tempMap;
}





