#include "mlx90640_app.h"

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "telemetry_data.h"

#include <stdio.h>

typedef struct
{
  uint16_t FrameData[834];
  float TempMap[MLX90640_SENSOR_COUNT][MLX90640_PIXEL_NUM];
  paramsMLX90640 Params[MLX90640_SENSOR_COUNT];
} MLX90640_RuntimeData_t;

static MLX90640_RuntimeData_t mlx90640Runtime;
static uint16_t mlx90640EeData[MLX90640_EEPROM_DUMP_NUM];
static float mlx90640ExtractScratch[MLX90640_PIXEL_NUM];
static uint8_t mlx90640SensorReady[MLX90640_SENSOR_COUNT];
static uint8_t mlx90640SubPageMask[MLX90640_SENSOR_COUNT];
static char mlx90640LineBuffer[MLX90640_LOG_BUFFER_SIZE];
static MLX90640_AppLogFn mlx90640LogCallback = NULL;

static int MLX90640_InitSensorOnChannel(uint8_t sensorIndex);
static int MLX90640_LoadParametersOnChannel(uint8_t sensorIndex);
static int MLX90640_SelectReadySensor(uint8_t sensorIndex);
static int MLX90640_IsFrameReady(void);
static void MLX90640_LogInitError(uint8_t sensorIndex, const char *step, int error);
static void MLX90640_UpdateRegionTemp(uint8_t sensorIndex, const float *pixelTemp);
static int32_t MLX90640_TempToCentiC(float tempC);

void MLX90640_App_SetLogCallback(MLX90640_AppLogFn callback)
{
  mlx90640LogCallback = callback;
}

void MLX90640_App_Log(const char *text)
{
  if ((mlx90640LogCallback != NULL) && (text != NULL))
  {
    mlx90640LogCallback(text);
  }
}

static void MLX90640_LogInitError(uint8_t sensorIndex, const char *step, int error)
{
  (void)snprintf(mlx90640LineBuffer, sizeof(mlx90640LineBuffer),
      "MLX90640 init error, sensor=%u, channel=0x%02X, step=%s, status=%d, hal=%d, err=0x%08lX\r\n",
      (unsigned int)sensorIndex,
      (unsigned int)MLX90640_GetSensorChannel(sensorIndex),
      step,
      error,
      (int)TCA9548A_GetLastHalStatus(),
      (unsigned long)TCA9548A_GetLastErrorCode());
  MLX90640_App_Log(mlx90640LineBuffer);
}

static int MLX90640_LoadParametersOnChannel(uint8_t sensorIndex)
{
  int status;

  status = MLX90640_SelectSensor(sensorIndex);
  if (status != MLX90640_NO_ERROR)
  {
    MLX90640_LogInitError(sensorIndex, "select TCA9548A channel", status);
    return status;
  }

  status = MLX90640_I2CProbe(MLX90640_ADDR);
  (void)snprintf(mlx90640LineBuffer, sizeof(mlx90640LineBuffer),
      "MLX90640 probe, sensor=%u, channel=0x%02X, status=%d, hal=%d, err=0x%08lX\r\n",
      (unsigned int)sensorIndex,
      (unsigned int)MLX90640_GetSensorChannel(sensorIndex),
      status,
      (int)MLX90640_GetLastHalStatus(),
      (unsigned long)MLX90640_GetLastErrorCode());
  MLX90640_App_Log(mlx90640LineBuffer);
  if (status != MLX90640_NO_ERROR)
  {
    return status;
  }

  status = MLX90640_DumpEE(MLX90640_ADDR, mlx90640EeData);
  if (status != MLX90640_NO_ERROR)
  {
    MLX90640_LogInitError(sensorIndex, "dump EEPROM", status);
    return status;
  }

  status = MLX90640_ExtractParameters(mlx90640EeData, &mlx90640Runtime.Params[sensorIndex]);
  if (status != MLX90640_NO_ERROR)
  {
    MLX90640_LogInitError(sensorIndex, "extract parameters", status);
  }

  return status;
}

static int MLX90640_SelectReadySensor(uint8_t sensorIndex)
{
  int status;

  status = MLX90640_SelectSensor(sensorIndex);
  if (status != MLX90640_NO_ERROR)
  {
    MLX90640_LogInitError(sensorIndex, "select TCA9548A channel", status);
    return status;
  }

  status = MLX90640_I2CProbe(MLX90640_ADDR);
  (void)snprintf(mlx90640LineBuffer, sizeof(mlx90640LineBuffer),
      "MLX90640 probe, sensor=%u, channel=0x%02X, status=%d, hal=%d, err=0x%08lX\r\n",
      (unsigned int)sensorIndex,
      (unsigned int)MLX90640_GetSensorChannel(sensorIndex),
      status,
      (int)MLX90640_GetLastHalStatus(),
      (unsigned long)MLX90640_GetLastErrorCode());
  MLX90640_App_Log(mlx90640LineBuffer);

  return status;
}

static int MLX90640_IsFrameReady(void)
{
  uint16_t statusRegister = 0U;
  int status = MLX90640_I2CRead(MLX90640_ADDR, MLX90640_STATUS_REG, 1U, &statusRegister);

  if (status != MLX90640_NO_ERROR)
  {
    return status;
  }

  return (MLX90640_GET_DATA_READY(statusRegister) != 0U) ? MLX90640_NO_ERROR : -MLX90640_FRAME_NOT_READY_ERROR;
}

static int MLX90640_InitSensorOnChannel(uint8_t sensorIndex)
{
  int status;

  status = MLX90640_LoadParametersOnChannel(sensorIndex);
  if (status != MLX90640_NO_ERROR)
  {
    return status;
  }

  status = MLX90640_SetChessMode(MLX90640_ADDR);
  if (status != MLX90640_NO_ERROR)
  {
    MLX90640_LogInitError(sensorIndex, "set chess mode", status);
    return status;
  }

  status = MLX90640_SetRefreshRate(MLX90640_ADDR, MLX90640_REF_1HZ);
  if (status != MLX90640_NO_ERROR)
  {
    MLX90640_LogInitError(sensorIndex, "set refresh rate", status);
    return status;
  }

  return MLX90640_NO_ERROR;
}

int MLX90640_App_Init(void)
{
  uint8_t initializedCount = 0U;

  MLX90640_SetExtractScratch(mlx90640ExtractScratch);
  MLX90640_I2CInit();

  for (uint8_t sensorIndex = 0U; sensorIndex < MLX90640_SENSOR_COUNT; sensorIndex++)
  {
    int status = MLX90640_InitSensorOnChannel(sensorIndex);
    if (status != MLX90640_NO_ERROR)
    {
      mlx90640SensorReady[sensorIndex] = 0U;
      g_MLX90640_Frame.LastError[sensorIndex] = status;
      g_MLX90640_Frame.ValidMask &= (uint8_t)(~(1U << sensorIndex));
      continue;
    }

    mlx90640SensorReady[sensorIndex] = 1U;
    g_MLX90640_Frame.LastError[sensorIndex] = 0;
    initializedCount++;

    (void)snprintf(mlx90640LineBuffer, sizeof(mlx90640LineBuffer),
        "MLX90640 init ok, sensor=%u, channel=0x%02X\r\n",
        (unsigned int)sensorIndex,
        (unsigned int)MLX90640_GetSensorChannel(sensorIndex));
    MLX90640_App_Log(mlx90640LineBuffer);
  }

  if (initializedCount == 0U)
  {
    return -MLX90640_I2C_WRITE_ERROR;
  }

  return MLX90640_NO_ERROR;
}

int MLX90640_App_CaptureOnce(uint8_t sensorIndex)
{
  float ta = 0.0f;
  float tr = 0.0f;
  int status;

  if (sensorIndex >= MLX90640_SENSOR_COUNT)
  {
    return -MLX90640_I2C_WRITE_ERROR;
  }

  if (mlx90640SensorReady[sensorIndex] == 0U)
  {
    return g_MLX90640_Frame.LastError[sensorIndex];
  }

  status = MLX90640_SelectReadySensor(sensorIndex);
  if (status != MLX90640_NO_ERROR)
  {
    mlx90640SensorReady[sensorIndex] = 0U;
    g_MLX90640_Frame.LastError[sensorIndex] = status;
    g_MLX90640_Frame.ValidMask &= (uint8_t)(~(1U << sensorIndex));
    return status;
  }

  status = MLX90640_IsFrameReady();
  if (status != MLX90640_NO_ERROR)
  {
    if (status == -MLX90640_FRAME_NOT_READY_ERROR)
    {
      return status;
    }

    g_MLX90640_Frame.LastError[sensorIndex] = status;
    return status;
  }

  status = MLX90640_GetFrameData(MLX90640_ADDR, mlx90640Runtime.FrameData);
  if (status < 0)
  {
    if ((status == -MLX90640_FRAME_NOT_READY_ERROR) ||
        (status == -MLX90640_FRAME_DATA_ERROR))
    {
      return -MLX90640_FRAME_INCOMPLETE_ERROR;
    }

    (void)snprintf(mlx90640LineBuffer, sizeof(mlx90640LineBuffer),
        "Frame read error: %d, hal=%d, err=0x%08lX\r\n",
        status,
        (int)MLX90640_GetLastHalStatus(),
        (unsigned long)MLX90640_GetLastErrorCode());
    MLX90640_App_Log(mlx90640LineBuffer);
    g_MLX90640_Frame.LastError[sensorIndex] = status;
    return status;
  }

  mlx90640SubPageMask[sensorIndex] |= (uint8_t)(1U << (mlx90640Runtime.FrameData[833] & 1U));
  ta = MLX90640_GetTa(mlx90640Runtime.FrameData, &mlx90640Runtime.Params[sensorIndex]);
  tr = ta - MLX90640_TA_SHIFT;
  MLX90640_CalculateTo(
      mlx90640Runtime.FrameData,
      &mlx90640Runtime.Params[sensorIndex],
      MLX90640_EMISSIVITY,
      tr,
      mlx90640Runtime.TempMap[sensorIndex]);

  if (mlx90640SubPageMask[sensorIndex] != 0x03U)
  {
    g_MLX90640_Frame.LastError[sensorIndex] = -MLX90640_FRAME_INCOMPLETE_ERROR;
    return -MLX90640_FRAME_INCOMPLETE_ERROR;
  }

  MLX90640_BadPixelsCorrection(
      mlx90640Runtime.Params[sensorIndex].brokenPixels,
      mlx90640Runtime.TempMap[sensorIndex],
      1,
      &mlx90640Runtime.Params[sensorIndex]);
  MLX90640_BadPixelsCorrection(
      mlx90640Runtime.Params[sensorIndex].outlierPixels,
      mlx90640Runtime.TempMap[sensorIndex],
      1,
      &mlx90640Runtime.Params[sensorIndex]);

  MLX90640_UpdateRegionTemp(sensorIndex, mlx90640Runtime.TempMap[sensorIndex]);
  g_MLX90640_Frame.Ta[sensorIndex] = MLX90640_TempToCentiC(ta);
  g_MLX90640_Frame.Tr[sensorIndex] = MLX90640_TempToCentiC(tr);
  g_MLX90640_Frame.LastError[sensorIndex] = 0;
  g_MLX90640_Frame.FrameCounter[sensorIndex]++;
  g_MLX90640_Frame.ActiveSensorId = sensorIndex;
  g_MLX90640_Frame.ValidMask |= (uint8_t)(1U << sensorIndex);
  mlx90640SubPageMask[sensorIndex] = 0U;

  return MLX90640_NO_ERROR;
}

int MLX90640_App_ServiceHealth(void)
{
  for (uint8_t sensorIndex = 0U; sensorIndex < MLX90640_SENSOR_COUNT; sensorIndex++)
  {
    if (mlx90640SensorReady[sensorIndex] != 0U)
    {
      continue;
    }

    int status = MLX90640_InitSensorOnChannel(sensorIndex);
    if (status != MLX90640_NO_ERROR)
    {
      g_MLX90640_Frame.LastError[sensorIndex] = status;
      g_MLX90640_Frame.ValidMask &= (uint8_t)(~(1U << sensorIndex));
      continue;
    }

    mlx90640SensorReady[sensorIndex] = 1U;
    g_MLX90640_Frame.LastError[sensorIndex] = 0;
  }

  return MLX90640_NO_ERROR;
}

uint8_t MLX90640_App_IsSensorReady(uint8_t sensorIndex)
{
  if (sensorIndex >= MLX90640_SENSOR_COUNT)
  {
    return 0U;
  }

  return mlx90640SensorReady[sensorIndex];
}

const float *MLX90640_App_GetTempMap(void)
{
  return mlx90640Runtime.TempMap[g_MLX90640_Frame.ActiveSensorId];
}

static int32_t MLX90640_TempToCentiC(float tempC)
{
  float scaledTemp = tempC * (float)MLX90640_TEMP_SCALE;

  if (scaledTemp >= 0.0f)
  {
    return (int32_t)(scaledTemp + 0.5f);
  }

  return (int32_t)(scaledTemp - 0.5f);
}

static void MLX90640_UpdateRegionTemp(uint8_t sensorIndex, const float *pixelTemp)
{
  float regionSum[MLX90640_REGION_COUNT] = {0.0f};
  uint16_t regionCount[MLX90640_REGION_COUNT] = {0U};
  const uint16_t rowsPerRegion = (uint16_t)(MLX90640_LINE_NUM / MLX90640_REGION_COUNT);

  for (uint16_t row = 0U; row < MLX90640_LINE_NUM; row++)
  {
    uint8_t regionIndex = (uint8_t)(row / rowsPerRegion);

    if (regionIndex >= MLX90640_REGION_COUNT)
    {
      regionIndex = (uint8_t)(MLX90640_REGION_COUNT - 1U);
    }

    for (uint16_t col = 0U; col < MLX90640_COLUMN_NUM; col++)
    {
      uint16_t pixelIndex = (uint16_t)(row * MLX90640_COLUMN_NUM + col);
      regionSum[regionIndex] += pixelTemp[pixelIndex];
      regionCount[regionIndex]++;
    }
  }

  for (uint8_t regionIndex = 0U; regionIndex < MLX90640_REGION_COUNT; regionIndex++)
  {
    if (regionCount[regionIndex] > 0U)
    {
      g_MLX90640_Frame.RegionTemp[sensorIndex][regionIndex] =
          MLX90640_TempToCentiC(regionSum[regionIndex] / (float)regionCount[regionIndex]);
    }
    else
    {
      g_MLX90640_Frame.RegionTemp[sensorIndex][regionIndex] = 0;
    }
  }
}
