#ifndef __MLX90640_APP_H
#define __MLX90640_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MLX90640_ADDR            0x33U
#define MLX90640_EMISSIVITY      0.95f
#define MLX90640_FRAME_ATTEMPTS_MAX 4U
#define MLX90640_LOG_BUFFER_SIZE    160U
#define MLX90640_APP_LOCK_TIMEOUT_MS 50U

typedef void (*MLX90640_AppLogFn)(const char *text);

void MLX90640_App_SetLogCallback(MLX90640_AppLogFn callback);
int MLX90640_App_Init(void);
int MLX90640_App_CaptureOnce(uint8_t sensor_id);
int MLX90640_App_ServiceHealth(void);
uint8_t MLX90640_App_IsSensorReady(uint8_t sensor_id);
const float *MLX90640_App_GetTempMap(void);

#ifdef __cplusplus
}
#endif

#endif
