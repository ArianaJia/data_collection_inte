#ifndef __MLX90640_I2C_DRIVER_H_
#define __MLX90640_I2C_DRIVER_H_

#include <stdint.h>
#include "MLX90640_API.h"
#include "stm32f4xx_hal.h"

#define MLX90640_DMA_BUF_SIZE          2048U
#define MLX90640_I2C_TIMEOUT_MS        100U
#define MLX90640_I2C_READ_CHUNK_WORDS  32U
#define MLX90640_I2C_DMA_TIMEOUT_MS    100U
#define MLX90640_FRAME_READY_TIMEOUT_MS 250U
#ifndef MLX90640_FRAME_NOT_READY_ERROR
#define MLX90640_FRAME_NOT_READY_ERROR   9U
#endif
#ifndef MLX90640_ADDR
#define MLX90640_ADDR       (0x33 << 1)
#endif

#define TCA9548A_ADDR           (0x70 << 1)
#define MLX90640_SENSOR_NUM            4U
#define MLX90640_TA_SHIFT              8.0f
#define MLX90640_REF_0P5HZ             0x00U
#define MLX90640_REF_1HZ               MLX90640_REF_0P5HZ

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;

void MLX90640_I2CInit(void);
int MLX90640_I2CGeneralReset(void);
int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);
int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data);
int MLX90640_I2CProbe(uint8_t slaveAddr);
void MLX90640_I2CFreqSet(int freq);

int TCA9548A_SelectChannel(uint8_t channel);
int MLX90640_SelectSensor(uint8_t sensorIndex);
uint8_t MLX90640_GetSensorChannel(uint8_t sensorIndex);
HAL_StatusTypeDef TCA9548A_GetLastHalStatus(void);
uint32_t TCA9548A_GetLastErrorCode(void);
HAL_StatusTypeDef MLX90640_GetLastHalStatus(void);
uint32_t MLX90640_GetLastErrorCode(void);

#endif

