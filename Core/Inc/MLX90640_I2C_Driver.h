/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef _MLX90640_I2C_Driver_H_
#define _MLX90640_I2C_Driver_H_

#include <stdint.h>
#include "MLX90640_API.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>

// 原有MLX90640宏定义（保留）
#define MLX90640_NO_ERROR 0
#define MLX90640_I2C_NACK_ERROR -1
#define MLX90640_FRAME_DATA_ERROR -8
#define MLX90640_EEPROM_START_ADDRESS 0x2400
#define MLX90640_PIXEL_NUM 768
#define MLX90640_AUX_NUM 64
#define MLX90640_EEPROM_DUMP_NUM 832

// 声明CubeIDE生成的I2C和DMA句柄（必须与i2c.c/dma.c一致）
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
#define MLX90640_I2C_HANDLE &hi2c1

// ------------------- DMA新增定义 -------------------
// DMA传输最大缓冲区（MLX90640最大读取834*2=1668字节，预留至2048字节）
#define MLX90640_DMA_BUF_SIZE 2048
// DMA传输完成/错误标志位（0：未完成，1：完成，-1：错误）
extern volatile int8_t mlx90640_dma_tx_flag;
extern volatile int8_t mlx90640_dma_rx_flag;
// DMA全局缓冲区（用于I2C收发）
extern uint8_t mlx90640_dma_buf[MLX90640_DMA_BUF_SIZE];

    extern void MLX90640_I2CInit(void);
    extern int MLX90640_I2CGeneralReset(void);

    // ------------------- TCA9548A I2C MUX -------------------
    // A0/A1/A2全部接地 => 7-bit地址 0x70
    #define TCA9548A_ADDR_7BIT 0x70
    // 选择通道：ch=0..7，传入0xFF表示不选择（关闭所有通道）
    extern int TCA9548A_SelectChannel(uint8_t ch);

    extern int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);
    extern int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data);
    extern void MLX90640_I2CFreqSet(int freq);

    // DMA中断回调函数声明
    void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
    void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif
