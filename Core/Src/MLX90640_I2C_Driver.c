/**
 * MLX90640 I2C driver (STM32 HAL) - DMA based implementation
 *
 * This file is missing in the current project, but the API header
 * [`Core/Inc/MLX90640_I2C_Driver.h`](Core/Inc/MLX90640_I2C_Driver.h:1) expects these functions.
 *
 * Implementation strategy:
 * - Use HAL_I2C_Master_Transmit_DMA + HAL_I2C_Master_Receive_DMA
 * - Perform a "register address write" (2 bytes) followed by repeated-start receive
 * - Use completion flags set in HAL_I2C callbacks
 * - Provide a blocking wait with timeout (safe for RTOS tasks)
 */

#include "MLX90640_I2C_Driver.h"
#include "cmsis_os.h"
#include <string.h>

// ------------------- DMA global objects declared in header -------------------
volatile int8_t mlx90640_dma_tx_flag = 0;
volatile int8_t mlx90640_dma_rx_flag = 0;
uint8_t mlx90640_dma_buf[MLX90640_DMA_BUF_SIZE];

// ------------------- local config -------------------
#ifndef MLX90640_I2C_TIMEOUT_MS
#define MLX90640_I2C_TIMEOUT_MS 50U
#endif

static int mlx_wait_flag(volatile int8_t *flag, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms)
    {
        if (*flag == 1)
            return 0;
        if (*flag < 0)
            return -1;
        // give time to other tasks/ISRs
        osDelay(1);
    }
    return -1;
}

void MLX90640_I2CInit(void)
{
    // I2C is initialized by CubeMX: [`Core/Src/i2c.c`](Core/Src/i2c.c:1)
    // Nothing to do here.
}

int MLX90640_I2CGeneralReset(void)
{
    // General Call Reset (0x00) + 0x06
    uint8_t reset_cmd = 0x06;

    mlx90640_dma_tx_flag = 0;
    mlx90640_dma_rx_flag = 0;

    if (HAL_I2C_Master_Transmit_DMA(MLX90640_I2C_HANDLE, 0x00, &reset_cmd, 1) != HAL_OK)
        return -MLX90640_I2C_NACK_ERROR;

    if (mlx_wait_flag(&mlx90640_dma_tx_flag, MLX90640_I2C_TIMEOUT_MS) != 0)
        return -MLX90640_I2C_NACK_ERROR;

    return MLX90640_NO_ERROR;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    if (data == NULL)
        return -MLX90640_FRAME_DATA_ERROR;

    // Each memory word is 16-bit
    uint32_t rx_bytes = (uint32_t)nMemAddressRead * 2U;
    if (rx_bytes + 2U > MLX90640_DMA_BUF_SIZE)
        return -MLX90640_FRAME_DATA_ERROR;

    // Prepare 2-byte register address (big endian)
    mlx90640_dma_buf[0] = (uint8_t)((startAddress >> 8) & 0xFF);
    mlx90640_dma_buf[1] = (uint8_t)(startAddress & 0xFF);

    mlx90640_dma_tx_flag = 0;
    mlx90640_dma_rx_flag = 0;

    // Write register address
    if (HAL_I2C_Master_Transmit_DMA(MLX90640_I2C_HANDLE, (uint16_t)(slaveAddr << 1), mlx90640_dma_buf, 2) != HAL_OK)
        return -MLX90640_I2C_NACK_ERROR;

    if (mlx_wait_flag(&mlx90640_dma_tx_flag, MLX90640_I2C_TIMEOUT_MS) != 0)
        return -MLX90640_I2C_NACK_ERROR;

    // Receive data payload into same DMA buffer
    if (HAL_I2C_Master_Receive_DMA(MLX90640_I2C_HANDLE, (uint16_t)(slaveAddr << 1), &mlx90640_dma_buf[0], (uint16_t)rx_bytes) != HAL_OK)
        return -MLX90640_I2C_NACK_ERROR;

    if (mlx_wait_flag(&mlx90640_dma_rx_flag, MLX90640_I2C_TIMEOUT_MS) != 0)
        return -MLX90640_I2C_NACK_ERROR;

    // Convert bytes -> uint16_t words (big endian)
    for (uint16_t i = 0; i < nMemAddressRead; i++)
    {
        uint16_t msb = mlx90640_dma_buf[(uint32_t)i * 2U + 0U];
        uint16_t lsb = mlx90640_dma_buf[(uint32_t)i * 2U + 1U];
        data[i] = (uint16_t)((msb << 8) | lsb);
    }

    return MLX90640_NO_ERROR;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    // Payload: addr_hi addr_lo data_hi data_lo
    uint8_t buf[4];
    buf[0] = (uint8_t)((writeAddress >> 8) & 0xFF);
    buf[1] = (uint8_t)(writeAddress & 0xFF);
    buf[2] = (uint8_t)((data >> 8) & 0xFF);
    buf[3] = (uint8_t)(data & 0xFF);

    mlx90640_dma_tx_flag = 0;
    mlx90640_dma_rx_flag = 0;

    if (HAL_I2C_Master_Transmit_DMA(MLX90640_I2C_HANDLE, (uint16_t)(slaveAddr << 1), buf, 4) != HAL_OK)
        return -MLX90640_I2C_NACK_ERROR;

    if (mlx_wait_flag(&mlx90640_dma_tx_flag, MLX90640_I2C_TIMEOUT_MS) != 0)
        return -MLX90640_I2C_NACK_ERROR;

    return MLX90640_NO_ERROR;
}

void MLX90640_I2CFreqSet(int freq)
{
    // Project already configures I2C to 400k in [`Core/Src/i2c.c`](Core/Src/i2c.c:32)
    // If you need dynamic frequency switching, regenerate CubeMX or re-init I2C here.
    (void)freq;
}

// ------------------- HAL callbacks: set flags -------------------
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == MLX90640_I2C_HANDLE)
        mlx90640_dma_tx_flag = 1;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == MLX90640_I2C_HANDLE)
        mlx90640_dma_rx_flag = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == MLX90640_I2C_HANDLE)
    {
        mlx90640_dma_tx_flag = -1;
        mlx90640_dma_rx_flag = -1;
    }
}
