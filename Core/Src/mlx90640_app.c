#include "mlx90640_app.h"

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#include "i2c.h"
#include "usart.h"
#include "battery.h"
#include "cmsis_os.h"
#include "main.h"

#include <string.h>

// ------------------- MLX90640 config -------------------
#define MLX90640_ADDR            0x33
#define MLX90640_EMISSIVITY      0.95f

// Reflected temperature (Tr) approximation: use Ta
static float g_tr = 23.15f;

static paramsMLX90640 g_params;
static uint16_t g_eeData[MLX90640_EEPROM_DUMP_NUM];
static uint16_t g_frameData[834];

// ------------------- UART DMA state -------------------
static volatile uint8_t s_uart1_tx_busy = 0;

uint8_t MLX90640_App_UartTxIdle(void)
{
    return (s_uart1_tx_busy == 0);
}

void MLX90640_App_Init(void)
{
    // I2C peripheral is already initialized by CubeMX: [`Core/Src/i2c.c`](Core/Src/i2c.c:32)
    // UART1 peripheral is already initialized by CubeMX: [`Core/Src/usart.c`](Core/Src/usart.c:32)

    // Dump EEPROM and extract calibration parameters
    (void)MLX90640_I2CInit();

    if (MLX90640_DumpEE(MLX90640_ADDR, g_eeData) == MLX90640_NO_ERROR)
    {
        (void)MLX90640_ExtractParameters(g_eeData, &g_params);

        // Keep default mode from sensor; optionally set refresh rate here.
        // Example: 4Hz => value per datasheet mapping, but we keep default to be safe.
        // (void)MLX90640_SetChessMode(MLX90640_ADDR);
        // (void)MLX90640_SetRefreshRate(MLX90640_ADDR, 0x02);
    }
}

int MLX90640_App_CaptureOnce(void)
{
    int status = MLX90640_GetFrameData(MLX90640_ADDR, g_frameData);
    if (status != MLX90640_NO_ERROR)
        return status;

    float ta = MLX90640_GetTa(g_frameData, &g_params);
    g_tr = ta - 8.0f; // typical approximation: Tr = Ta - 8

    MLX90640_CalculateTo(g_frameData, &g_params, MLX90640_EMISSIVITY, g_tr, g_MLX90640_Frame.To);

    // optional: bad pixel correction
    MLX90640_BadPixelsCorrection(g_params.brokenPixels, g_MLX90640_Frame.To, 1, &g_params);
    MLX90640_BadPixelsCorrection(g_params.outlierPixels, g_MLX90640_Frame.To, 1, &g_params);

    return 0;
}

int MLX90640_App_UartSendFrame_DMA(void)
{
    if (s_uart1_tx_busy)
        return -1;

    // Pack float array as bytes (little-endian, STM32 is little-endian)
    memcpy(g_MLX90640_Frame.UartTxBuf, (uint8_t*)g_MLX90640_Frame.To, sizeof(g_MLX90640_Frame.To));

    // MAX485: 发送前切到“写”(驱动使能)
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);

    s_uart1_tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart1, g_MLX90640_Frame.UartTxBuf, (uint16_t)sizeof(g_MLX90640_Frame.To)) != HAL_OK)
    {
        s_uart1_tx_busy = 0;
        // 失败则立即回到“读”(接收)
        HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
        return -2;
    }

    return 0;
}

// HAL callback: clear busy flag
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // DMA发送完成：延时10ms后切回“读”(接收)模式
        osDelay(10);
        HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);

        s_uart1_tx_busy = 0;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 出错：立即回到“读”(接收)模式，避免总线被占用
        HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
        s_uart1_tx_busy = 0;
    }
}
