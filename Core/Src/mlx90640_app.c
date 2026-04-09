#include "mlx90640_app.h"

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#include "i2c.h"
#include "usart.h"
#include "battery.h"
#include "cmsis_os.h"
#include "main.h"
#include "gpio.h"
#include <string.h>

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

    // Init low-level driver
    (void)MLX90640_I2CInit();

    // TCA9548A：默认先选CH0（避免总线悬空）
    (void)TCA9548A_SelectChannel(0);

    // 仅需初始化一次参数：4个MLX90640地址相同(0x33)，但参数可能略有差异。
    // 这里先按“同型号同参数”处理：从CH0读取EEPROM并提取参数。
    // 如你希望每路独立参数，需要扩展为 params[4] + eeData[4]。
    if (MLX90640_DumpEE(MLX90640_ADDR, g_eeData) == MLX90640_NO_ERROR)
    {
        (void)MLX90640_ExtractParameters(g_eeData, &g_params);
    }
}

int MLX90640_App_CaptureOnce(uint8_t sensor_id)
{
    // 通过TCA9548A选择对应通道：sensor_id 0..3 => CH0..CH3
    if (sensor_id > 3)
        return -1;
    if (TCA9548A_SelectChannel(sensor_id) != 0)
        return -2;

    int status = MLX90640_GetFrameData(MLX90640_ADDR, g_frameData);
    if (status != MLX90640_NO_ERROR)
        return status;

    float ta = MLX90640_GetTa(g_frameData, &g_params);
    g_tr = ta - 8.0f; // typical approximation: Tr = Ta - 8

    MLX90640_CalculateTo(g_frameData, &g_params, MLX90640_EMISSIVITY, g_tr, g_MLX90640_Frame.To[sensor_id]);

    // optional: bad pixel correction
    MLX90640_BadPixelsCorrection(g_params.brokenPixels, g_MLX90640_Frame.To[sensor_id], 1, &g_params);
    MLX90640_BadPixelsCorrection(g_params.outlierPixels, g_MLX90640_Frame.To[sensor_id], 1, &g_params);

    g_MLX90640_Frame.Ta[sensor_id] = ta;
    g_MLX90640_Frame.Tr[sensor_id] = g_tr;
    g_MLX90640_Frame.LastError[sensor_id] = 0;
    g_MLX90640_Frame.FrameCounter[sensor_id]++;
    g_MLX90640_Frame.ActiveSensorId = sensor_id;
    g_MLX90640_Frame.ValidMask |= (uint8_t)(1U << sensor_id);
    g_MLX90640_Frame.FrameState = MLX90640_FRAME_STATE_READY;

    return 0;
}

int MLX90640_App_UartSendFrame_DMA(uint8_t sensor_id)
{
    if (s_uart1_tx_busy)
        return -1;

    // 第1字节：sensor_id；后续：768个float
    g_MLX90640_Frame.UartTxBuf[0] = sensor_id;
    memcpy(&g_MLX90640_Frame.UartTxBuf[1], (uint8_t *)g_MLX90640_Frame.To[sensor_id], sizeof(g_MLX90640_Frame.To[sensor_id]));

    // MAX485: 发送前切到“写”(驱动使能)
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);

    g_MLX90640_Frame.TxSensorId = sensor_id;
    g_MLX90640_Frame.UartTxLen = MLX90640_UART_FRAME_BYTES;
    g_MLX90640_Frame.FrameState = MLX90640_FRAME_STATE_BUSY;
    s_uart1_tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart1, g_MLX90640_Frame.UartTxBuf, g_MLX90640_Frame.UartTxLen) != HAL_OK)
    {
        g_MLX90640_Frame.FrameState = MLX90640_FRAME_STATE_READY;
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

        g_MLX90640_Frame.FrameState = MLX90640_FRAME_STATE_IDLE;
        s_uart1_tx_busy = 0;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 出错：立即回到“读”(接收)模式，避免总线被占用
        HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
        g_MLX90640_Frame.FrameState = MLX90640_FRAME_STATE_READY;
        s_uart1_tx_busy = 0;
    }
}
