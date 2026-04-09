#ifndef __MLX90640_APP_H
#define __MLX90640_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


// ------------------- MLX90640 config -------------------
#define MLX90640_ADDR            0x33
#define MLX90640_EMISSIVITY      0.95f

// 初始化 MLX90640（读取EEPROM并提取参数，设置默认刷新率/模式）
void MLX90640_App_Init(void);

// 采集一帧并计算温度数组（当前实现会写入全局帧缓冲）
// 返回 0 成功，非0失败
int MLX90640_App_CaptureOnce(uint8_t sensor_id);

// 将温度数组打包并通过 UART1 DMA 发送：第1字节为sensor_id，后续为768个float(小端)
// 返回 0 成功启动发送，非0表示忙或失败
int MLX90640_App_UartSendFrame_DMA(uint8_t sensor_id);

// 查询 UART1 DMA 是否空闲（1=空闲可发送，0=忙）
uint8_t MLX90640_App_UartTxIdle(void);

#ifdef __cplusplus
}
#endif

#endif /* __MLX90640_APP_H */
