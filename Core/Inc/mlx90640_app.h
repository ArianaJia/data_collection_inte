#ifndef __MLX90640_APP_H
#define __MLX90640_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 初始化 MLX90640（读取EEPROM并提取参数，设置默认刷新率/模式）
void MLX90640_App_Init(void);

// 采集一帧并计算温度数组 g_MLX90640_To[768]
// 返回 0 成功，非0失败
int MLX90640_App_CaptureOnce(void);

// 将 g_MLX90640_To 打包到 g_MLX90640_UartTxBuf 并通过 UART1 DMA 发送
// 返回 0 成功启动发送，非0表示忙或失败
int MLX90640_App_UartSendFrame_DMA(void);

// 查询 UART1 DMA 是否空闲（1=空闲可发送，0=忙）
uint8_t MLX90640_App_UartTxIdle(void);

#ifdef __cplusplus
}
#endif

#endif /* __MLX90640_APP_H */
