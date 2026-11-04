#ifndef MODEM_TRANSPORT_H
#define MODEM_TRANSPORT_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Single task owns Init/Send. Send consumes a protobuf payload synchronously.
 * modem_at.c supplies AT/TCP/MQTT framing. A future DTU backend must implement
 * these same functions and replace modem_at.c in the build (not coexist).
 * OnRxEvent is called from the UART1 DMA/IDLE ISR callback. */
void Modem_Init(void);
void Modem_AssumeReady(void);
uint8_t Modem_IsReady(void);
HAL_StatusTypeDef Modem_Send(const uint8_t *payload, uint16_t length);
void Modem_OnRxEvent(uint16_t size);
#ifdef __cplusplus
}
#endif
#endif
