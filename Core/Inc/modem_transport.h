#ifndef MODEM_TRANSPORT_H
#define MODEM_TRANSPORT_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Single publishing task owns Init/Send and the static DMA buffer.
 * Send adds AA55/length/sequence/CRC and waits for UART completion.
 * sequence must be the low byte of the protobuf frame_id/header.seq.
 * IsReady reports local UART readiness, not MQTT connectivity or delivery.
 * DTU owns MQTT; downlink commands are currently ignored. */
void Modem_Init(void);
uint8_t Modem_IsReady(void);
HAL_StatusTypeDef Modem_Send(const uint8_t *payload, uint16_t length, uint8_t sequence);
void Modem_OnRxEvent(uint16_t size);
#ifdef __cplusplus
}
#endif
#endif
