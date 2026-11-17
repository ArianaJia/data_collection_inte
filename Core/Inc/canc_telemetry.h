#ifndef CANC_TELEMETRY_H
#define CANC_TELEMETRY_H
#include "can.h"
#include "fsae_telemetry.pb.h"
#include "telemetry_updates.h"
TelemetryUpdates Canc_Receive(const CAN_Msg_Queue_t *frame);
void Canc_MapGroups(fsae_TelemetryFrame *frame, uint32_t now, TelemetryUpdates groups);
void Canc_Map(fsae_TelemetryFrame *frame, uint32_t now);
#endif
