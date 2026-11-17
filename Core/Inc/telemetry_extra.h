#ifndef TELEMETRY_EXTRA_H
#define TELEMETRY_EXTRA_H
#include "can.h"
#include "fsae_telemetry.pb.h"
#include "telemetry_updates.h"
TelemetryUpdates Extra_Receive(const CAN_Msg_Queue_t *frame);
void Extra_MapGroups(fsae_TelemetryFrame *frame, uint32_t now, TelemetryUpdates groups);
void Extra_Map(fsae_TelemetryFrame *frame, uint32_t now);
#endif
