#ifndef TELEMETRY_EXTRA_H
#define TELEMETRY_EXTRA_H
#include "can.h"
#include "fsae_telemetry.pb.h"
void Extra_Receive(const CAN_Msg_Queue_t *frame);
void Extra_Map(fsae_TelemetryFrame *frame, uint32_t now);
#endif
