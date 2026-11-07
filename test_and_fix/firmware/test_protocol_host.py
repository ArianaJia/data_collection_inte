from pathlib import Path
import shutil,subprocess
out=Path('build/host_protocol');out.mkdir(parents=True,exist_ok=True)
for name in ['canc_telemetry.c','telemetry_extra.c']:
 shutil.copyfile(Path('Core/Src')/name,out/name)
for name in ['canc_telemetry.h','telemetry_extra.h','fsae_telemetry.pb.h','pb.h']:
 shutil.copyfile(Path('Core/Inc')/name,out/name)
(out/'can.h').write_text('''#pragma once
#include <stdint.h>
#define CAN_CHANNEL_CDC_MONITOR 4
#define CAN_CHANNEL_VEHICLE_CANB 2
#define CAN_CHANNEL_BATTERY_BOX 1
typedef struct {uint8_t can_channel;uint32_t msg_id;uint8_t msg_data[8];uint8_t dlc,is_ext_id;} CAN_Msg_Queue_t;
uint32_t HAL_GetTick(void);
''')
(out/'FreeRTOS.h').write_text('#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n')
(out/'task.h').write_text('')
(out/'test.c').write_text('''#include "canc_telemetry.h"
#include "telemetry_extra.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t tick;
uint32_t HAL_GetTick(void){return tick;}
static fsae_TelemetryFrame out;
int main(void){
 CAN_Msg_Queue_t f={.can_channel=4,.msg_id=0x301,.dlc=8,.msg_data={0x12,0x31,0x45,0x60,0x78,0x90,0xAB,0xC0}};
 Canc_Receive(&f);Canc_Map(&out,100);assert(out.canc.cdc_info.CDC_HeiFL==0x123);assert(out.canc.cdc_info.CDC_HeiFR==0x456);assert(out.canc.cdc_info.status.fresh);assert(out.canc.cdc_info.status.received_ms==0);
 memset(&out,0,sizeof(out));Canc_Map(&out,101);assert(!out.canc.cdc_info.has_CDC_HeiFL);assert(!out.canc.cdc_info.status.fresh);
 f.dlc=7;Canc_Receive(&f);memset(&out,0,sizeof(out));Canc_Map(&out,101);assert(out.canc.cdc_info.status.rejected_count==1);assert(out.canc.cdc_info.status.rx_count==1);
 f.can_channel=2;f.msg_id=0x5A2;f.dlc=8;memset(f.msg_data,0,8);Extra_Receive(&f);Extra_Map(&out,0);assert(out.has_fan_telemetry && out.fan_telemetry.has_fan1_rpm && out.fan_telemetry.fan1_rpm==0);
 memset(&out,0,sizeof(out));Extra_Map(&out,2001);assert(!out.has_fan_telemetry);
 f.msg_id=0x5AA;f.msg_data[2]=50;Extra_Receive(&f);memset(&out,0,sizeof(out));Extra_Map(&out,0);assert(out.battery_fan.actual_duty_pct_x10==500);
 f.can_channel=1;f.is_ext_id=1;f.msg_id=0x186E50F4;f.msg_data[2]=0;f.msg_data[3]=123;Extra_Receive(&f);memset(&out,0,sizeof(out));Extra_Map(&out,0);assert(out.battery_fan.actual_duty_pct_x10==123);
 puts("PASS: CANC Motorola, exact DLC, tick zero, freshness; fan zero, timeout and source priority");
}
''')
import argparse
parser=argparse.ArgumentParser()
parser.add_argument('--cc', default='gcc')
cc=parser.parse_args().cc
subprocess.run([cc,'-std=c99','-I'+str(out),str(out/'test.c'),str(out/'canc_telemetry.c'),str(out/'telemetry_extra.c'),'-o',str(out/'test.exe')],check=True)
subprocess.run([str(out/'test.exe')],check=True)
