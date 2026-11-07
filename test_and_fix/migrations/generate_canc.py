from pathlib import Path
import re
import argparse
parser=argparse.ArgumentParser(description='Generate CANC decoder from pinned vehicle-interfaces DBC and local proto')
parser.add_argument('--interfaces', type=Path, required=True)
root=parser.parse_args().interfaces
dbc=(root/'can/Vehicle_CanC.dbc').read_text(encoding='utf-8')
proto=Path('test_and_fix/communication/fsae_telemetry.proto').read_text(encoding='utf-8')
msgs={}
for b in re.split(r'(?m)^BO_ ',dbc)[1:]:
 h=re.match(r'(\d+) (\w+): (\d+)',b)
 if not h: continue
 sig={m[0]:(int(m[1]),int(m[2]),m[3],float(m[4]),float(m[5]),float(m[6]),float(m[7])) for m in re.findall(r'SG_ (\w+) : (\d+)\|(\d+)@0([+-]) \(([^,]+),([^\)]+)\) \[([^|]+)\|([^\]]+)\]',b)}
 msgs[int(h[1])]=(int(h[3]),sig)
links=re.findall(r'    (Canc\w+) (\w+) = \d+;',proto[proto.index('message CancTelemetry'):proto.index('message RegenTelemetry')])
head='''#ifndef CANC_TELEMETRY_H
#define CANC_TELEMETRY_H
#include "can.h"
#include "fsae_telemetry.pb.h"
void Canc_Receive(const CAN_Msg_Queue_t *frame);
void Canc_Map(fsae_TelemetryFrame *frame, uint32_t now);
#endif
'''
src='''/* Generated from vehicle-interfaces 27a5f677970dd9729e445b077fd8169809b34113. */
#include "canc_telemetry.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
typedef struct { uint32_t tick, interval, count, rejected; uint8_t data[8]; } CancCache;
static CancCache cache[10];
static int32_t ReadMotorola(const uint8_t *data, unsigned bit, unsigned width, unsigned sign) {
 uint32_t value=0;
 for(unsigned i=0;i<width;i++){value=(value<<1)|((data[bit/8]>>(bit%8))&1U);bit=(bit%8)?bit-1:bit+15;}
 return sign && (value & (1UL<<(width-1))) ? (int32_t)value-(int32_t)(1UL<<width) : (int32_t)value;
}
void Canc_Receive(const CAN_Msg_Queue_t *frame) {
 if(!frame || frame->can_channel!=CAN_CHANNEL_CDC_MONITOR || frame->is_ext_id) return;
 unsigned index, length;
 switch(frame->msg_id){
'''
entries=[]
for i,(typ,name) in enumerate(links):
 body=re.search(r'message '+typ+r' \{(.*?)\n}',proto,re.S)[1]
 ident=int(re.search(r'CANC 0x([0-9A-Fa-f]+)',body)[1],16)
 length,sig=msgs[ident];entries.append((typ,name,length,sig,body))
 src+=f' case 0x{ident:03X}: index={i}; length={length}; break;\n'
src+=''' default:return;
 }
 taskENTER_CRITICAL();
 CancCache *c=&cache[index];
 if(frame->dlc!=length){c->rejected++;taskEXIT_CRITICAL();return;}
 uint32_t now=HAL_GetTick(); c->interval=now-c->tick;c->tick=now;c->count++;
 memcpy(c->data,frame->msg_data,length);
 taskEXIT_CRITICAL();
}
static void MapStatus(fsae_CancFrameStatus *s,const CancCache *c,unsigned length,uint32_t now){
 s->has_seen=s->has_fresh=s->has_rx_count=s->has_rejected_count=true;
 s->seen=c->count!=0;s->fresh=c->count && (uint32_t)(now-c->tick)<=100U;
 s->rx_count=c->count;s->rejected_count=c->rejected;
 if(c->count){s->has_received_ms=s->has_age_ms=true;s->received_ms=c->tick;s->age_ms=now-c->tick;
 s->has_interval_ms=c->count>1;s->interval_ms=c->interval;s->raw_payload.size=length;memcpy(s->raw_payload.bytes,c->data,length);}
}
void Canc_Map(fsae_TelemetryFrame *frame,uint32_t now){
 CancCache c;
'''
for i,(typ,name,length,sig,body) in enumerate(entries):
 src+=f''' taskENTER_CRITICAL();c=cache[{i}];taskEXIT_CRITICAL();
 if(c.count || c.rejected){{
 frame->has_canc=true;frame->canc.has_{name}=true;
 fsae_{typ} *m=&frame->canc.{name};m->has_status=true;MapStatus(&m->status,&c,{length},now);
 m->status.has_out_of_range_mask=m->status.has_unknown_enum_mask=c.count!=0;
'''
 for field,num in re.findall(r'optional [us]int32 (\w+) = (\d+);',body):
  bit,w,sign,scale,offset,lo,hi=sig[field]
  src+=f''' if(c.count){{int32_t v=ReadMotorola(c.data,{bit},{w},{int(sign=='-')});
 double physical=(double)v*{scale!r}+({offset!r});
 if(physical < {lo!r}-1e-8 || physical > {hi!r}+1e-8)m->status.out_of_range_mask|=1UL<<{int(num)-1};
'''
  if field in ('CDC_Mod','VCU_DrvModSts'):src+=f' if(v>=5)m->status.unknown_enum_mask|=1UL<<{int(num)-1};\n'
  src+=f' if(m->status.fresh){{m->has_{field}=true;m->{field}=v;}} }}\n'
 src+=' }\n'
src+='}\n'
Path('Core/Inc/canc_telemetry.h').write_text(head)
Path('Core/Src/canc_telemetry.c').write_text(src)
print('Generated',len(entries),'CANC frames')
