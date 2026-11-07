/* Generated from vehicle-interfaces 27a5f677970dd9729e445b077fd8169809b34113. */
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
 case 0x301: index=0; length=8; break;
 case 0x302: index=1; length=6; break;
 case 0x303: index=2; length=7; break;
 case 0x304: index=3; length=8; break;
 case 0x125: index=4; length=8; break;
 case 0x132: index=5; length=2; break;
 case 0x270: index=6; length=5; break;
 case 0x17E: index=7; length=5; break;
 case 0x0AB: index=8; length=7; break;
 case 0x166: index=9; length=8; break;
 default:return;
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
 taskENTER_CRITICAL();c=cache[0];taskEXIT_CRITICAL();
 if(c.count || c.rejected){
 frame->has_canc=true;frame->canc.has_cdc_info=true;
 fsae_CancCdcInfo *m=&frame->canc.cdc_info;m->has_status=true;MapStatus(&m->status,&c,8,now);
 m->status.has_out_of_range_mask=m->status.has_unknown_enum_mask=c.count!=0;
 if(c.count){int32_t v=ReadMotorola(c.data,7,12,0);
 double physical=(double)v*0.0001+(-0.2);
 if(physical < -0.2-1e-8 || physical > 0.2095+1e-8)m->status.out_of_range_mask|=1UL<<0;
 if(m->status.fresh){m->has_CDC_HeiFL=true;m->CDC_HeiFL=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,11,4,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 15.0+1e-8)m->status.out_of_range_mask|=1UL<<1;
 if(v>=5)m->status.unknown_enum_mask|=1UL<<1;
 if(m->status.fresh){m->has_CDC_Mod=true;m->CDC_Mod=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,23,12,0);
 double physical=(double)v*0.0001+(-0.2);
 if(physical < -0.2-1e-8 || physical > 0.2095+1e-8)m->status.out_of_range_mask|=1UL<<2;
 if(m->status.fresh){m->has_CDC_HeiFR=true;m->CDC_HeiFR=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,27,2,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 3.0+1e-8)m->status.out_of_range_mask|=1UL<<3;
 if(m->status.fresh){m->has_CDC_OprlSts=true;m->CDC_OprlSts=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,39,12,0);
 double physical=(double)v*0.0001+(-0.2);
 if(physical < -0.2-1e-8 || physical > 0.2095+1e-8)m->status.out_of_range_mask|=1UL<<4;
 if(m->status.fresh){m->has_CDC_HeiRL=true;m->CDC_HeiRL=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,55,12,0);
 double physical=(double)v*0.0001+(-0.2);
 if(physical < -0.2-1e-8 || physical > 0.2095+1e-8)m->status.out_of_range_mask|=1UL<<5;
 if(m->status.fresh){m->has_CDC_HeiRR=true;m->CDC_HeiRR=v;} }
 }
 taskENTER_CRITICAL();c=cache[1];taskEXIT_CRITICAL();
 if(c.count || c.rejected){
 frame->has_canc=true;frame->canc.has_cdc_acceleration=true;
 fsae_CancCdcAcceleration *m=&frame->canc.cdc_acceleration;m->has_status=true;MapStatus(&m->status,&c,6,now);
 m->status.has_out_of_range_mask=m->status.has_unknown_enum_mask=c.count!=0;
 if(c.count){int32_t v=ReadMotorola(c.data,7,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<0;
 if(m->status.fresh){m->has_CDC_HeaveAccV=true;m->CDC_HeaveAccV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,6,12,0);
 double physical=(double)v*0.01+(-30.0);
 if(physical < -30.0-1e-8 || physical > 30.0+1e-8)m->status.out_of_range_mask|=1UL<<1;
 if(m->status.fresh){m->has_CDC_HeaveAcc=true;m->CDC_HeaveAcc=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,23,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<2;
 if(m->status.fresh){m->has_CDC_LatAccV=true;m->CDC_LatAccV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,22,12,0);
 double physical=(double)v*0.01+(-30.0);
 if(physical < -30.0-1e-8 || physical > 30.0+1e-8)m->status.out_of_range_mask|=1UL<<3;
 if(m->status.fresh){m->has_CDC_LatAcc=true;m->CDC_LatAcc=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,39,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<4;
 if(m->status.fresh){m->has_CDC_LgtAccV=true;m->CDC_LgtAccV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,38,12,0);
 double physical=(double)v*0.01+(-30.0);
 if(physical < -30.0-1e-8 || physical > 30.0+1e-8)m->status.out_of_range_mask|=1UL<<5;
 if(m->status.fresh){m->has_CDC_LgtAcc=true;m->CDC_LgtAcc=v;} }
 }
 taskENTER_CRITICAL();c=cache[2];taskEXIT_CRITICAL();
 if(c.count || c.rejected){
 frame->has_canc=true;frame->canc.has_cdc_rotation=true;
 fsae_CancCdcRotation *m=&frame->canc.cdc_rotation;m->has_status=true;MapStatus(&m->status,&c,7,now);
 m->status.has_out_of_range_mask=m->status.has_unknown_enum_mask=c.count!=0;
 if(c.count){int32_t v=ReadMotorola(c.data,7,16,0);
 double physical=(double)v*0.01+(-300.0);
 if(physical < -300.0-1e-8 || physical > 300.0+1e-8)m->status.out_of_range_mask|=1UL<<0;
 if(m->status.fresh){m->has_CDC_YawRate=true;m->CDC_YawRate=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,23,16,0);
 double physical=(double)v*0.01+(-300.0);
 if(physical < -300.0-1e-8 || physical > 300.0+1e-8)m->status.out_of_range_mask|=1UL<<1;
 if(m->status.fresh){m->has_CDC_PitchRate=true;m->CDC_PitchRate=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,39,16,0);
 double physical=(double)v*0.01+(-300.0);
 if(physical < -300.0-1e-8 || physical > 300.0+1e-8)m->status.out_of_range_mask|=1UL<<2;
 if(m->status.fresh){m->has_CDC_RollRate=true;m->CDC_RollRate=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,55,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<3;
 if(m->status.fresh){m->has_CDC_YawRateV=true;m->CDC_YawRateV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,54,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<4;
 if(m->status.fresh){m->has_CDC_PitchRateV=true;m->CDC_PitchRateV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,53,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<5;
 if(m->status.fresh){m->has_CDC_RollRateV=true;m->CDC_RollRateV=v;} }
 }
 taskENTER_CRITICAL();c=cache[3];taskEXIT_CRITICAL();
 if(c.count || c.rejected){
 frame->has_canc=true;frame->canc.has_cdc_current=true;
 fsae_CancCdcCurrent *m=&frame->canc.cdc_current;m->has_status=true;MapStatus(&m->status,&c,8,now);
 m->status.has_out_of_range_mask=m->status.has_unknown_enum_mask=c.count!=0;
 if(c.count){int32_t v=ReadMotorola(c.data,7,8,0);
 double physical=(double)v*7.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1785.0+1e-8)m->status.out_of_range_mask|=1UL<<0;
 if(m->status.fresh){m->has_CDC_CurrCmprFL=true;m->CDC_CurrCmprFL=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,15,8,0);
 double physical=(double)v*7.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1785.0+1e-8)m->status.out_of_range_mask|=1UL<<1;
 if(m->status.fresh){m->has_CDC_CurrCmprFR=true;m->CDC_CurrCmprFR=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,23,8,0);
 double physical=(double)v*7.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1785.0+1e-8)m->status.out_of_range_mask|=1UL<<2;
 if(m->status.fresh){m->has_CDC_CurrCmprRL=true;m->CDC_CurrCmprRL=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,31,8,0);
 double physical=(double)v*7.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1785.0+1e-8)m->status.out_of_range_mask|=1UL<<3;
 if(m->status.fresh){m->has_CDC_CurrCmprRR=true;m->CDC_CurrCmprRR=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,39,8,0);
 double physical=(double)v*7.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1785.0+1e-8)m->status.out_of_range_mask|=1UL<<4;
 if(m->status.fresh){m->has_CDC_CurrRbndFL=true;m->CDC_CurrRbndFL=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,47,8,0);
 double physical=(double)v*7.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1785.0+1e-8)m->status.out_of_range_mask|=1UL<<5;
 if(m->status.fresh){m->has_CDC_CurrRbndFR=true;m->CDC_CurrRbndFR=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,55,8,0);
 double physical=(double)v*7.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1785.0+1e-8)m->status.out_of_range_mask|=1UL<<6;
 if(m->status.fresh){m->has_CDC_CurrRbndRL=true;m->CDC_CurrRbndRL=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,63,8,0);
 double physical=(double)v*7.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1785.0+1e-8)m->status.out_of_range_mask|=1UL<<7;
 if(m->status.fresh){m->has_CDC_CurrRbndRR=true;m->CDC_CurrRbndRR=v;} }
 }
 taskENTER_CRITICAL();c=cache[4];taskEXIT_CRITICAL();
 if(c.count || c.rejected){
 frame->has_canc=true;frame->canc.has_abs_status=true;
 fsae_CancAbsStatus *m=&frame->canc.abs_status;m->has_status=true;MapStatus(&m->status,&c,8,now);
 m->status.has_out_of_range_mask=m->status.has_unknown_enum_mask=c.count!=0;
 if(c.count){int32_t v=ReadMotorola(c.data,15,13,0);
 double physical=(double)v*0.05625+(0.0);
 if(physical < 0.0-1e-8 || physical > 460.63125+1e-8)m->status.out_of_range_mask|=1UL<<0;
 if(m->status.fresh){m->has_ABS_VehSpd=true;m->ABS_VehSpd=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,18,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<1;
 if(m->status.fresh){m->has_ABS_VehSpdV=true;m->ABS_VehSpdV=v;} }
 }
 taskENTER_CRITICAL();c=cache[5];taskEXIT_CRITICAL();
 if(c.count || c.rejected){
 frame->has_canc=true;frame->canc.has_esc_status=true;
 fsae_CancEscStatus *m=&frame->canc.esc_status;m->has_status=true;MapStatus(&m->status,&c,2,now);
 m->status.has_out_of_range_mask=m->status.has_unknown_enum_mask=c.count!=0;
 if(c.count){int32_t v=ReadMotorola(c.data,7,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<0;
 if(m->status.fresh){m->has_ESC_MstCylPV=true;m->ESC_MstCylPV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,6,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<1;
 if(m->status.fresh){m->has_ESC_TcsCtrlActv=true;m->ESC_TcsCtrlActv=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,5,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<2;
 if(m->status.fresh){m->has_ESC_EscCtrlActv=true;m->ESC_EscCtrlActv=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,4,12,0);
 double physical=(double)v*0.1+(-100.0);
 if(physical < -100.0-1e-8 || physical > 309.4+1e-8)m->status.out_of_range_mask|=1UL<<3;
 if(m->status.fresh){m->has_ESC_MstCylP=true;m->ESC_MstCylP=v;} }
 }
 taskENTER_CRITICAL();c=cache[6];taskEXIT_CRITICAL();
 if(c.count || c.rejected){
 frame->has_canc=true;frame->canc.has_ibs_info=true;
 fsae_CancIbsInfo *m=&frame->canc.ibs_info;m->has_status=true;MapStatus(&m->status,&c,5,now);
 m->status.has_out_of_range_mask=m->status.has_unknown_enum_mask=c.count!=0;
 if(c.count){int32_t v=ReadMotorola(c.data,7,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<0;
 if(m->status.fresh){m->has_AB_BrkPdlSwtStsV=true;m->AB_BrkPdlSwtStsV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,6,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<1;
 if(m->status.fresh){m->has_AB_BrkPdlSwtSts=true;m->AB_BrkPdlSwtSts=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,5,8,0);
 double physical=(double)v*0.4+(0.0);
 if(physical < 0.0-1e-8 || physical > 100.0+1e-8)m->status.out_of_range_mask|=1UL<<2;
 if(m->status.fresh){m->has_AB_BrkPdlPct=true;m->AB_BrkPdlPct=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,13,10,0);
 double physical=(double)v*0.1+(0.0);
 if(physical < 0.0-1e-8 || physical > 100.0+1e-8)m->status.out_of_range_mask|=1UL<<3;
 if(m->status.fresh){m->has_AB_BrkPdlTrvl=true;m->AB_BrkPdlTrvl=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,19,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<4;
 if(m->status.fresh){m->has_AB_AccPdlSwtSts=true;m->AB_AccPdlSwtSts=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,18,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<5;
 if(m->status.fresh){m->has_AB_AccPdlSwtStsV=true;m->AB_AccPdlSwtStsV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,17,8,0);
 double physical=(double)v*0.4+(0.0);
 if(physical < 0.0-1e-8 || physical > 100.0+1e-8)m->status.out_of_range_mask|=1UL<<6;
 if(m->status.fresh){m->has_AB_AccPdlPct=true;m->AB_AccPdlPct=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,25,10,0);
 double physical=(double)v*0.1+(0.0);
 if(physical < 0.0-1e-8 || physical > 100.0+1e-8)m->status.out_of_range_mask|=1UL<<7;
 if(m->status.fresh){m->has_AB_AccPdlTrvl=true;m->AB_AccPdlTrvl=v;} }
 }
 taskENTER_CRITICAL();c=cache[7];taskEXIT_CRITICAL();
 if(c.count || c.rejected){
 frame->has_canc=true;frame->canc.has_sas_sensor=true;
 fsae_CancSasSensor *m=&frame->canc.sas_sensor;m->has_status=true;MapStatus(&m->status,&c,5,now);
 m->status.has_out_of_range_mask=m->status.has_unknown_enum_mask=c.count!=0;
 if(c.count){int32_t v=ReadMotorola(c.data,7,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<0;
 if(m->status.fresh){m->has_SAS_SteerAgV=true;m->SAS_SteerAgV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,6,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<1;
 if(m->status.fresh){m->has_SAS_SASStsSnsr=true;m->SAS_SASStsSnsr=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,5,14,0);
 double physical=(double)v*0.125+(-1024.0);
 if(physical < -1024.0-1e-8 || physical > 1023.75+1e-8)m->status.out_of_range_mask|=1UL<<2;
 if(m->status.fresh){m->has_SAS_SteerRotSpd=true;m->SAS_SteerRotSpd=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,23,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<3;
 if(m->status.fresh){m->has_SAS_SteerRotSpdV=true;m->SAS_SteerRotSpdV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,31,16,0);
 double physical=(double)v*0.0238+(-780.0);
 if(physical < -780.0-1e-8 || physical > 779.733+1e-8)m->status.out_of_range_mask|=1UL<<4;
 if(m->status.fresh){m->has_SAS_SteerAg=true;m->SAS_SteerAg=v;} }
 }
 taskENTER_CRITICAL();c=cache[8];taskEXIT_CRITICAL();
 if(c.count || c.rejected){
 frame->has_canc=true;frame->canc.has_vcu_requests=true;
 fsae_CancVcuRequests *m=&frame->canc.vcu_requests;m->has_status=true;MapStatus(&m->status,&c,7,now);
 m->status.has_out_of_range_mask=m->status.has_unknown_enum_mask=c.count!=0;
 if(c.count){int32_t v=ReadMotorola(c.data,7,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<0;
 if(m->status.fresh){m->has_VCU_FMCU_TarTqReqV=true;m->VCU_FMCU_TarTqReqV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,6,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<1;
 if(m->status.fresh){m->has_VCU_RMCU_TarTqReqV=true;m->VCU_RMCU_TarTqReqV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,5,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<2;
 if(m->status.fresh){m->has_VCU_MCU_TarDrvgReqV=true;m->VCU_MCU_TarDrvgReqV=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,15,16,0);
 double physical=(double)v*0.1+(-400.0);
 if(physical < -400.0-1e-8 || physical > 400.0+1e-8)m->status.out_of_range_mask|=1UL<<3;
 if(m->status.fresh){m->has_VCU_FMCU_TarTqReq=true;m->VCU_FMCU_TarTqReq=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,31,16,0);
 double physical=(double)v*0.1+(-400.0);
 if(physical < -400.0-1e-8 || physical > 400.0+1e-8)m->status.out_of_range_mask|=1UL<<4;
 if(m->status.fresh){m->has_VCU_RMCU_TarTqReq=true;m->VCU_RMCU_TarTqReq=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,47,16,0);
 double physical=(double)v*1.0+(-5000.0);
 if(physical < -5000.0-1e-8 || physical > 5000.0+1e-8)m->status.out_of_range_mask|=1UL<<5;
 if(m->status.fresh){m->has_VCU_MCU_TarDrvgReq=true;m->VCU_MCU_TarDrvgReq=v;} }
 }
 taskENTER_CRITICAL();c=cache[9];taskEXIT_CRITICAL();
 if(c.count || c.rejected){
 frame->has_canc=true;frame->canc.has_vcu_status=true;
 fsae_CancVcuStatus *m=&frame->canc.vcu_status;m->has_status=true;MapStatus(&m->status,&c,8,now);
 m->status.has_out_of_range_mask=m->status.has_unknown_enum_mask=c.count!=0;
 if(c.count){int32_t v=ReadMotorola(c.data,7,1,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 1.0+1e-8)m->status.out_of_range_mask|=1UL<<0;
 if(m->status.fresh){m->has_VCU_EOLoffsetOvrdSwt=true;m->VCU_EOLoffsetOvrdSwt=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,6,3,0);
 double physical=(double)v*1.0+(0.0);
 if(physical < 0.0-1e-8 || physical > 7.0+1e-8)m->status.out_of_range_mask|=1UL<<1;
 if(v>=5)m->status.unknown_enum_mask|=1UL<<1;
 if(m->status.fresh){m->has_VCU_DrvModSts=true;m->VCU_DrvModSts=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,3,15,1);
 double physical=(double)v*0.1+(0.0);
 if(physical < -1000.0-1e-8 || physical > 1000.0+1e-8)m->status.out_of_range_mask|=1UL<<2;
 if(m->status.fresh){m->has_VCU_EOLoffsetFL=true;m->VCU_EOLoffsetFL=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,20,15,1);
 double physical=(double)v*0.1+(0.0);
 if(physical < -1000.0-1e-8 || physical > 1000.0+1e-8)m->status.out_of_range_mask|=1UL<<3;
 if(m->status.fresh){m->has_VCU_EOLoffsetFR=true;m->VCU_EOLoffsetFR=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,37,15,1);
 double physical=(double)v*0.1+(0.0);
 if(physical < -1000.0-1e-8 || physical > 1000.0+1e-8)m->status.out_of_range_mask|=1UL<<4;
 if(m->status.fresh){m->has_VCU_EOLoffsetRL=true;m->VCU_EOLoffsetRL=v;} }
 if(c.count){int32_t v=ReadMotorola(c.data,54,15,1);
 double physical=(double)v*0.1+(0.0);
 if(physical < -1000.0-1e-8 || physical > 1000.0+1e-8)m->status.out_of_range_mask|=1UL<<5;
 if(m->status.fresh){m->has_VCU_EOLoffsetRR=true;m->VCU_EOLoffsetRR=v;} }
 }
}
