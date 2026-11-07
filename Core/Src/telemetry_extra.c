/* Additional protocol domains, pinned to vehicle-interfaces 27a5f67. */
#include "telemetry_extra.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <math.h>
typedef struct { uint32_t tick; uint8_t seen, data[8]; } Sample;
enum { PDM_BUS, PDM_BAT, FAN, BAT_FAN1, BAT_FANB, CH_V, CH_I, CH_P, CH_O, SOP_P, SOP_I, SOP_ACK, SOP_MIRROR, BMS_RELAYS, BMS_HV, IMD, SAMPLE_COUNT };
static Sample samples[SAMPLE_COUNT];
static unsigned be16(const uint8_t *p){return ((unsigned)p[0]<<8)|p[1];}
static unsigned le16(const uint8_t *p){return ((unsigned)p[1]<<8)|p[0];}
static uint32_t le32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static float fl32(const uint8_t *p){uint32_t u=le32(p);float f;memcpy(&f,&u,4);return f;}
static uint8_t crc(unsigned id,const uint8_t *p,unsigned n){uint8_t c=255;for(unsigned i=0;i<n+2;i++){c^=i==0?id>>8:i==1?id:p[i-2];for(unsigned j=0;j<8;j++)c=(c&128)?(c<<1)^0x1D:c<<1;}return c^255;}
void Extra_Receive(const CAN_Msg_Queue_t *f){
 if(!f || f->dlc>8)return;
 int index=-1;unsigned min=8;
 if(f->can_channel==CAN_CHANNEL_VEHICLE_CANB && !f->is_ext_id){switch(f->msg_id){
 case 0x5A0:index=PDM_BUS;break;case 0x5A1:index=PDM_BAT;break;case 0x5A2:index=FAN;break;
 case 0x5AA:if(f->msg_data[2]>100)return;index=BAT_FANB;break;
 case 0x201:index=CH_V;min=7;break;case 0x202:index=CH_I;min=7;break;
 case 0x204:index=CH_P;min=7;break;case 0x205:index=CH_O;min=4;break;
 case 0x4A0:if(f->dlc!=8 || f->msg_data[6]>>6!=3 || crc(0x4A0,f->msg_data,7)!=f->msg_data[7])return;index=SOP_P;break;
 case 0x4A3:if(f->msg_data[7]>>4!=3)return;index=SOP_I;break;
 case 0x4A4:if(f->dlc!=4 || f->msg_data[1]!=3 || (f->msg_data[2]&0xFE) || crc(0x4A4,f->msg_data,3)!=f->msg_data[3])return;index=SOP_ACK;min=4;break;
 default:return;}}
 else if(f->can_channel==CAN_CHANNEL_BATTERY_BOX && f->is_ext_id){switch(f->msg_id){
 case 0x186E50F4:if(be16(f->msg_data+2)>1000)return;index=BAT_FAN1;break;
 case 0x186A50F4:index=SOP_MIRROR;break;case 0x186350F4:index=BMS_RELAYS;break;
 case 0x186950F4:index=BMS_HV;break;case 0x186850F4:index=IMD;break;default:return;}}
 if(index<0 || f->dlc<min || (index!=CH_V && index!=CH_I && index!=CH_P && index!=CH_O && f->dlc!=min))return;
 if((index==CH_V || index==CH_I) && (!isfinite(fl32(f->msg_data+3)) || fl32(f->msg_data+3)<0))return;
 taskENTER_CRITICAL();
 if(index==SOP_P && samples[index].seen && (uint32_t)(HAL_GetTick()-samples[index].tick)<=2000U && (uint8_t)(f->msg_data[4]-samples[index].data[4])==0){taskEXIT_CRITICAL();return;}
 samples[index].tick=HAL_GetTick();samples[index].seen=1;memcpy(samples[index].data,f->msg_data,8);taskEXIT_CRITICAL();
}
#define SET(msg,field,value) do{frame->has_##msg=true;frame->msg.has_##field=true;frame->msg.field=(value);}while(0)
void Extra_Map(fsae_TelemetryFrame *frame,uint32_t now){
 Sample s[SAMPLE_COUNT];uint8_t fresh[SAMPLE_COUNT];
 taskENTER_CRITICAL();memcpy(s,samples,sizeof(s));taskEXIT_CRITICAL();
 for(unsigned i=0;i<SAMPLE_COUNT;i++)fresh[i]=s[i].seen && (uint32_t)(now-s[i].tick)<=(i>=CH_V && i<=CH_O?750U:2000U);
 for(unsigned i=0;i<2;i++){const uint8_t *p=s[i].data;if(!fresh[i])continue;
 if(i==0){SET(pdm_telemetry,bus_voltage_v,(int16_t)be16(p)*0.001f);SET(pdm_telemetry,bus_current_a,(int16_t)be16(p+2)*0.01f);SET(pdm_telemetry,bus_power_w,be16(p+4)*0.1f);SET(pdm_telemetry,bus_energy_wh,be16(p+6)*0.01f);}
 else{SET(pdm_telemetry,battery_voltage_v,(int16_t)be16(p)*0.001f);SET(pdm_telemetry,battery_current_a,(int16_t)be16(p+2)*0.01f);SET(pdm_telemetry,battery_power_w,be16(p+4)*0.1f);SET(pdm_telemetry,battery_energy_wh,be16(p+6)*0.01f);}}
 if(fresh[FAN]){const uint8_t *p=s[FAN].data;SET(fan_telemetry,fan1_rpm,be16(p));SET(fan_telemetry,fan2_rpm,be16(p+2));SET(fan_telemetry,fan3_rpm,be16(p+4));if(p[6]<=100)SET(fan_telemetry,pwm1_duty_pct,p[6]);if(p[7]<=100)SET(fan_telemetry,pwm2_duty_pct,p[7]);}
 int bf=fresh[BAT_FAN1]?BAT_FAN1:fresh[BAT_FANB]?BAT_FANB:-1;
 if(bf>=0){SET(battery_fan,rpm,be16(s[bf].data));SET(battery_fan,actual_duty_pct_x10,bf==BAT_FAN1?be16(s[bf].data+2):s[bf].data[2]*10U);}
 if(fresh[CH_V])SET(charger_telemetry,voltage_v,fl32(s[CH_V].data+3));
 if(fresh[CH_I])SET(charger_telemetry,current_a,fl32(s[CH_I].data+3));
 if(fresh[CH_P])SET(charger_telemetry,protection,le32(s[CH_P].data+3));
 if(fresh[CH_O])SET(charger_telemetry,output_state,s[CH_O].data[3]);
 if(fresh[SOP_MIRROR]){const uint8_t *p=s[SOP_MIRROR].data;SET(sop_limits,discharge_current_limit_a_x10,be16(p));SET(sop_limits,charge_current_limit_a_x10,be16(p+2));SET(sop_limits,discharge_power_limit_kw_x10,be16(p+4));SET(sop_limits,charge_power_limit_kw_x10,be16(p+6));}
 if(fresh[SOP_P]){const uint8_t *p=s[SOP_P].data;SET(sop_limits,protocol_version,3);SET(sop_limits,sequence,p[4]);SET(sop_limits,bms_flags,(p[6]>>4)&3U);frame->sop_limits.has_discharge_power_limit_kw_x10=frame->sop_limits.has_charge_power_limit_kw_x10=false;if(p[6]&16){SET(sop_limits,discharge_power_limit_kw_x10,le16(p));SET(sop_limits,charge_power_limit_kw_x10,le16(p+2));}}
 if(fresh[SOP_I]){SET(sop_limits,discharge_current_limit_a_x10,le16(s[SOP_I].data));SET(sop_limits,charge_current_limit_a_x10,le16(s[SOP_I].data+2));}
 if(fresh[SOP_ACK])SET(sop_limits,ecu_flags,s[SOP_ACK].data[2]);
 if(fresh[BMS_RELAYS]){const uint8_t *p=s[BMS_RELAYS].data;SET(bms_telemetry,pos_relay_state,(p[0]>>6)&3);SET(bms_telemetry,neg_relay_state,(p[0]>>4)&3);SET(bms_telemetry,pre_relay_state,(p[0]>>2)&3);SET(bms_telemetry,charge_state,(p[1]>>4)&1);SET(bms_telemetry,charge_comm_state,(p[1]>>3)&1);}
 if(fresh[BMS_HV]){const uint8_t *p=s[BMS_HV].data;SET(bms_telemetry,last_precharge_result,(p[0]>>2)&3);SET(bms_telemetry,last_precharge_success_ms,be16(p+1));SET(bms_telemetry,last_precharge_failure_ms,be16(p+3));}
 if(fresh[IMD]){const uint8_t *p=s[IMD].data;SET(imd_telemetry,status_code,p[0]>>4);SET(imd_telemetry,frequency_class,p[0]&15);SET(imd_telemetry,flags,p[1]);SET(imd_telemetry,duty_pct_x10,be16(p+2));SET(imd_telemetry,resistance_kohm,be16(p+4));SET(imd_telemetry,frequency_hz_x100,be16(p+6));}
}
