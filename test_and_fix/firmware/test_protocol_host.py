"""Host regression for real CAN decoders and protobuf publishing (no hardware).

Use --cc for a native GCC compiler and --dbc for the supplied Vehicle_CanB.dbc.
Only generated test/build artifacts are written under build/host_protocol.
"""
from pathlib import Path
import argparse
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "build" / "host_protocol"


def vectors_from_dbc(path):
    messages = {}
    current = None
    for line in path.read_text(encoding="utf-8-sig").splitlines():
        match = re.match(r"BO_ (\d+) (\w+): (\d+)", line)
        if match:
            current = {"id": int(match[1]), "dlc": int(match[3]), "signals": {}}
            messages[current["id"]] = current
        match = re.match(r"\s*SG_ (\w+) : (\d+)\|(\d+)@(\d)([+-])\s+\(([^,]+),([^\)]+)\)", line)
        if match and current is not None:
            current["signals"][match[1]] = (int(match[2]), int(match[3]), int(match[4]),
                match[5], float(match[6]), float(match[7]))
    def encode(identifier, values):
        msg = messages[identifier]
        payload = bytearray(msg["dlc"])
        for name, value in values.items():
            start, size, intel = msg["signals"][name][:3]
            raw = value & ((1 << size) - 1)
            position = start
            for i in range(size):
                bit = i if intel else size - i - 1
                if raw & (1 << bit):
                    payload[position // 8] |= 1 << (position % 8)
                position = position + 1 if intel else position + 15 if position % 8 == 0 else position - 1
        return "{" + ",".join(str(x) for x in payload) + "}"
    cases = {
        "CG": (0x69, dict(CGVelocityX=-1234, CGVelocityY=567, CGVelocityValid=1,
            CGSpeedValid=1, CGTrackValid=1, CGHeadingValid=1, CGYawRateValid=1,
            CGYawRateEstimated=1, CGSampleCounter=255, CGSampleAge=30)),
        "INPUT": (0x305, dict(SteeringWheelAngle=-123, APS_OpenPct=456,
            OilPressure_R=1234, OilPressure_F=5678)),
        "GPS_MOTION": (0x68, dict(GPS_GroundTrack=35999, GPS_DualHeading=12345,
            GPS_Altitude=-123, GPS_PositionValid=1, GPS_RMCValid=1,
            GPS_HeadingValid=1, GPS_LapDiagState=6, GPS_HeadingQuality=5,
            GPS_FixQuality=4, GPS_SignalLevel=3)),
        "ODOMETER": (0x7A0, dict(GPS_Odometer=0xABCDEF, GPS_MaxSNR=-12,
            GPS_AvgSNR=-30, GPS_Satellites=18, GPS_GSVTrackedSatellites=22, GPS_LapCount=7)),
        "LAPS": (0x7A1, dict(GPS_CurrentLapTime=1234, GPS_LastLapTime=5678,
            GPS_BestLapTime=3456, GPS_LapDelta=-123)),
        "FAN_DIAG": (0x5A3, dict(Fan_Faults=3, Fan_StatusFlags=9, MaxMotorTemp=-125,
            MaxControllerTemp=32767, Fan_PWM1_Target=45, Fan_PWM2_Target=67)),
        "FAN_PROFILE": (0x5AF, dict(ProfileProtocolVersion=4, RequestedProfile=2,
            EffectiveStrategy1=8, EffectiveStrategy2=9, RuntimeCap1=65, RuntimeCap2=75,
            ProfileLeaseRemaining=30, DerateRequested=1, StallLocked2=1,
            ProfileLeaseActive=1, SourceState=4, LimitReason=8)),
        "FAN_POWER": (0x5A8, dict(PowerSupplyState=3, PowerLimitReason=2,
            ThermalRequest1_Duty=45, ThermalRequest2_Duty=55, PowerLimitedTarget1_Duty=40,
            PowerLimitedTarget2_Duty=50, CurrentBudget=120, PredictedCurrent=321)),
        "FAN_CALIB": (0x5A9, dict(CalibState=2, CalibAbortReason=6, CalibStep=8,
            CalibPWM1Target=40, CalibPWM2Target=60, CalibLeaseRemaining=20,
            CalibParamVersion=4, CalibFlags=0x1234)),
        "BAT_FAN": (0x5AA, dict(BatteryFanRPM=1234, BatteryFanActualDuty=50,
            BatteryFanActiveLimit=60, BatteryFanMode=1, BatteryFanPowerSource=2,
            BatteryFanHardwareReady=1, BatteryFanCalibrationActive=1,
            BatteryFanLeaseRemaining=30, BatteryFanProtocolVersion=4)),
    }
    for identifier, name, signed, factor in [
        (0x305, "OilPressure_R", "+", 0.001), (0x305, "OilPressure_F", "+", 0.001),
        (0x69, "CGVelocityX", "-", 0.01), (0x69, "CGVelocityY", "-", 0.01),
        (0x301, "GroundSpeed", "+", 0.1), (0x7A1, "GPS_LapDelta", "-", 0.01),
    ]:
        signal = messages[identifier]["signals"][name]
        assert signal[3:] == (signed, factor, 0.0), (identifier, name, signal)
    # Detect stale/wrong thermal mailbox constants independently of the C code.
    assert "MLX90640_TireTemp_01_04_0x071" in path.read_text(encoding="utf-8-sig")
    return "\n".join("#define V_" + name + " " + encode(*case) for name, case in cases.items())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    parser.add_argument("--dbc", type=Path, default=Path.home() / "Downloads" / "Vehicle_CanB.dbc")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    sources = ["can_decode.c", "telemetry_data.c", "canc_telemetry.c", "telemetry_extra.c",
               "publish.c", "fsae_telemetry.pb.c", "pb_common.c", "pb_encode.c", "pb_decode.c"]
    headers = ["can_decode.h", "telemetry_data.h", "canc_telemetry.h", "telemetry_extra.h",
                "publish.h", "telemetry_updates.h", "fsae_telemetry.pb.h", "pb.h", "pb_common.h", "pb_encode.h", "pb_decode.h"]
    for name in sources:
        shutil.copyfile(ROOT / "Core/Src" / name, OUT / name)
    for name in headers:
        shutil.copyfile(ROOT / "Core/Inc" / name, OUT / name)
    can = (ROOT / "Core/Inc/can.h").read_text(encoding="utf-8-sig")
    defines = "\n".join(re.findall(r"^#define CAN_(?:ID_|CHANNEL_|BATTERY_)[^\n]+", can, re.M))
    (OUT / "can.h").write_text("#pragma once\n#include <stdint.h>\n#include <stddef.h>\n#include <string.h>\n" + defines + """
 typedef struct {uint8_t can_channel;uint32_t msg_id;uint8_t msg_data[8];uint8_t dlc,is_ext_id;} CAN_Msg_Queue_t;
 uint32_t HAL_GetTick(void);
 """)
    (OUT / "main.h").write_text('#include "can.h"\n')
    (OUT / "FreeRTOS.h").write_text('#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n')
    (OUT / "task.h").write_text('void TestSuspend(void);\nint TestResume(void);\n#define vTaskSuspendAll TestSuspend\n#define xTaskResumeAll TestResume\n')
    (OUT / "freertos_app.h").write_text("")
    (OUT / "app_log.h").write_text('#define App_LogPrintf(...) ((void)0)\n')
    (OUT / "app_config.h").write_text('#define APP_UPLINK_ENABLED 1U\n')
    (OUT / "cmsis_os2.h").write_text("#pragma once\n#include <stdint.h>\ntypedef void *osMessageQueueId_t;\ntypedef int osStatus_t;\n#define osErrorResource (-3)\n#define osOK 0\nint osMessageQueuePut(void *, const void *, unsigned, unsigned);\n")
    (OUT / "vectors.h").write_text(vectors_from_dbc(args.dbc))
    # Validate actual hardware filter allowlist, including spare bank capacity.
    body = (ROOT / "Core/Src/can.c").read_text(encoding="utf-8").split("g_can2_vehicle_filter_ids[] = {", 1)[1].split("};", 1)[0]
    thermal_source = (ROOT / "Core/Src/freertos.c").read_text(encoding="utf-8")
    thermal_id_function = thermal_source[thermal_source.index("static uint32_t Mlx_GetCanSummaryId(uint8_t sensor_id)\n{"):]
    thermal_id_function = thermal_id_function[:thermal_id_function.index("\nstatic void Mlx_EncodeCanSummaryPair")]
    (OUT / "filters.h").write_text("static const uint16_t filter_ids[]={" + body + "};\n" + thermal_id_function)
    dispatch = thermal_source[thermal_source.index("static void CanVehicle_Dispatch(const CAN_Msg_Queue_t *recv_data)\n{"):]
    dispatch = dispatch[:dispatch.index("static void Rs485_Dispatch")]
    (OUT / "dispatch.h").write_text("#define APP_SPICANA_DECODE_ENABLED 0U\n" + dispatch)
    (OUT / "test.c").write_text(TEST_C)
    shutil.copyfile(ROOT / "test_and_fix/firmware/publish_presence_test.c", OUT / "publish_presence_test.c")
    exe = OUT / "test.exe"
    subprocess.run([args.cc, "-std=c99", "-O0", "-Werror=return-type", "-static", "-I" + str(OUT), str(OUT / "test.c"),
        str(OUT / "publish_presence_test.c"), *[str(OUT / x) for x in sources], "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    (OUT / "app_config.h").write_text('#define APP_UPLINK_ENABLED 0U\n')
    subprocess.run([args.cc, "-std=c99", "-O0", "-Werror=return-type", "-static", "-I" + str(OUT), str(OUT / "test.c"),
        str(OUT / "publish_presence_test.c"), *[str(OUT / x) for x in sources], "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


TEST_C = r'''#include "can_decode.h"
#include "telemetry_data.h"
#include "canc_telemetry.h"
#include "telemetry_extra.h"
#include "publish.h"
#include "pb_decode.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "vectors.h"
#include "filters.h"
#include "app_config.h"
static uint32_t tick;
uint32_t HAL_GetTick(void){return tick;}
static unsigned scheduler_depth, mutate_on_resume;
void TestSuspend(void){scheduler_depth++;}
int TestResume(void){assert(scheduler_depth);if(--scheduler_depth==0 && mutate_on_resume){
 mutate_on_resume=0;g_CANB_LoopData.ECU.SteeringAngleDeciDeg=999;
 Publish_QueueUpdates(TELEMETRY_UPDATE(TELEMETRY_DRIVER_INPUTS));
}return 0;}
int osMessageQueuePut(void *q,const void *p,unsigned a,unsigned b){(void)q;(void)p;(void)a;(void)b;return 0;}
osMessageQueueId_t PublishQueueItemHandle;
static fsae_TelemetryFrame out;
static uint8_t payload[PUBLISH_MAX_PAYLOAD_SIZE];
int Publish_PresenceRegressionTest(void);
static TelemetryUpdates queued;
static unsigned queue_calls;
static osStatus_t Test_QueueUpdates(TelemetryUpdates updates){queued|=updates;if(updates)queue_calls++;return 0;}
#define Publish_QueueUpdates Test_QueueUpdates
#include "dispatch.h"
#undef Publish_QueueUpdates
static void decoder_returns(void){
 CAN_Msg_Queue_t f={.can_channel=CAN_CHANNEL_VEHICLE_CANB,.msg_id=CAN_ID_CANB_DRIVER_INPUTS_OIL_PRESSURE,.dlc=8,.msg_data={1}};
 assert(!CAN_DecodeVehicleCanbMessage(NULL));assert(!CAN_DecodeBatteryBoxMessage(NULL));
 assert(!CAN_DecodeControllerCanaMessage(NULL));assert(!CAN_DecodeCdcMonitorMessage(NULL));
 assert(!Extra_Receive(NULL));assert(!Canc_Receive(NULL));
 assert(CAN_DecodeVehicleCanbMessage(&f));assert(g_CANB_LoopData.ECU.SteeringAngleDeciDeg==1);
 memset(f.msg_data,0,8);assert(CAN_DecodeVehicleCanbMessage(&f));assert(CAN_DecodeVehicleCanbMessage(&f));
 assert(g_CANB_LoopData.ECU.SteeringAngleDeciDeg==0);
 f.dlc=7;assert(!CAN_DecodeVehicleCanbMessage(&f));f.dlc=9;assert(!CAN_DecodeVehicleCanbMessage(&f));
 f.dlc=8;f.msg_id=0x777;assert(!CAN_DecodeVehicleCanbMessage(&f));
 f.msg_id=CAN_ID_CANB_DRIVING_MODE_CMD;f.msg_data[0]=46;assert(!CAN_DecodeVehicleCanbMessage(&f));
 f.msg_id=CAN_ID_CANB_BRAKE_PEDAL_ENCODER;f.dlc=7;memcpy(f.msg_data,(uint8_t[]){7,1,1,0,16,0,0},7);
 assert(!CAN_DecodeVehicleCanbMessage(&f));f.msg_data[4]=0;assert(CAN_DecodeVehicleCanbMessage(&f));
 f.msg_id=CAN_ID_CANB_GPS_SPEED;f.dlc=2;f.msg_data[0]=255;f.msg_data[1]=255;
  assert(CAN_DecodeVehicleCanbMessage(&f)==TELEMETRY_UPDATE(TELEMETRY_GPS_SPEED));
  // Invalid measurement status is accepted when the frame updates validity caches.
 f.msg_id=CAN_ID_CANB_GPS_CG_VELOCITY;f.dlc=8;memset(f.msg_data,0,8);
 assert(CAN_DecodeVehicleCanbMessage(&f));assert(!g_CANB_LoopData.GPS.CgVelocityValid);
 f.msg_id=CAN_ID_CANB_AMK_STATUS_MODE_LOGIC;f.dlc=5;f.msg_data[0]=0xF1;
 assert(CAN_DecodeVehicleCanbMessage(&f));assert(g_CANB_LoopData.ECU.ERRO[3]);
 f.msg_id=CAN_ID_COMPETITION_CURRENT;f.dlc=6;f.msg_data[0]=0;f.msg_data[1]=0x20;
 assert(CAN_DecodeVehicleCanbMessage(&f));assert(!(g_CANB_LoopData.EnergyMeter.ValidFlags&1));
 f.msg_data[0]=1;assert(!CAN_DecodeVehicleCanbMessage(&f));
 // Extra-only frames, including accepted zero and repeated payloads.
 f.msg_id=0x5A0;f.dlc=8;memset(f.msg_data,0,8);f.msg_data[0]=1;
 assert(CAN_DecodeVehicleCanbMessage(&f));memset(f.msg_data,0,8);
 assert(CAN_DecodeVehicleCanbMessage(&f));assert(CAN_DecodeVehicleCanbMessage(&f));
 f.dlc=7;assert(!CAN_DecodeVehicleCanbMessage(&f));f.dlc=8;f.msg_id=0x5AA;f.msg_data[2]=101;
 assert(!CAN_DecodeVehicleCanbMessage(&f));f.msg_id=0x5AF;f.msg_data[0]=3;assert(!CAN_DecodeVehicleCanbMessage(&f));
 // Battery standard IVT, extended, CANB mirrors, and Extra-only battery fan.
 f=(CAN_Msg_Queue_t){.can_channel=CAN_CHANNEL_BATTERY_BOX,.msg_id=CAN_ID_BATTERY_IVT_CURRENT,.dlc=6,.msg_data={0,0,1}};
 assert(CAN_DecodeBatteryBoxMessage(&f));assert(g_BatteryInfo.IVT.CurrentMa==1);
 f.msg_data[2]=0;assert(CAN_DecodeBatteryBoxMessage(&f));assert(CAN_DecodeBatteryBoxMessage(&f));
 f.msg_data[1]=0x20;assert(CAN_DecodeBatteryBoxMessage(&f));assert(!(g_BatteryInfo.IVT.ValidFlags&1));
 f.msg_data[0]=1;assert(!CAN_DecodeBatteryBoxMessage(&f));f.msg_data[0]=0;f.dlc=5;assert(!CAN_DecodeBatteryBoxMessage(&f));
 f.is_ext_id=1;f.msg_id=CAN_ID_BATTERY_PACK_SUMMARY;f.dlc=8;memset(f.msg_data,0,8);
 assert(CAN_DecodeBatteryBoxMessage(&f));assert(CAN_DecodeBatteryBoxMessage(&f));
 f.dlc=6;assert(!CAN_DecodeBatteryBoxMessage(&f));f.dlc=8;f.msg_id=0x186E50F4;
 assert(CAN_DecodeBatteryBoxMessage(&f));f.msg_data[2]=4;assert(!CAN_DecodeBatteryBoxMessage(&f));
 f.msg_data[2]=0;f.msg_id=0x1FFFFFFF;assert(!CAN_DecodeBatteryBoxMessage(&f));
 f.msg_id=CAN_ID_BATTERY_CELL_VOLTAGE_BASE+1;assert(!CAN_DecodeBatteryBoxMessage(&f));
 f.msg_id=CAN_ID_BATTERY_CELL_VOLTAGE_BASE;assert(CAN_DecodeBatteryBoxMessage(&f));
 f.msg_id=CAN_ID_BATTERY_CELL_SUM;f.dlc=2;f.msg_data[0]=f.msg_data[1]=255;
 assert(CAN_DecodeBatteryBoxMessage(&f));assert(!g_BatteryInfo.CellSumValid);
 f=(CAN_Msg_Queue_t){.can_channel=CAN_CHANNEL_VEHICLE_CANB,.msg_id=CAN_ID_CANB_BATTERY_POWER_STATUS,.dlc=8};
 assert(CAN_DecodeBatteryBoxMessage(&f));f.dlc=6;assert(!CAN_DecodeBatteryBoxMessage(&f));
 // CDC unknown/DLC errors must not report success, even though diagnostics update.
 f=(CAN_Msg_Queue_t){.can_channel=CAN_CHANNEL_CDC_MONITOR,.msg_id=0x302,.dlc=6};
 assert(CAN_DecodeCdcMonitorMessage(&f));assert(CAN_DecodeCdcMonitorMessage(&f));
 f.dlc=5;assert(!CAN_DecodeCdcMonitorMessage(&f));f.dlc=9;assert(!CAN_DecodeCdcMonitorMessage(&f));
 f.dlc=8;f.msg_id=0x777;assert(!CAN_DecodeCdcMonitorMessage(&f));
 f=(CAN_Msg_Queue_t){.can_channel=CAN_CHANNEL_CONTROLLER_CANA,.msg_id=CAN_ID_CANA_FR_STATUS_SPEED_TORQUE_CURRENT,.dlc=8};
 assert(CAN_DecodeControllerCanaMessage(&f));assert(CAN_DecodeControllerCanaMessage(&f));
 f.dlc=9;assert(!CAN_DecodeControllerCanaMessage(&f));f.dlc=8;f.msg_id=0x777;
 assert(!CAN_DecodeControllerCanaMessage(&f));
 // Execute the real native dispatch functions with a counting publish stub.
 f=(CAN_Msg_Queue_t){.can_channel=CAN_CHANNEL_VEHICLE_CANB,.msg_id=0x5A0,.dlc=8};
  queued=0;queue_calls=0;CanVehicle_Dispatch(&f);CanVehicle_Dispatch(&f);
  assert(queued==TELEMETRY_EXTRA_UPDATE(0) && queue_calls==2);
 f.dlc=7;CanVehicle_Dispatch(&f);f.dlc=8;f.msg_id=0x777;CanVehicle_Dispatch(&f);
  assert(queued==TELEMETRY_EXTRA_UPDATE(0) && queue_calls==2);
 f=(CAN_Msg_Queue_t){.can_channel=CAN_CHANNEL_BATTERY_BOX,.is_ext_id=1,.msg_id=0x186E50F4,.dlc=8};
  queued=0;queue_calls=0;CanBattery_Dispatch(&f);CanBattery_Dispatch(&f);
 f.msg_data[2]=4;CanBattery_Dispatch(&f);f.msg_id=0x1FFFFFFF;CanBattery_Dispatch(&f);CanBattery_Dispatch(NULL);
  assert(queued==TELEMETRY_EXTRA_UPDATE(3) && queue_calls==2);
 puts("PASS: decoder success nonzero/zero/repeated, Extra-only, unknown/DLC/range rejection, validity/error updates, native dispatch publish gating");
}
static void send(unsigned id,unsigned dlc,const uint8_t *data){
 CAN_Msg_Queue_t f={0};f.can_channel=2;f.msg_id=id;f.dlc=dlc;memcpy(f.msg_data,data,dlc);
 CAN_DecodeVehicleCanbMessage(&f);
}
static void roundtrip(PublishTopic_t topic){
 uint16_t size;assert(Publish_BuildFrame(topic,42,payload,sizeof(payload),&size));
 out=(fsae_TelemetryFrame)fsae_TelemetryFrame_init_zero;
 pb_istream_t s=pb_istream_from_buffer(payload,size);assert(pb_decode(&s,fsae_TelemetryFrame_fields,&out));
 assert(out.frame_id==42 && out.header.seq==42);
}
static int has_filter(unsigned id){for(unsigned i=0;i<sizeof(filter_ids)/sizeof(filter_ids[0]);i++)if(filter_ids[i]==id)return 1;return 0;}
static void updates_roundtrip(TelemetryUpdates groups,unsigned id){
 uint16_t size=0;assert(Publish_BuildUpdates(groups,id,payload,sizeof(payload),&size));
 assert(scheduler_depth==0);out=(fsae_TelemetryFrame)fsae_TelemetryFrame_init_zero;
 pb_istream_t s=pb_istream_from_buffer(payload,size);assert(pb_decode(&s,fsae_TelemetryFrame_fields,&out));
 assert(out.has_header && out.header.source_id==1 && out.header.seq==id && out.frame_id==id);
 assert(out.header.timestamp_ms==out.timestamp_ms);
}
static void update_groups(void){
 CANB_Loop_Data_Clear();Battery_Data_Clear();tick=20;
 CAN_Msg_Queue_t f={.can_channel=CAN_CHANNEL_VEHICLE_CANB,.msg_id=CAN_ID_CANB_DRIVER_INPUTS_OIL_PRESSURE,.dlc=8};
 TelemetryUpdates driver=TELEMETRY_UPDATE(TELEMETRY_DRIVER_INPUTS),energy=TELEMETRY_UPDATE(TELEMETRY_ENERGY);
 uint16_t size=99;
 assert(CAN_DecodeVehicleCanbMessage(&f)==driver);
 assert(CAN_DecodeVehicleCanbMessage(&f)==driver); // valid repeated zero
 f.dlc=7;assert(!CAN_DecodeVehicleCanbMessage(&f));f.dlc=8;f.msg_id=0x777;assert(!CAN_DecodeVehicleCanbMessage(&f));
 const unsigned ids[]={CAN_ID_CANB_WHEEL_ACTUAL_TORQUE,CAN_ID_CANB_MOTOR_DIAGNOSTIC_CODES,CAN_ID_CANB_WHEEL_ACTUAL_SPEED,
 CAN_ID_CANB_MOTOR_TEMPERATURE,CAN_ID_CANB_INVERTER_TEMPERATURE,CAN_ID_CANB_IGBT_TEMPERATURE};
 for(unsigned i=0;i<6;i++){f.msg_id=ids[i];assert(CAN_DecodeVehicleCanbMessage(&f)==TELEMETRY_UPDATE(TELEMETRY_MOTOR_TORQUE+i));}
 f.msg_id=0x5A0;assert(CAN_DecodeVehicleCanbMessage(&f)==TELEMETRY_EXTRA_UPDATE(0));
 f.msg_id=0x5A1;assert(CAN_DecodeVehicleCanbMessage(&f)==TELEMETRY_EXTRA_UPDATE(1));
 f.msg_id=0x5A2;assert(CAN_DecodeVehicleCanbMessage(&f)==TELEMETRY_EXTRA_UPDATE(2));
 g_BatteryInfo.TotalVolt=777000;g_CANB_LoopData.ECU.Motor_RPM[0]=1234;
 g_CANB_LoopData.ECU.SteeringAngleDeciDeg=-120;
 updates_roundtrip(driver,256);
 assert(out.steering_angle==-12 && out.hv_voltage==0 && out.motor_rpm==0);
 assert(out.has_vehicle_state && out.vehicle_state.motors_count==0);
 assert(!out.has_energy_meter && !out.has_fan_telemetry && !out.has_canc && !out.has_ivt_telemetry && !out.has_fast_telemetry);
 updates_roundtrip(TELEMETRY_EXTRA_UPDATE(0),257);
 assert(out.has_pdm_telemetry && out.pdm_telemetry.has_bus_voltage_v && out.pdm_telemetry.bus_voltage_v==0);
 assert(!out.pdm_telemetry.has_battery_voltage_v && !out.has_fan_telemetry && !out.has_vehicle_state && out.hv_voltage==0);
 updates_roundtrip(TELEMETRY_UPDATE(TELEMETRY_MOTOR_SPEED),UINT32_MAX);
 assert(out.vehicle_state.motors_count==4 && out.vehicle_state.motors[0].rpm==1234);
 for(unsigned i=0;i<4;i++)assert(out.vehicle_state.motors[i].position!=0 && !out.vehicle_state.motors[i].has_logic_state);
 updates_roundtrip(driver|TELEMETRY_UPDATE(TELEMETRY_MOTOR_SPEED)|TELEMETRY_UPDATE(TELEMETRY_MOTOR_TORQUE),255);
 assert(out.steering_angle==-12 && out.vehicle_state.motors[0].rpm==1234 && !out.has_fan_telemetry);
 // Pack -> IVT -> pack fallback with original physical units and freshness.
 g_BatteryInfo.PackReceived=1;g_BatteryInfo.PackValidFlags=7;
 g_BatteryInfo.PackLastUpdateMs=tick;g_BatteryInfo.TotalVolt=500000;g_BatteryInfo.TotalCurrent=-12300;
 g_BatteryInfo.SocDeciPct=0;
 updates_roundtrip(TELEMETRY_UPDATE(TELEMETRY_PACK),5);
 assert(out.hv_voltage==500 && fabs(out.hv_current+12.3f)<0.001f && out.has_battery_soc_pct_x10 && out.battery_soc_pct_x10==0);
 g_BatteryInfo.IVT.ValidFlags=3;g_BatteryInfo.IVT.ReceivedFlags=3;
 g_BatteryInfo.IVT.LastUpdateMs[0]=g_BatteryInfo.IVT.LastUpdateMs[1]=tick;
 g_BatteryInfo.IVT.VoltageU1Mv=499000;g_BatteryInfo.IVT.CurrentMa=0;
 updates_roundtrip(TELEMETRY_UPDATE(TELEMETRY_IVT),6);
 assert(out.hv_voltage==499 && out.ivt_telemetry.has_current_ma && out.ivt_telemetry.current_ma==0);
 g_BatteryInfo.IVT.ValidFlags=0;g_BatteryInfo.IVT.CurrentState=2;
 updates_roundtrip(TELEMETRY_UPDATE(TELEMETRY_IVT),7);
 assert(out.hv_voltage==500 && out.has_ivt_telemetry && !out.ivt_telemetry.has_current_ma && out.ivt_telemetry.current_state==2);
 assert((CAN_DecodeBatteryService(tick+2001)&TELEMETRY_UPDATE(TELEMETRY_PACK))!=0);
 f=(CAN_Msg_Queue_t){.can_channel=CAN_CHANNEL_CDC_MONITOR,.msg_id=0x301,.dlc=8};
 assert(CAN_DecodeCdcMonitorMessage(&f)==TELEMETRY_CANC_UPDATE(0));
 f.msg_id=0x302;f.dlc=6;assert(CAN_DecodeCdcMonitorMessage(&f)==TELEMETRY_CANC_UPDATE(1));
 updates_roundtrip(TELEMETRY_CANC_UPDATE(0),1);
 assert(out.has_canc && out.canc.has_cdc_info && !out.canc.has_cdc_acceleration && !out.has_pdm_telemetry);
 (void)Publish_TakePending();
#if APP_UPLINK_ENABLED
 assert(Publish_QueueUpdates(TELEMETRY_UPDATE(TELEMETRY_CG_VELOCITY)|TELEMETRY_EXTRA_UPDATE(19)|TELEMETRY_EXTRA_UPDATE(21))==0);
 assert(!Publish_TakePending()); // accepted cache-only groups produce no fake telemetry
 assert(Publish_QueueUpdates(driver)==0 && Publish_QueueUpdates(driver|energy)==0);
 TelemetryUpdates in_flight=Publish_TakePending();assert(in_flight==(driver|energy));assert(!Publish_TakePending());
 // A producer runs after snapshot release, before encoding. The encoded value
 // stays coherent, while its new pending bit survives successful transmission.
 g_CANB_LoopData.ECU.SteeringAngleDeciDeg=120;mutate_on_resume=1;
 updates_roundtrip(driver,2);assert(out.steering_angle==12 && g_CANB_LoopData.ECU.SteeringAngleDeciDeg==999);
 assert(Publish_TakePending()==driver);
 assert(!Publish_BuildUpdates(in_flight,3,payload,1,&size) && size==0);
 assert(Publish_QueueUpdates(TELEMETRY_UPDATE(TELEMETRY_THERMAL))==0);
 assert(Publish_QueueUpdates(in_flight)==0); // build/send failure requeue
 assert(Publish_TakePending()==(in_flight|TELEMETRY_UPDATE(TELEMETRY_THERMAL)));
 assert(!Publish_TakePending());
#else
 assert(Publish_QueueUpdates(driver|energy)==osErrorResource);
 assert(!Publish_TakePending());
#endif
 assert(!Publish_BuildUpdates(0,4,payload,sizeof(payload),&size));
 puts("PASS: exact update groups, zero/repeat, selective cache mapping, nested positions, coherent snapshot, concurrent pending merge/drain/retry, frame headers");
}
int main(void){
 assert(Publish_PresenceRegressionTest()==0);
 CANB_Loop_Data_Clear();Battery_Data_Clear();
 const uint8_t cg[8]=V_CG, input[8]=V_INPUT, motion[8]=V_GPS_MOTION;
 const uint8_t odometer[8]=V_ODOMETER,laps[8]=V_LAPS,diag[8]=V_FAN_DIAG;
 const uint8_t profile[8]=V_FAN_PROFILE,power[8]=V_FAN_POWER,calib[8]=V_FAN_CALIB,batfan[8]=V_BAT_FAN;
 const unsigned expected[]={0x69,0x67,0x68,0x7A0,0x7A1,0x703,0x310,0x5A3,0x5A5,0x5A6,0x5A7,0x5A8,0x5A9,0x5AF};
 assert((sizeof(filter_ids)/sizeof(filter_ids[0])+3)/4<=14);
 for(unsigned i=0;i<sizeof(expected)/sizeof(expected[0]);i++)assert(has_filter(expected[i]));
 assert(CAN_ID_CANB_THERMAL_SUMMARY_FR==0x71 && CAN_ID_CANB_THERMAL_SUMMARY_FL==0x72);
 assert(Mlx_GetCanSummaryId(0)==0x71 && Mlx_GetCanSummaryId(1)==0x72 && Mlx_GetCanSummaryId(2)==0x73 && Mlx_GetCanSummaryId(3)==0x74);
 assert(!has_filter(0x501) && !has_filter(0x504));
 send(0x305,8,input);
 assert(g_CANB_LoopData.ECU.SteeringAngleDeciDeg==-123);
 assert(g_CANB_LoopData.ECU.ThrottlePositionDeciPct==456);
 assert(g_CANB_LoopData.ECU.OilPressureRearKpa==1234 && g_CANB_LoopData.ECU.OilPressureFrontKpa==5678);
 roundtrip(PUBLISH_TOPIC_VEHICLE_STATE);
 assert(fabs(out.brake_pressure-1.234f)<0.0001f && fabs(out.steering_angle+12.3f)<0.0001f);
 send(0x305,7,input);assert(g_CANB_LoopData.ECU.OilPressureFrontKpa==5678);
 send(0x68,8,motion);
 assert(g_CANB_LoopData.GPS.GroundTrackCentiDeg==35999 && g_CANB_LoopData.GPS.DualHeadingCentiDeg==12345);
 assert(g_CANB_LoopData.GPS.AltitudeDeciM==-123 && g_CANB_LoopData.GPS.LapDiagState==6);
 assert(g_CANB_LoopData.GPS.HeadingQuality==5 && g_CANB_LoopData.GPS.FixQuality==4 && g_CANB_LoopData.GPS.SignalLevel==3);
 send(0x7A0,8,odometer);send(0x7A1,8,laps);
 assert(g_CANB_LoopData.GPS.OdometerDeciKm==0xABCDEF && g_CANB_LoopData.GPS.MaxSnrDbHz==-12 && g_CANB_LoopData.GPS.AvgSnrDbHz==-30);
 assert(g_CANB_LoopData.GPS.LapCount==7 && g_CANB_LoopData.GPS.LapDeltaCentiS==-123 && g_CANB_LoopData.GPS.LastLapCentiS==5678);
 // Tick zero, signed body velocity, independent legacy speed, local timeout.
 send(0x69,8,cg);assert(g_CANB_LoopData.GPS.CgSeen && g_CANB_LoopData.GPS.CgVelocityValid);
 assert(g_CANB_LoopData.GPS.CgVelocityXCentiMps==-1234 && g_CANB_LoopData.GPS.CgVelocityYCentiMps==567);
 assert(g_CANB_LoopData.GPS.CgSampleCounter==255 && g_CANB_LoopData.GPS.CgSampleAgeMs==30);
 CAN_DecodeVehicleService(150);assert(g_CANB_LoopData.GPS.CgVelocityValid);
 CAN_DecodeVehicleService(151);assert(!g_CANB_LoopData.GPS.CgVelocityValid);
 uint8_t bad[8];memcpy(bad,cg,8);bad[4]|=64;send(0x69,8,bad);assert(!g_CANB_LoopData.GPS.CgVelocityValid);
 memcpy(bad,cg,8);bad[0]=0;bad[1]=128;send(0x69,8,bad);assert(!g_CANB_LoopData.GPS.CgVelocityValid);
 memcpy(bad,cg,8);bad[6]=bad[7]=255;send(0x69,8,bad);assert(!g_CANB_LoopData.GPS.CgVelocityValid);
 for(unsigned bit=0;bit<5;bit++){memcpy(bad,cg,8);bad[4]&=~(1U<<bit);send(0x69,8,bad);assert(!g_CANB_LoopData.GPS.CgVelocityValid);}
 tick=UINT32_MAX-50;send(0x69,8,cg);CAN_DecodeVehicleService(49);assert(g_CANB_LoopData.GPS.CgVelocityValid);
 CAN_DecodeVehicleService(100);assert(!g_CANB_LoopData.GPS.CgVelocityValid);
 tick=0;send(0x301,2,(const uint8_t[]){0xD2,4});assert(g_CANB_LoopData.IMU.GpsSpeedKmh==123);
 CAN_DecodeVehicleService(501);assert(!(g_CANB_LoopData.IMU.ValidFlags&CANB_MOTION_VALID_GPS));
 // Exact DLC and source ownership; rejected frames must not refresh valid data.
 CANB_Loop_Data_Clear();
 for(unsigned d=0;d<=8;d++)if(d!=8){send(0x69,d,cg);assert(!g_CANB_LoopData.GPS.CgSeen);}
 CAN_Msg_Queue_t f={.can_channel=1,.msg_id=0x69,.dlc=8};memcpy(f.msg_data,cg,8);
 CAN_DecodeVehicleCanbMessage(&f);assert(!g_CANB_LoopData.GPS.CgSeen);
 f.can_channel=2;f.is_ext_id=1;CAN_DecodeVehicleCanbMessage(&f);assert(!g_CANB_LoopData.GPS.CgSeen);
 uint8_t status[8]={0x21,0xA5,0xC3,0x12,0x34};send(0x509,5,status);
 assert(g_CANB_LoopData.ECU.driving_mode==3 && g_CANB_LoopData.ECU.AmkSystemReadyMask==5 && g_CANB_LoopData.ECU.AmkEnableMask==12);
 status[0]=0xF1;send(0x509,5,status);assert(g_CANB_LoopData.ECU.driving_mode==3);
 send(0x310,8,(const uint8_t[]){51,0,0,0,0,0,0,0});assert(g_CANB_LoopData.ECU.RequestedDriveModeCode==51 && g_CANB_LoopData.ECU.driving_mode==3);
 send(0x703,2,(const uint8_t[]){7,5});assert(g_CANB_LoopData.ECU.SlipLevel==7 && g_CANB_LoopData.ECU.DisableRequest==1 && g_CANB_LoopData.ECU.LapTimerToggle==1);
 send(0x503,8,(const uint8_t[]){1,0,2,0,3,0,255,255});assert(g_CANB_LoopData.ECU.DiagnosticNumber[0]==1 && g_CANB_LoopData.ECU.DiagnosticNumber[3]==65535);
 send(0x001,7,(const uint8_t[]){7,1,1,255,15,0,0});assert(g_CANB_LoopData.ECU.PedalCount==4095);
 send(0x001,7,(const uint8_t[]){7,1,1,0,16,0,0});assert(g_CANB_LoopData.ECU.PedalCount==4095);
 send(0x726,3,(const uint8_t[]){0x21,0x43,120});assert(!g_CANB_LoopData.ECU.RegenSeen);
 send(0x726,8,(const uint8_t[]){0x21,0x43,120,0,0,0,0,0});assert(g_CANB_LoopData.ECU.RegenRate==120 && g_CANB_LoopData.ECU.RegenReasons[3]==4);
 // Fan BE temperature sentinel, CH1/CH2 separation, V4 and independent ages.
 send(0x5A3,8,diag);roundtrip(PUBLISH_TOPIC_VEHICLE_STATE);
 assert(out.fan_telemetry.max_motor_temp_dc==-125 && !out.fan_telemetry.has_max_controller_temp_dc);
 assert(out.fan_telemetry.pwm1_target_pct==45 && out.fan_telemetry.pwm2_target_pct==67);
 send(0x5A5,8,(const uint8_t[]){1,9,7,0x21,40,50,60,70});
 send(0x5A6,8,(const uint8_t[]){20,30,60,25,10,100,40,1});
 send(0x5A6,8,(const uint8_t[]){21,31,61,26,11,101,41,2});
 send(0x5A7,8,(const uint8_t[]){2,40,50,10,8,1,20,4});roundtrip(PUBLISH_TOPIC_VEHICLE_STATE);
 assert(out.fan_telemetry.ack_result==7 && out.fan_telemetry.ack_failsafe==2);
 assert(out.fan_telemetry.curve_temp_off==20 && g_CANB_LoopData.Fan.Curve[1].TempOff==21);
 assert(out.fan_telemetry.failsafe_strategy==2 && out.fan_telemetry.control_mode==1);
 send(0x5A8,8,power);send(0x5A9,8,calib);send(0x5AF,8,profile);
 assert(g_CANB_LoopData.Fan.Power.PredictedCurrentDeciA==321 && g_CANB_LoopData.Fan.Power.LimitReason==2);
 assert(g_CANB_LoopData.Fan.Calibration.Flags==0x1234 && g_CANB_LoopData.Fan.Calibration.AbortReason==6);
 assert(g_CANB_LoopData.Fan.Profile.EffectiveStrategy[1]==9 && g_CANB_LoopData.Fan.Profile.LimitReason==8);
 memcpy(bad,profile,8);bad[0]=3;tick=10;send(0x5AF,8,bad);assert(g_CANB_LoopData.Fan.Profile.Tick==0);
 CAN_DecodeVehicleService(1501);assert(!g_CANB_LoopData.Fan.Power.Valid && !g_CANB_LoopData.Fan.Profile.Valid);
 memset(&out,0,sizeof(out));Extra_Map(&out,1501);assert(!out.fan_telemetry.has_max_motor_temp_dc);
 tick=0;send(0x5AA,8,batfan);assert(g_BatteryInfo.CanbFan.Rpm==1234 && g_BatteryInfo.CanbFan.PowerSource==2 && g_BatteryInfo.CanbFan.StatusFlags==129);
 memset(&out,0,sizeof(out));Extra_Map(&out,0);assert(out.battery_fan.actual_duty_pct_x10==500);
 f=(CAN_Msg_Queue_t){.can_channel=1,.is_ext_id=1,.msg_id=0x186E50F4,.dlc=8,.msg_data={0,77,0,123}};
 Extra_Receive(&f);memset(&out,0,sizeof(out));Extra_Map(&out,0);assert(out.battery_fan.rpm==77 && out.battery_fan.actual_duty_pct_x10==123);
 // Real thermal protobuf positions agree with physical FR, FL, RL, RR sensors.
 g_MLX90640_Frame.ValidMask=15;
 for(unsigned i=0;i<4;i++)for(unsigned j=0;j<4;j++)g_MLX90640_Frame.RegionTemp[i][j]=1000*(i+1);
 roundtrip(PUBLISH_TOPIC_THERMAL_SUMMARY);
 assert(out.thermal_summary.sensors[2].position==fsae_MotorPosition_MOTOR_POSITION_FRONT_LEFT && out.thermal_summary.sensors[2].avg_temp_centi_c==2000);
 assert(out.thermal_summary.sensors[3].position==fsae_MotorPosition_MOTOR_POSITION_FRONT_RIGHT && out.thermal_summary.sensors[3].avg_temp_centi_c==1000);
 // Preserve the original CANC regression.
 tick=0;f=(CAN_Msg_Queue_t){.can_channel=4,.msg_id=0x301,.dlc=8,.msg_data={0x12,0x31,0x45,0x60,0x78,0x90,0xAB,0xC0}};
 Canc_Receive(&f);memset(&out,0,sizeof(out));Canc_Map(&out,100);assert(out.canc.cdc_info.CDC_HeiFL==0x123 && out.canc.cdc_info.status.fresh);
 memset(&out,0,sizeof(out));Canc_Map(&out,101);assert(!out.canc.cdc_info.has_CDC_HeiFL);
 f.dlc=7;Canc_Receive(&f);memset(&out,0,sizeof(out));Canc_Map(&out,101);assert(out.canc.cdc_info.status.rejected_count==1);
  puts("PASS: DBC vectors, CANB filters/decoders, source/DLC rejection, tick wrap, fan V4, thermal positions, protobuf round trips, CANC");
   decoder_returns();
  update_groups();
 return 0;
}
'''

if __name__ == "__main__":
    main()
