/* Isolated firmware test: run before starting acquisition/RTOS tasks, after
 * HAL tick initialization. Uses real Publish_BuildFrame and nanopb round trips.
 * Returns 0 on success or the failing source line. Not part of normal firmware.
 * Replaces telemetry caches; never invoke on a running vehicle.
 */
#include "publish.h"
#include "telemetry_data.h"
#include "pb_decode.h"
#include <string.h>

static uint8_t payload[PUBLISH_MAX_PAYLOAD_SIZE];
static fsae_TelemetryFrame decoded;

static bool round_trip(void)
{
    uint16_t size;
    pb_istream_t stream;
    if (!Publish_BuildFrame(PUBLISH_TOPIC_BMS_SUMMARY, payload,
                           sizeof(payload), &size)) return false;
    decoded = (fsae_TelemetryFrame)fsae_TelemetryFrame_init_zero;
    stream = pb_istream_from_buffer(payload, size);
    return pb_decode(&stream, fsae_TelemetryFrame_fields, &decoded);
}

#define CHECK(expr) do { if (!(expr)) return __LINE__; } while (0)

int Publish_PresenceRegressionTest(void)
{
    static const int32_t values[] = {12345, 0, -12345};
    memset(&g_BatteryInfo, 0, sizeof(g_BatteryInfo));
    memset(&g_CANB_LoopData, 0, sizeof(g_CANB_LoopData));

    for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
    {
        int32_t value = values[i];
        g_BatteryInfo.IVT.ValidFlags = BATTERY_IVT_TELEMETRY_VALID_MASK;
        g_BatteryInfo.IVT.CurrentMa = value;
        g_BatteryInfo.IVT.VoltageU1Mv = value;
        g_BatteryInfo.IVT.VoltageU2Mv = value;
        g_BatteryInfo.IVT.PowerW = value;
        g_BatteryInfo.IVT.EnergyWh = value;
        g_BatteryInfo.IVT.VoltageU2State = 1U;
        g_BatteryInfo.IVT.PowerState = 1U;
        g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_COMPETITION_IVT;
        g_CANB_LoopData.EnergyMeter.ValidFlags = CANB_ENERGY_VALID_CURRENT |
            CANB_ENERGY_VALID_VOLTAGE | CANB_ENERGY_VALID_POWER | CANB_ENERGY_VALID_ENERGY;
        g_CANB_LoopData.EnergyMeter.CurrentMa = value + 1;
        g_CANB_LoopData.EnergyMeter.VoltageMv = value;
        g_CANB_LoopData.EnergyMeter.PowerW = value;
        g_CANB_LoopData.EnergyMeter.EnergyWh = value;
        CHECK(round_trip());
        CHECK(decoded.has_ivt_telemetry && decoded.has_energy_meter);
#define CHECK_IVT(field) CHECK(decoded.ivt_telemetry.has_##field && decoded.ivt_telemetry.field == value)
        CHECK_IVT(current_ma);
        CHECK_IVT(voltage_u1_mv);
        CHECK_IVT(voltage_u2_mv);
        CHECK_IVT(power_w);
        CHECK_IVT(energy_wh);
        CHECK(decoded.ivt_telemetry.voltage_u2_state == 1U);
        CHECK(decoded.ivt_telemetry.power_state == 1U);
        CHECK(decoded.energy_meter.has_current_ma && decoded.energy_meter.current_ma == value + 1);
#define CHECK_METER(field) CHECK(decoded.energy_meter.has_##field && decoded.energy_meter.field == value)
        CHECK_METER(voltage_mv);
        CHECK_METER(power_w);
        CHECK_METER(energy_wh);

        /* Clear validity but retain stale numbers from the preceding packet. */
        g_BatteryInfo.IVT.ValidFlags = BATTERY_IVT_VALID_U3 | BATTERY_IVT_VALID_TEMP | BATTERY_IVT_VALID_AS;
        g_CANB_LoopData.EnergyMeter.AutoSource = CANB_ENERGY_METER_SOURCE_FS;
        g_CANB_LoopData.EnergyMeter.ValidFlags = CANB_ENERGY_VALID_STATUS;
        g_CANB_LoopData.EnergyMeter.Status = 7U;
        CHECK(round_trip());
        CHECK(!decoded.has_ivt_telemetry);
        CHECK(decoded.has_energy_meter && decoded.energy_meter.status == 7U);
        CHECK(!decoded.energy_meter.has_current_ma && !decoded.energy_meter.has_voltage_mv);
        CHECK(!decoded.energy_meter.has_power_w && !decoded.energy_meter.has_energy_wh);
        g_CANB_LoopData.EnergyMeter.ValidFlags = 0U;
        CHECK(round_trip());
        CHECK(!decoded.has_energy_meter);
    }
    return 0;
}
