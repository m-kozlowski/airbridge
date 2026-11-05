#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Air10Edf {

enum class FileKind : uint8_t {
    BRP,
    PLD,
    SAD,
    EVE,
    CSL,
    STR,
};

struct SignalSpec {
    const char *label;
    const char *dimension;
    const char *physical_min;
    const char *physical_max;
    const char *digital_min;
    const char *digital_max;
    uint16_t samples_per_record;
};

struct Schema {
    FileKind kind;
    const char *suffix;
    const char *reserved;
    const SignalSpec *signals;
    uint8_t signal_count;
    uint16_t annotation_payload_bytes;
    uint32_t record_duration_seconds;
};

struct HeaderInfo {
    const char *recording_id;
    const char *start_date;
    const char *start_time;
    uint32_t record_count;
};

inline constexpr SignalSpec BRP_SIGNALS[] = {
    {"Flow.40ms", "L/s", "-2.00", "3.00", "-1000", "1500", 1500},
    {"Press.40ms", "cmH2O", "0.00", "40.00", "0", "2000", 1500},
    {"Crc16", "", "-32768", "32767", "-32768", "32767", 1},
};

inline constexpr SignalSpec BRP_TCV_SIGNALS[] = {
    {"Flow.40ms", "L/s", "-2.00", "3.00", "-1000", "1500", 1500},
    {"Press.40ms", "cmH2O", "0.00", "40.00", "0", "2000", 1500},
    {"TrigCycEvt.40ms", "", "0", "16", "0", "16", 1500},
    {"Crc16", "", "-32768", "32767", "-32768", "32767", 1},
};

// Breath timing is selected per BRH schema.
inline constexpr SignalSpec PLD_SIGNALS[] = {
    {"MaskPress.2s", "cmH2O", "0.00", "40.00", "0", "2000", 30},
    {"Press.2s", "cmH2O", "0.00", "30.00", "0", "1500", 30},
    {"EprPress.2s", "cmH2O", "0.00", "30.00", "0", "1500", 30},
    {"Leak.2s", "L/s", "0.00", "2.00", "0", "100", 30},
    {"RespRate.2s", "bpm", "0.00", "50.00", "0", "250", 30},
    {"TidVol.2s", "L", "0.00", "4.00", "0", "200", 30},
    {"MinVent.2s", "L/min", "0.00", "30.00", "0", "240", 30},
    {"TgtVent.2s", "L/min", "0.00", "30.00", "0", "240", 30},
    {"IERatio.2s", "%", "0.00", "200.00", "0", "200", 30},
    {"Snore.2s", "", "0.00", "5.00", "0", "250", 30},
    {"FlowLim.2s", "", "0.00", "1.00", "0", "100", 30},
    {"B5ITime.2s", "seconds", "0.00", "10.00", "0", "500", 30},
    {"B5ETime.2s", "seconds", "0.00", "10.00", "0", "500", 30},
    {"Ti.2s", "seconds", "0.00", "10.00", "0", "500", 30},
    {"AlvMinVent.2s", "L/min", "0.00", "30.00", "0", "240", 30},
    // Optional: also enable RCR/RTR in EdfRecorder::sample_pld().
    // {"CLRatio.2s", "%", "0.00", "100.00", "0", "100", 30},
    // {"TRRatio.2s", "%", "0.00", "100.00", "0", "100", 30},
    {"Crc16", "", "-32768", "32767", "-32768", "32767", 1},
};

inline constexpr SignalSpec SAD_SIGNALS[] = {
    {"Pulse.1s", "bpm", "18.00", "300.00", "18", "300", 60},
    {"SpO2.1s", "%", "0.00", "100.00", "0", "100", 60},
    {"Crc16", "", "-32768", "32767", "-32768", "32767", 1},
};

inline constexpr SignalSpec EVE_SIGNALS[] = {
    {"EDF Annotations", "", "0.00", "86400.00", "-32768", "32767", 19},
    {"Crc16", "", "-32768", "32767", "-32768", "32767", 1},
};

inline constexpr SignalSpec CSL_SIGNALS[] = {
    {"EDF Annotations", "", "0.00", "86400.00", "-32768", "32767", 15},
    {"Crc16", "", "-32768", "32767", "-32768", "32767", 1},
};

// Airbreak SX567 STR superset. These bounds follow the EDF metadata rather
// than narrowing settings to the active model's current configuration range.
inline constexpr SignalSpec STR_SIGNALS[] = {
    {"Date", "", "0.00", "24836.00", "0", "24836", 1},
    {"MaskOn", "min.", "0.00", "1440.00", "0", "1440", 10},
    {"MaskOff", "min.", "0.00", "1440.00", "0", "1440", 10},
    {"MaskEvents", "", "0.00", "255.00", "0", "255", 1},
    {"Duration", "min.", "0.00", "1440.00", "0", "1440", 1},
    {"OnDuration", "min.", "0.00", "1440.00", "0", "1440", 1},
    {"PatientHours", "hrs", "0.00", "65534.00", "0", "65534", 1},
    {"Mode", "", "0", "16", "0", "16", 1},
    {"S.RampEnable", "", "0", "16", "0", "16", 1},
    {"S.RampTime", "min.", "5.00", "45.00", "5", "45", 1},
    {"S.C.StartPress", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.C.Press", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.EPR.ClinEnable", "", "0", "16", "0", "16", 1},
    {"S.EPR.EPREnable", "", "0", "16", "0", "16", 1},
    {"S.EPR.Level", "cmH2O", "1.00", "3.00", "50", "150", 1},
    {"S.EPR.EPRType", "", "0", "16", "0", "16", 1},
    {"S.BL.StartPress", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.BL.IPAP", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.BL.EPAP", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.EasyBreathe", "", "0", "16", "0", "16", 1},
    {"S.VA.StartPress", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.VA.MaxIPAP", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.VA.MinEPAP", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.VA.PS", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"S.RiseEnable", "", "0", "16", "0", "16", 1},
    {"S.RiseTime", "ms", "150.00", "900.00", "150", "900", 1},
    {"S.Cycle", "", "0", "16", "0", "16", 1},
    {"S.Trigger", "", "0", "16", "0", "16", 1},
    {"S.TiMax", "seconds", "0.30", "4.00", "15", "200", 1},
    {"S.TiMin", "seconds", "0.10", "4.00", "5", "200", 1},
    {"S.AS.Comfort", "", "0", "16", "0", "16", 1},
    {"S.AS.StartPress", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.AS.MaxPress", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.AS.MinPress", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.AV.StartPress", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.AV.EPAP", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.AV.MaxPS", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"S.AV.MinPS", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"S.AA.StartPress", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.AA.MaxEPAP", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.AA.MinEPAP", "cmH2O", "1.00", "30.00", "50", "1500", 1},
    {"S.AA.MaxPS", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"S.AA.MinPS", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"S.SmartStart", "", "0", "16", "0", "16", 1},
    {"S.PtAccess", "", "0", "16", "0", "16", 1},
    {"S.ABFilter", "", "0", "16", "0", "16", 1},
    {"S.LeakAlert", "", "0", "16", "0", "16", 1},
    {"S.Mask", "", "0", "16", "0", "16", 1},
    {"S.Tube", "", "0", "16", "0", "16", 1},
    {"S.ClimateControl", "", "0", "16", "0", "16", 1},
    {"S.HumEnable", "", "0", "16", "0", "16", 1},
    {"S.HumLevel", "", "1.00", "8.00", "1", "8", 1},
    {"S.TempEnable", "", "0", "16", "0", "16", 1},
    {"S.Temp", "\xC2\xB0" "C", "16.00", "30.00", "160", "300", 1},
    {"S.ExternalHum", "", "0", "16", "0", "16", 1},
    {"HeatedTube", "", "0", "16", "0", "16", 1},
    {"Humidifier", "", "0", "16", "0", "16", 1},
    {"BlowPress.95", "cmH2O", "0.00", "45.00", "0", "2250", 1},
    {"BlowPress.5", "cmH2O", "0.00", "45.00", "0", "2250", 1},
    {"Flow.95", "L/s", "0.00", "3.00", "0", "1500", 1},
    {"Flow.5", "L/s", "0.00", "3.00", "0", "1500", 1},
    {"BlowFlow.50", "L/min", "0.00", "250.00", "0", "25000", 1},
    {"AmbHumidity.50", "mg/L", "0.00", "100.00", "0", "100", 1},
    {"HumTemp.50", "\xC2\xB0" "C", "0.00", "95.00", "0", "950", 1},
    {"HTubeTemp.50", "\xC2\xB0" "C", "0.00", "40.00", "0", "400", 1},
    {"HTubePow.50", "", "0.00", "100.00", "0", "100", 1},
    {"HumPow.50", "", "0.00", "100.00", "0", "100", 1},
    {"SpO2.50", "%", "0.00", "100.00", "0", "100", 1},
    {"SpO2.95", "%", "0.00", "100.00", "0", "100", 1},
    {"SpO2.Max", "%", "0.00", "100.00", "0", "100", 1},
    {"SpO2Thresh", "min.", "0.00", "1440.00", "0", "1440", 1},
    {"CSR", "", "0.00", "1440.00", "0", "1440", 1},
    {"SpontCyc%", "%", "0.00", "100.00", "0", "200", 1},
    {"MaskPress.50", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"MaskPress.95", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"MaskPress.Max", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"TgtIPAP.50", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"TgtIPAP.95", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"TgtIPAP.Max", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"TgtEPAP.50", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"TgtEPAP.95", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"TgtEPAP.Max", "cmH2O", "0.00", "30.00", "0", "1500", 1},
    {"Leak.50", "L/s", "0.00", "2.00", "0", "100", 1},
    {"Leak.95", "L/s", "0.00", "2.00", "0", "100", 1},
    {"Leak.70", "L/s", "0.00", "2.00", "0", "100", 1},
    {"Leak.Max", "L/s", "0.00", "2.00", "0", "100", 1},
    {"MinVent.50", "L/min", "0.00", "30.00", "0", "240", 1},
    {"MinVent.95", "L/min", "0.00", "30.00", "0", "240", 1},
    {"MinVent.Max", "L/min", "0.00", "30.00", "0", "240", 1},
    {"RespRate.50", "bpm", "0.00", "50.00", "0", "250", 1},
    {"RespRate.95", "bpm", "0.00", "50.00", "0", "250", 1},
    {"RespRate.Max", "bpm", "0.00", "50.00", "0", "250", 1},
    {"TidVol.50", "L", "0.00", "4.00", "0", "200", 1},
    {"TidVol.95", "L", "0.00", "4.00", "0", "200", 1},
    {"TidVol.Max", "L", "0.00", "4.00", "0", "200", 1},
    {"IERatio.50", "%", "0.00", "200.00", "0", "200", 1},
    {"IERatio.95", "%", "0.00", "200.00", "0", "200", 1},
    {"IERatio.Max", "%", "0.00", "200.00", "0", "200", 1},
    {"Ti.50", "seconds", "0.00", "10.00", "0", "500", 1},
    {"Ti.95", "seconds", "0.00", "10.00", "0", "500", 1},
    {"Ti.Max", "seconds", "0.00", "10.00", "0", "500", 1},
    {"TgtVent.50", "L/min", "0.00", "30.00", "0", "240", 1},
    {"TgtVent.95", "L/min", "0.00", "30.00", "0", "240", 1},
    {"TgtVent.Max", "L/min", "0.00", "30.00", "0", "240", 1},
    {"AHI", "", "0.00", "240.00", "0", "2400", 1},
    {"HI", "", "0.00", "240.00", "0", "2400", 1},
    {"AI", "", "0.00", "240.00", "0", "2400", 1},
    {"OAI", "", "0.00", "240.00", "0", "2400", 1},
    {"CAI", "", "0.00", "240.00", "0", "2400", 1},
    {"UAI", "", "0.00", "240.00", "0", "2400", 1},
    {"RIN", "", "0.00", "240.00", "0", "2400", 1},
    {"Fault.Device", "", "0", "16", "0", "16", 1},
    {"Fault.Alarm", "", "0", "16", "0", "16", 1},
    {"Fault.Humidifier", "", "0", "16", "0", "16", 1},
    {"Fault.HeatedTube", "", "0", "16", "0", "16", 1},
    {"S.BL.BackupRate", "bpm", "0.00", "50.00", "0", "250", 1},
    {"S.BL.RespRate", "bpm", "5.00", "50.00", "25", "250", 1},
    {"S.Ti", "seconds", "0.30", "4.00", "15", "200", 1},
    {"SpontTrig%", "%", "0.00", "100.00", "0", "200", 1},
    {"Crc16", "", "-32768", "32767", "-32768", "32767", 1},
};

constexpr size_t PLD_MAX_SIGNALS = sizeof(PLD_SIGNALS) / sizeof(PLD_SIGNALS[0]);

const Schema &brp_schema(bool include_tcv);
const Schema &pld_schema();
// The caller owns the filtered signal array for the lifetime of the schema.
Schema pld_schema(bool include_int, bool include_ext,
                  SignalSpec (&signals)[PLD_MAX_SIGNALS]);
const Schema &sad_schema();
const Schema &eve_schema();
const Schema &csl_schema();
const Schema &str_schema();
const char *str_signal_tag(size_t signal_index);

size_t header_size(const Schema &schema);
size_t record_size(const Schema &schema);
size_t numeric_sample_count(const Schema &schema);

bool render_header(const Schema &schema, const HeaderInfo &info,
                   uint8_t *dst, size_t capacity, size_t &written);
bool update_record_count(uint8_t *header, size_t header_size,
                         uint32_t record_count);
bool render_numeric_record(const Schema &schema, const int16_t *samples,
                           size_t sample_count, uint8_t *dst,
                           size_t capacity, size_t &written);
bool render_annotation_record(const Schema &schema, uint32_t onset_seconds,
                              uint32_t duration_seconds, const char *label,
                              uint8_t *dst, size_t capacity, size_t &written);

}  // namespace Air10Edf
