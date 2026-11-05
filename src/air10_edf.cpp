#include "air10_edf.h"

#include <stdio.h>
#include <string.h>

#include "crc.h"

namespace Air10Edf {
namespace {

constexpr size_t EDF_FIXED_HEADER_SIZE = 256;
constexpr size_t EDF_SIGNAL_HEADER_SIZE = 256;
constexpr size_t EDF_PATIENT_OFFSET = 8;
constexpr size_t EDF_PATIENT_WIDTH = 80;
constexpr size_t EDF_PATIENT_CRC_SOURCE_OFFSET = 0x19;
constexpr size_t EDF_PATIENT_CRC_SOURCE_LENGTH = 0xe7;
constexpr size_t EDF_RECORD_COUNT_OFFSET = 236;
constexpr size_t EDF_RECORD_COUNT_WIDTH = 8;

const char *const STR_TAGS[] = {
    "LSD", "ONT", "OFT", "MSE", "OND", "THD", "PHM", "MOP",
    "RMA", "RMT", "STP", "IPC", "EPA", "EPX", "EPR", "EPT",
    "EPS", "IPP", "EPP", "EBE", "STV", "MXI", "MNE", "SPT",
    "RSC", "RST", "VCS", "VTS", "ITX", "ITN",
    "AFC", "STU", "MPA", "MPI",
    "STE", "EEP", "MXS", "MNS", "EAS", "EAX", "EAI", "AXS", "ANS",
    "SST", "ACC", "ABF", "ALR", "MSK", "TBT", "CCO", "HMX",
    "HMS", "HTX", "HTS", "HME", "HTB", "HUM",
    "BP9", "BP5", "RF9", "RF5", "BFM", "ABM", "HHM", "HTM",
    "TPM", "HPM", "SOM", "SO9", "SOX", "SAU", "CSD", "VCR",
    "MSP", "PM9",
    "PMA", "PIM", "PI9", "PIA", "PEM", "PE9", "PEA", "LKM",
    "LK9", "LK7", "LMX", "VTM", "VT9", "VTA", "RRM", "RR9",
    "RRA", "TVM", "TV9", "TVA", "IEM", "IE9", "IEA", "ISM",
    "IS9", "ISA", "VAM", "VA9", "VAA", "AHI", "HIS", "AIS",
    "CLI", "OPI", "UAI", "RIN", "SYS", "SYT", "SYC", "SYH",
    "BRR", "RRT", "ITT", "VSR", "DCR",
};

const Schema BRP = {
    FileKind::BRP, "BRP", "EDF", BRP_SIGNALS,
    sizeof(BRP_SIGNALS) / sizeof(BRP_SIGNALS[0]), 0, 60,
};
const Schema BRP_TCV = {
    FileKind::BRP, "BRP", "EDF", BRP_TCV_SIGNALS,
    sizeof(BRP_TCV_SIGNALS) / sizeof(BRP_TCV_SIGNALS[0]), 0, 60,
};
const Schema PLD = {
    FileKind::PLD, "PLD", "EDF", PLD_SIGNALS,
    PLD_MAX_SIGNALS, 0, 60,
};
const Schema SAD = {
    FileKind::SAD, "SAD", "EDF", SAD_SIGNALS,
    sizeof(SAD_SIGNALS) / sizeof(SAD_SIGNALS[0]), 0, 60,
};
const Schema EVE = {
    FileKind::EVE, "EVE", "EDF+D", EVE_SIGNALS,
    sizeof(EVE_SIGNALS) / sizeof(EVE_SIGNALS[0]), 38, 0,
};
const Schema CSL = {
    FileKind::CSL, "CSL", "EDF+D", CSL_SIGNALS,
    sizeof(CSL_SIGNALS) / sizeof(CSL_SIGNALS[0]), 30, 0,
};
const Schema STR = {
    FileKind::STR, "STR", "EDF", STR_SIGNALS,
    sizeof(STR_SIGNALS) / sizeof(STR_SIGNALS[0]), 0, 86400,
};

static_assert(sizeof(STR_SIGNALS) / sizeof(STR_SIGNALS[0]) ==
                  sizeof(STR_TAGS) / sizeof(STR_TAGS[0]),
              "STR tags and signals must stay aligned");

void append_field(uint8_t *dst, size_t capacity, size_t &offset,
                  const char *text, size_t width) {
    if (!dst || offset + width > capacity) return;
    memset(dst + offset, ' ', width);
    if (text) memcpy(dst + offset, text, strnlen(text, width));
    offset += width;
}

void append_u32(uint8_t *dst, size_t capacity, size_t &offset,
                uint32_t value, size_t width) {
    char text[16];
    snprintf(text, sizeof(text), "%lu", static_cast<unsigned long>(value));
    append_field(dst, capacity, offset, text, width);
}

bool append_bytes(uint8_t *dst, size_t capacity, size_t &offset,
                  const void *src, size_t len) {
    if (!dst || !src || offset + len > capacity) return false;
    memcpy(dst + offset, src, len);
    offset += len;
    return true;
}

bool append_byte(uint8_t *dst, size_t capacity, size_t &offset, uint8_t value) {
    return append_bytes(dst, capacity, offset, &value, 1);
}

bool append_i16_le(uint8_t *dst, size_t capacity, size_t &offset, int16_t value) {
    uint8_t bytes[2] = {
        static_cast<uint8_t>(value),
        static_cast<uint8_t>(static_cast<uint16_t>(value) >> 8),
    };
    return append_bytes(dst, capacity, offset, bytes, sizeof(bytes));
}

void finalize_patient_id(uint8_t *header, size_t size) {
    memset(header + EDF_PATIENT_OFFSET, ' ', EDF_PATIENT_WIDTH);
    memcpy(header + EDF_PATIENT_OFFSET, "X X X X 0000 0000", 17);

    const uint16_t fixed_crc = crc16_ccitt(
        header + EDF_PATIENT_CRC_SOURCE_OFFSET,
        EDF_PATIENT_CRC_SOURCE_LENGTH);
    const uint16_t signal_crc = crc16_ccitt(
        header + EDF_FIXED_HEADER_SIZE, size - EDF_FIXED_HEADER_SIZE);

    char patient[24];
    snprintf(patient, sizeof(patient), "X X X X %04X %04X",
             static_cast<unsigned>(fixed_crc),
             static_cast<unsigned>(signal_crc));
    memcpy(header + EDF_PATIENT_OFFSET, patient, strlen(patient));
}

}  // namespace

const Schema &brp_schema(bool include_tcv) {
    return include_tcv ? BRP_TCV : BRP;
}

const Schema &pld_schema() { return PLD; }

Schema pld_schema(bool include_int, bool include_ext,
                  SignalSpec (&signals)[PLD_MAX_SIGNALS]) {
    Schema selected = PLD;
    selected.signals = signals;
    selected.signal_count = 0;
    for (const SignalSpec &signal : PLD_SIGNALS) {
        if (!include_int && (!strcmp(signal.label, "Ti.2s") ||
                             !strcmp(signal.label, "B5ITime.2s"))) continue;
        if (!include_ext && !strcmp(signal.label, "B5ETime.2s")) continue;
        signals[selected.signal_count++] = signal;
    }
    return selected;
}

const Schema &sad_schema() { return SAD; }
const Schema &eve_schema() { return EVE; }
const Schema &csl_schema() { return CSL; }
const Schema &str_schema() { return STR; }

const char *str_signal_tag(size_t signal_index) {
    return signal_index < sizeof(STR_TAGS) / sizeof(STR_TAGS[0])
               ? STR_TAGS[signal_index] : nullptr;
}

size_t header_size(const Schema &schema) {
    return EDF_FIXED_HEADER_SIZE +
           static_cast<size_t>(schema.signal_count) * EDF_SIGNAL_HEADER_SIZE;
}

size_t record_size(const Schema &schema) {
    size_t samples = 0;
    for (uint8_t i = 0; i < schema.signal_count; i++)
        samples += schema.signals[i].samples_per_record;
    return samples * sizeof(int16_t);
}

size_t numeric_sample_count(const Schema &schema) {
    if (schema.annotation_payload_bytes || schema.signal_count < 2) return 0;
    size_t samples = 0;
    for (uint8_t i = 0; i + 1 < schema.signal_count; i++)
        samples += schema.signals[i].samples_per_record;
    return samples;
}

bool render_header(const Schema &schema, const HeaderInfo &info,
                   uint8_t *dst, size_t capacity, size_t &written) {
    written = 0;
    const size_t required = header_size(schema);
    if (!dst || !schema.signals || !schema.signal_count || capacity < required)
        return false;

    memset(dst, ' ', required);
    size_t offset = 0;
    append_field(dst, capacity, offset, "0", 8);
    append_field(dst, capacity, offset, "", 80);
    append_field(dst, capacity, offset, info.recording_id, 80);
    append_field(dst, capacity, offset, info.start_date, 8);
    append_field(dst, capacity, offset, info.start_time, 8);
    append_u32(dst, capacity, offset, static_cast<uint32_t>(required), 8);
    append_field(dst, capacity, offset, schema.reserved, 44);
    append_u32(dst, capacity, offset, info.record_count, 8);
    append_field(dst, capacity, offset,
                 schema.record_duration_seconds == 86400 ? "86400.00" :
                 schema.record_duration_seconds ? "60.00" : "0.00", 8);
    append_u32(dst, capacity, offset, schema.signal_count, 4);

    for (uint8_t i = 0; i < schema.signal_count; i++)
        append_field(dst, capacity, offset, schema.signals[i].label, 16);
    for (uint8_t i = 0; i < schema.signal_count; i++)
        append_field(dst, capacity, offset, "", 80);
    for (uint8_t i = 0; i < schema.signal_count; i++)
        append_field(dst, capacity, offset, schema.signals[i].dimension, 8);
    for (uint8_t i = 0; i < schema.signal_count; i++)
        append_field(dst, capacity, offset, schema.signals[i].physical_min, 8);
    for (uint8_t i = 0; i < schema.signal_count; i++)
        append_field(dst, capacity, offset, schema.signals[i].physical_max, 8);
    for (uint8_t i = 0; i < schema.signal_count; i++)
        append_field(dst, capacity, offset, schema.signals[i].digital_min, 8);
    for (uint8_t i = 0; i < schema.signal_count; i++)
        append_field(dst, capacity, offset, schema.signals[i].digital_max, 8);
    for (uint8_t i = 0; i < schema.signal_count; i++)
        append_field(dst, capacity, offset, "", 80);
    for (uint8_t i = 0; i < schema.signal_count; i++)
        append_u32(dst, capacity, offset, schema.signals[i].samples_per_record, 8);
    for (uint8_t i = 0; i < schema.signal_count; i++)
        append_field(dst, capacity, offset, "", 32);

    if (offset != required) return false;
    finalize_patient_id(dst, required);
    written = required;
    return true;
}

bool update_record_count(uint8_t *header, size_t size,
                         uint32_t record_count) {
    if (!header || size < EDF_FIXED_HEADER_SIZE ||
        (size - EDF_FIXED_HEADER_SIZE) % EDF_SIGNAL_HEADER_SIZE != 0) {
        return false;
    }

    char text[16];
    const int len = snprintf(text, sizeof(text), "%lu",
                             static_cast<unsigned long>(record_count));
    if (len <= 0 || static_cast<size_t>(len) > EDF_RECORD_COUNT_WIDTH)
        return false;
    memset(header + EDF_RECORD_COUNT_OFFSET, ' ', EDF_RECORD_COUNT_WIDTH);
    memcpy(header + EDF_RECORD_COUNT_OFFSET, text, len);
    finalize_patient_id(header, size);
    return true;
}

bool render_numeric_record(const Schema &schema, const int16_t *samples,
                           size_t sample_count, uint8_t *dst,
                           size_t capacity, size_t &written) {
    written = 0;
    const size_t required = record_size(schema);
    if (!samples || sample_count != numeric_sample_count(schema) ||
        !dst || capacity < required) {
        return false;
    }

    size_t offset = 0;
    for (size_t i = 0; i < sample_count; i++)
        if (!append_i16_le(dst, capacity, offset, samples[i])) return false;
    const uint16_t crc = crc16_ccitt(dst, offset);
    if (!append_i16_le(dst, capacity, offset, static_cast<int16_t>(crc)))
        return false;
    written = offset;
    return offset == required;
}

bool render_annotation_record(const Schema &schema, uint32_t onset_seconds,
                              uint32_t duration_seconds, const char *label,
                              uint8_t *dst, size_t capacity, size_t &written) {
    written = 0;
    if (!schema.annotation_payload_bytes || !label ||
        capacity < record_size(schema)) {
        return false;
    }

    uint8_t payload[38] = {};
    if (schema.annotation_payload_bytes > sizeof(payload)) return false;
    size_t offset = 0;
    char text[16];
    if (!append_bytes(payload, sizeof(payload), offset, "+0", 2) ||
        !append_byte(payload, sizeof(payload), offset, 0x14) ||
        !append_byte(payload, sizeof(payload), offset, 0x14) ||
        !append_byte(payload, sizeof(payload), offset, 0x00)) {
        return false;
    }
    snprintf(text, sizeof(text), "+%lu", static_cast<unsigned long>(onset_seconds));
    if (!append_bytes(payload, schema.annotation_payload_bytes, offset,
                      text, strlen(text)) ||
        !append_byte(payload, schema.annotation_payload_bytes, offset, 0x15)) {
        return false;
    }
    snprintf(text, sizeof(text), "%lu", static_cast<unsigned long>(duration_seconds));
    if (!append_bytes(payload, schema.annotation_payload_bytes, offset,
                      text, strlen(text)) ||
        !append_byte(payload, schema.annotation_payload_bytes, offset, 0x14) ||
        !append_bytes(payload, schema.annotation_payload_bytes, offset,
                      label, strlen(label)) ||
        !append_byte(payload, schema.annotation_payload_bytes, offset, 0x14) ||
        !append_byte(payload, schema.annotation_payload_bytes, offset, 0x00)) {
        return false;
    }

    memcpy(dst, payload, schema.annotation_payload_bytes);
    size_t out = schema.annotation_payload_bytes;
    const uint16_t crc = crc16_ccitt(payload, schema.annotation_payload_bytes);
    if (!append_i16_le(dst, capacity, out, static_cast<int16_t>(crc)))
        return false;
    written = out;
    return out == record_size(schema);
}

}  // namespace Air10Edf
