#pragma once
#include <Arduino.h>

typedef enum {
    SET_ENUM,       // discrete options
    SET_SCALED,     // raw divisor = display value
    SET_INT,        // raw integer
} setting_type_t;


struct var_def_t {
    const char *cmd;            // 3-letter UART command
    const char *label;
    setting_type_t type;
    int16_t scale_div;
    uint8_t decimals;
    const char *enum_options;   // comma-separated (NULL if not enum)
};

static const var_def_t VAR_CATALOG[] PROGMEM = {
    {"IPC", "Set Pressure",     SET_SCALED, 50, 1, NULL},
    {"MPA", "Max Pressure",     SET_SCALED, 50, 1, NULL},
    {"MPI", "Min Pressure",     SET_SCALED, 50, 1, NULL},
    {"IPP", "IPAP",             SET_SCALED, 50, 1, NULL},
    {"EPP", "EPAP",             SET_SCALED, 50, 1, NULL},
    {"MXI", "Max IPAP",         SET_SCALED, 50, 1, NULL},
    {"MNE", "Min EPAP",         SET_SCALED, 50, 1, NULL},
    {"SPT", "Pressure Support", SET_SCALED, 50, 1, NULL},
    {"EEP", "EPAP",             SET_SCALED, 50, 1, NULL},  // ASV
    {"EAI", "Min EPAP",         SET_SCALED, 50, 1, NULL},  // ASVAuto
    {"EAX", "Max EPAP",         SET_SCALED, 50, 1, NULL},  // ASVAuto
    {"EPI", "EPAP",             SET_SCALED, 50, 1, NULL},  // iVAPS
    {"MNS", "Min PS",           SET_SCALED, 50, 1, NULL},
    {"MXS", "Max PS",           SET_SCALED, 50, 1, NULL},
    {"ANS", "Min PS",           SET_SCALED, 50, 1, NULL},  // ASVAuto
    {"AXS", "Max PS",           SET_SCALED, 50, 1, NULL},  // ASVAuto
    {"WPM", "Min PS",           SET_SCALED, 50, 1, NULL},  // iVAPS
    {"WPA", "Max PS",           SET_SCALED, 50, 1, NULL},  // iVAPS

    {"BKR", "Backup Rate",      SET_SCALED,  5, 1, NULL},
    {"RRT", "Resp. Rate",       SET_SCALED,  5, 1, NULL},
    {"ITN", "Ti Min",           SET_SCALED, 50, 2, NULL},
    {"ITX", "Ti Max",           SET_SCALED, 50, 2, NULL},
    {"ITT", "Ti",               SET_SCALED, 50, 2, NULL},
    {"RST", "Rise Time (ms)",   SET_INT,    1, 0, NULL},
    {"IBR", "Target Pt Rate",   SET_INT,    1, 0, NULL},

    {"EBE", "Easy-Breathe",     SET_ENUM,   1, 0, "Off,On"},
    {"VTS", "Trigger Sens.",    SET_ENUM,   1, 0, "Very Low,Low,Med,High,Very High"},
    {"VCS", "Cycle Sens.",      SET_ENUM,   1, 0, "Very Low,Low,Med,High,Very High"},
    {"AFC", "Response",         SET_ENUM,   1, 0, "Standard,Soft"},

    {"WMV", "Target Va",        SET_SCALED, 10, 1, NULL},
    {"PHT", "Height (cm)",      SET_INT,    1, 0, NULL},

    {"STP", "Start Pressure",   SET_SCALED, 50, 1, NULL},
    {"STU", "Start Pressure",   SET_SCALED, 50, 1, NULL},
    {"EPS", "Start EPAP",       SET_SCALED, 50, 1, NULL},
    {"STV", "Start EPAP",       SET_SCALED, 50, 1, NULL},
    {"STE", "Start EPAP",       SET_SCALED, 50, 1, NULL},
    {"EAS", "Start EPAP",       SET_SCALED, 50, 1, NULL},
    {"IVS", "Start EPAP",       SET_SCALED, 50, 1, NULL},

    {"EPR", "EPR Level",        SET_SCALED, 50, 0, NULL},
    {"EPA", "EPR Clinical",     SET_ENUM,   1, 0, "Off,On"},
    {"EPX", "EPR Patient",      SET_ENUM,   1, 0, "Off,On"},
    {"EPT", "EPR Type",         SET_ENUM,   1, 0, "Ramp Only,Full Time"},

    {"RMA", "Ramp",             SET_ENUM,   1, 0, "Off,On,Auto"},
    {"RMT", "Ramp Time (min)",  SET_INT,    1, 0, NULL},
    {"ALR", "Leak Alert",       SET_ENUM,   1, 0, "Off,On"},
    {"SST", "SmartStart",       SET_ENUM,   1, 0, "Off,On"},
    {"MSK", "Mask Type",        SET_ENUM,   1, 0, "Pillows,Full Face,Nasal,Pediatric"},
    {"TBT", "Tube Type",        SET_ENUM,   1, 0, "SlimLine,Standard,3m"},
    {"QFC", "Airplane Mode",    SET_ENUM,   1, 0, "Off,On"},

    {"MOP", "Therapy Mode",     SET_ENUM,   1, 0,
     "CPAP,AutoSet,APAP,S,ST,T,VAuto,ASV,ASVAuto,iVAPS,PAC,AutoSet Her"},

    {"CCO", "Climate Control",  SET_ENUM,   1, 0, "Auto,Manual"},
    {"HMS", "Humidity Level",   SET_INT,    1, 0, NULL},
    {"HMX", "Humidity",         SET_ENUM,   1, 0, "Off,On"},
    {"HTF", "Tube Temp (\xc2\xb0""F)", SET_SCALED, 10, 0, NULL},
    {"HTS", "Tube Temp (\xc2\xb0""C)", SET_SCALED, 10, 0, NULL},
    {"HTX", "Tube Heating",     SET_ENUM,   1, 0, "Off,On,Auto"},

    {"LAN", "Language",         SET_ENUM,   1, 0,
     "English,French,German,Italian,Spanish(EU),Spanish(US),Portuguese(EU),Portuguese(US),"
     "Dutch,Swedish,Danish,Norwegian,Finnish,Japanese,Russian,Turkish,"
     "Chinese(Trad),Chinese(Simp),Polish,Japanese(KN),Czech"},
    {"PRD", "Pressure Units",   SET_ENUM,   1, 0, "cmH2O,hPa"},
    {"TMU", "Temp Units",       SET_ENUM,   1, 0, "\xc2\xb0""C,\xc2\xb0""F"},
    {"ACC", "Patient Access",   SET_ENUM,   1, 0, "Plus,On"},

    {"ALV", "Alarm Volume",     SET_ENUM,   1, 0, "Low,Med,High"},
    {"HLE", "High Leak Alarm",  SET_ENUM,   1, 0, "Off,On"},
    {"NMF", "Non-Vent Mask",    SET_ENUM,   1, 0, "Off,On"},
    {"SPX", "Low SpO2 Alarm",   SET_INT,    1, 0, NULL},
    {"APX", "Apnea Alarm (s)",  SET_INT,    1, 0, NULL},
    {"LMA", "Low MV Alarm",     SET_INT,    1, 0, NULL},
    {NULL, NULL, SET_INT, 0, 0, NULL}  // sentinel
};


static const var_def_t *var_lookup(const char *cmd) {
    for (int i = 0; VAR_CATALOG[i].cmd; i++) {
        if (strcmp(VAR_CATALOG[i].cmd, cmd) == 0) return &VAR_CATALOG[i];
    }
    return NULL;
}

// Each mode lists its GRP_MODE variables in display order.
// NULL-terminated.
#define MODE_COUNT 12
#define MODE_MAX_VARS 16

static const char * const MODE_LAYOUT[MODE_COUNT][MODE_MAX_VARS] = {
    // 0: CPAP
    {"IPC", NULL},
    // 1: AutoSet
    {"MPA", "MPI", NULL},
    // 2: APAP
    {"MPA", "MPI", NULL},
    // 3: S
    {"IPP", "EPP", "BKR", "EBE", "ITN", "ITX", "RST", "VTS", "VCS", NULL},
    // 4: ST
    {"IPP", "EPP", "RRT", "ITN", "ITX", "RST", "VTS", "VCS", NULL},
    // 5: T
    {"IPP", "EPP", "RRT", "ITT", "RST", NULL},
    // 6: VAuto
    {"MXI", "MNE", "SPT", "ITN", "ITX", "VTS", "VCS", NULL},
    // 7: ASV
    {"EEP", "MNS", "MXS", NULL},
    // 8: ASVAuto
    {"EAI", "EAX", "ANS", "AXS", NULL},
    // 9: iVAPS
    {"PHT", "IBR", "WMV", "EPI", "WPM", "WPA", "ITN", "ITX", "RST", "VTS", "VCS", NULL},
    // 10: PAC
    {"IPP", "EPP", "RRT", "ITT", "RST", "VTS", NULL},
    // 11: AutoSet Her
    {"MPA", "MPI", NULL},
};

#define COMFORT_MAX_VARS 4

static const char * const COMFORT_LAYOUT[MODE_COUNT][COMFORT_MAX_VARS] = {
    // 0: CPAP
    {"STP", NULL},
    // 1: AutoSet
    {"STU", "AFC", NULL},
    // 2: APAP
    {"STU", "AFC", NULL},
    // 3: S
    {"EPS", NULL},
    // 4: ST
    {"EPS", NULL},
    // 5: T
    {"EPS", NULL},
    // 6: VAuto
    {"STV", NULL},
    // 7: ASV
    {"STE", NULL},
    // 8: ASVAuto
    {"EAS", NULL},
    // 9: iVAPS
    {"IVS", NULL},
    // 10: PAC
    {"EPS", NULL},
    // 11: AutoSet Her
    {"STU", "AFC", NULL},
};


static const char * const EPR_VARS[] = {"RMA", "RMT", "EPR", "EPA", "EPX", "EPT", NULL};

static const char * const PATIENT_VARS[] = {
    "RMA", "RMT", "ALR", "SST", "MSK", "TBT", "QFC", NULL
};

static const char * const CLIMATE_VARS[] = {
    "CCO", "HMS", "HMX", "HTF", "HTS", "HTX", NULL
};

static const char * const SYSTEM_VARS[] = {
    "LAN", "PRD", "TMU", "ACC", NULL
};

static const char * const ALARM_VARS[] = {
    "ALV", "HLE", "NMF", "SPX", "APX", "LMA", NULL
};

