#include "debug_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Preferences.h>
#include <stdarg.h>

static Preferences log_prefs;

static Print *outputs[LOG_MAX_OUTPUTS] = { &Serial };
static int output_count = 1;
static SemaphoreHandle_t log_mutex = nullptr;
static log_level_t cat_levels[CAT_COUNT];

void Log::init() {
    log_mutex = xSemaphoreCreateMutex();
    log_prefs.begin("log_levels", true);
    for (int i = 0; i < CAT_COUNT; i++) {
        cat_levels[i] = (log_level_t)log_prefs.getUChar(
            Log::cat_name((log_cat_t)i), LOG_INFO);
    }
    log_prefs.end();
}

static void save_levels() {
    log_prefs.begin("log_levels", false);
    for (int i = 0; i < CAT_COUNT; i++)
        log_prefs.putUChar(Log::cat_name((log_cat_t)i), (uint8_t)cat_levels[i]);
    log_prefs.end();
}

void Log::set_level(log_level_t lvl) {
    for (int i = 0; i < CAT_COUNT; i++)
        cat_levels[i] = lvl;
    save_levels();
}

log_level_t Log::get_level() {
    return cat_levels[CAT_GENERAL];
}

void Log::set_cat_level(log_cat_t cat, log_level_t lvl) {
    if (cat < CAT_COUNT) {
        cat_levels[cat] = lvl;
        log_prefs.begin("log_levels", false);
        log_prefs.putUChar(Log::cat_name(cat), (uint8_t)lvl);
        log_prefs.end();
    }
}

log_level_t Log::get_cat_level(log_cat_t cat) {
    return (cat < CAT_COUNT) ? cat_levels[cat] : LOG_INFO;
}

const char *Log::level_name(log_level_t lvl) {
    switch (lvl) {
        case LOG_ERROR: return "ERROR";
        case LOG_WARN:  return "WARN";
        case LOG_INFO:  return "INFO";
        case LOG_DEBUG: return "DEBUG";
        default:        return "?";
    }
}

const char *Log::cat_name(log_cat_t cat) {
    switch (cat) {
        case CAT_GENERAL: return "GENERAL";
        case CAT_OXI:     return "OXI";
        case CAT_TCP:     return "TCP";
        case CAT_OTA:     return "OTA";
        case CAT_WEB:     return "WEB";
        case CAT_ARB:     return "ARB";
        case CAT_HEALTH:  return "HEALTH";
        case CAT_INIT:    return "INIT";
        default:          return "?";
    }
}

void Log::add_output(Print *out) {
    if (!log_mutex) return;
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    if (output_count < LOG_MAX_OUTPUTS) {
        outputs[output_count++] = out;
    }
    xSemaphoreGive(log_mutex);
}

void Log::remove_output(Print *out) {
    if (!log_mutex) return;
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    for (int i = 1; i < output_count; i++) {
        if (outputs[i] == out) {
            for (int j = i; j < output_count - 1; j++)
                outputs[j] = outputs[j + 1];
            output_count--;
            break;
        }
    }
    xSemaphoreGive(log_mutex);
}

static void log_dispatch(const char *fmt, va_list args) {
    char buf[128];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len <= 0) return;
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;

    if (log_mutex && xSemaphoreTake(log_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < output_count; i++) {
            if (outputs[i]) outputs[i]->write((const uint8_t*)buf, len);
        }
        xSemaphoreGive(log_mutex);
    } else {
        Serial.write((const uint8_t*)buf, len);
    }
}

void Log::printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    log_dispatch(fmt, args);
    va_end(args);
}

void Log::logf(log_cat_t cat, log_level_t lvl, const char *fmt, ...) {
    if (cat < CAT_COUNT && lvl > cat_levels[cat]) return;
    va_list args;
    va_start(args, fmt);
    log_dispatch(fmt, args);
    va_end(args);
}
