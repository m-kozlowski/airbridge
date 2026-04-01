#pragma once
#include <Arduino.h>
#include <Print.h>

#define LOG_MAX_OUTPUTS 4

typedef enum {
    LOG_ERROR = 0,
    LOG_WARN  = 1,
    LOG_INFO  = 2,
    LOG_DEBUG = 3,
} log_level_t;

typedef enum {
    CAT_GENERAL = 0,
    CAT_BLE,
    CAT_TCP,
    CAT_OTA,
    CAT_WEB,
    CAT_ARB,
    CAT_HEALTH,
    CAT_INIT,
    CAT_COUNT
} log_cat_t;

namespace Log {
    void init();

    void printf(const char *fmt, ...);

    void logf(log_cat_t cat, log_level_t lvl, const char *fmt, ...);

    void set_level(log_level_t lvl);
    log_level_t get_level();

    void set_cat_level(log_cat_t cat, log_level_t lvl);
    log_level_t get_cat_level(log_cat_t cat);

    const char *level_name(log_level_t lvl);
    const char *cat_name(log_cat_t cat);

    void add_output(Print *out);
    void remove_output(Print *out);
}
