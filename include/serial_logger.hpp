#pragma once

#include <Arduino.h>

#define LOG_LEVEL_TRACE 0
#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_WARN 3
#define LOG_LEVEL_ERROR 4
#define LOG_LEVEL_OFF 5

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_DEBUG
#endif

#ifndef LOG_BAUDRATE
#define LOG_BAUDRATE 115200
#endif

inline void log_begin() {
    Serial.begin(LOG_BAUDRATE);
    const unsigned long start = millis();
    while (!Serial && millis() - start < 1500) {
        delay(10);
    }
}

#define LOG_PRINT(level, msg) \
    do { \
        Serial.print(level); \
        Serial.print(' '); \
        Serial.println(msg); \
    } while (0)

#if LOG_LEVEL <= LOG_LEVEL_DEBUG
#define LOG_DEBUG(msg) LOG_PRINT("[DEBUG]", msg)
#else
#define LOG_DEBUG(msg)
#endif

#if LOG_LEVEL <= LOG_LEVEL_INFO
#define LOG_INFO(msg) LOG_PRINT("[INFO ]", msg)
#else
#define LOG_INFO(msg)
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARN
#define LOG_WARN(msg) LOG_PRINT("[WARN ]", msg)
#else
#define LOG_WARN(msg)
#endif

#if LOG_LEVEL <= LOG_LEVEL_ERROR
#define LOG_ERROR(msg) LOG_PRINT("[ERROR]", msg)
#else
#define LOG_ERROR(msg)
#endif
