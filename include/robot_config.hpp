#pragma once

#include <Arduino.h>

// Keep real credentials out of source control. Copy this file locally or edit
// these values while testing.
namespace robot_config {

constexpr const char* WIFI_SSID = "PhaseSpaceNetwork_2.4G";
constexpr const char* WIFI_PASSWORD = "8igMacNet";
constexpr uint16_t UDP_PORT = 4210;

// Starter pin map. Adjust to match the motor driver you choose.
constexpr uint8_t LEFT_ENC_A = 22;
constexpr uint8_t LEFT_ENC_B = 23;
constexpr uint8_t RIGHT_ENC_A = 24;
constexpr uint8_t RIGHT_ENC_B = 25;

// Pololu Dual VNH5019 Motor Driver Shield default Arduino pin mapping.
constexpr uint8_t LEFT_IN_A = 2;
constexpr uint8_t LEFT_IN_B = 4;
constexpr uint8_t LEFT_PWM = 9;
constexpr uint8_t LEFT_ENABLE = 6;
constexpr uint8_t RIGHT_IN_A = 7;
constexpr uint8_t RIGHT_IN_B = 8;
constexpr uint8_t RIGHT_PWM = 10;
constexpr uint8_t RIGHT_ENABLE = 12;

constexpr float MOTOR_GEAR_RATIO = 50.0f;
constexpr float MOTOR_RAW_CPR = 24.0f;

// QTR reflectance sensor array test pin map. On Arduino GIGA, A8-A11 are
// special analog-only pins, so qtr_test_app.cpp maps these indexes explicitly.
constexpr uint8_t QTR_FIRST_ANALOG_PIN = 1;
constexpr size_t QTR_SENSOR_COUNT = 9;

// Set to 255 if the emitter LEDs are wired directly to power.
constexpr uint8_t QTR_EMITTER_PIN = 255;

// Flip this after checking raw values if black tape gives lower readings.
constexpr bool QTR_LINE_IS_HIGH_RAW = true;
constexpr bool QTR_FOLLOW_BLACK_LINE = true;
constexpr uint8_t QTR_RAW_SAMPLE_COUNT = 8;
constexpr uint8_t QTR_SMOOTHING_PERCENT = 35;
constexpr uint16_t QTR_LINE_DETECT_ON_THRESHOLD = 650;
constexpr uint16_t QTR_LINE_DETECT_OFF_THRESHOLD = 450;
constexpr uint16_t QTR_SURFACE_DECISION_MARGIN = 120;

}
