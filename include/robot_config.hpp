#pragma once

#include <Arduino.h>

// Keep real credentials out of source control. Copy this file locally or edit
// these values while testing.
namespace robot_config {

constexpr const char* WIFI_SSID = "YOUR_WIFI_SSID";
constexpr const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
constexpr uint16_t UDP_PORT = 4210;

// Starter pin map. Adjust to match the motor driver you choose.
constexpr uint8_t LEFT_ENC_A = 2;
constexpr uint8_t LEFT_ENC_B = 4;
constexpr uint8_t RIGHT_ENC_A = 3;
constexpr uint8_t RIGHT_ENC_B = 5;

constexpr uint8_t LEFT_PWM = 6;
constexpr uint8_t LEFT_DIR = 7;
constexpr uint8_t RIGHT_PWM = 8;
constexpr uint8_t RIGHT_DIR = 9;

constexpr float MOTOR_GEAR_RATIO = (22.0f / 12.0f) * (22.0f / 10.0f) * (24.0f / 10.0f);
constexpr float MOTOR_RAW_CPR = 24.0f;

}
