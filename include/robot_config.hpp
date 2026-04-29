#pragma once

#include <Arduino.h>

// Keep real credentials out of source control. Copy this file locally or edit
// these values while testing.
namespace robot_config {

constexpr const char* WIFI_SSID = "爸爸";
constexpr const char* WIFI_PASSWORD = "yu200702";
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

constexpr float MOTOR_GEAR_RATIO = (22.0f / 12.0f) * (22.0f / 10.0f) * (24.0f / 10.0f);
constexpr float MOTOR_RAW_CPR = 24.0f;

}
