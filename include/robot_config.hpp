#pragma once

#include <Arduino.h>

// Keep real credentials out of source control. Copy this file locally or edit
// these values while testing.
namespace robot_config {

//constexpr const char* WIFI_SSID = "PhaseSpaceNetwork_2.4G";
//constexpr const char* WIFI_PASSWORD = "8igMacNet";
constexpr const char* WIFI_SSID = "PhaseSpaceNetwork_2.4G";
constexpr const char* WIFI_PASSWORD = "8igMacNet";
constexpr uint16_t UDP_PORT = 4210;

// Final robot demo feature switches. If a pin is wrong or not wired yet, turn
// that feature off here without changing the state machine.
constexpr bool ENABLE_TOP_RGB = true;
constexpr bool ENABLE_MECHANICAL_KILL_BUTTON = true;
constexpr bool ENABLE_REVIVE_BUTTON = true;
constexpr bool ENABLE_QTR_STATUS = true;
constexpr bool ENABLE_ULTRASONIC_STATUS = true;
constexpr bool ENABLE_RFID_STATUS = true;
constexpr bool ENABLE_SEED_DISPENSER = true;

// Common-anode RGB LED on the top tapping mechanism.
// For common-anode LEDs, 0 is on and 255 is off.
constexpr int TOP_RGB_RED_PIN = 9;
constexpr int TOP_RGB_BLUE_PIN = 10;
constexpr int TOP_RGB_GREEN_PIN = 11;
constexpr bool TOP_RGB_COMMON_ANODE = true;

// Mechanical kill switch button. Uses INPUT_PULLUP: pressed = LOW.
constexpr int MECHANICAL_KILL_BUTTON_PIN = 2;

// Front revive interface button. Uses INPUT_PULLUP: pressed = LOW.
// D50 is used by the QTR-RC array in the integrated demo, so use a free pin.
constexpr int REVIVE_BUTTON_PIN = 40;

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

constexpr float MOTOR_GEAR_RATIO = 100.0f;
constexpr float MOTOR_RAW_CPR = 24.0f;

// QTR-RC reflectance sensor array. Physical sensors S1-S9 are wired to
// Arduino digital pins D45-D53 and read by measuring RC discharge time.
constexpr uint8_t QTR_FIRST_ANALOG_PIN = 0;
constexpr size_t QTR_SENSOR_COUNT = 9;
constexpr uint16_t QTR_RC_TIMEOUT_US = 1000;

// Set to 255 if the emitter LEDs are wired directly to power.
constexpr uint8_t QTR_EMITTER_PIN = 255;

// Flip this after checking raw values if black tape gives lower readings.
constexpr bool QTR_LINE_IS_HIGH_RAW = true;
constexpr bool QTR_FOLLOW_BLACK_LINE = true;
constexpr uint8_t QTR_RAW_SAMPLE_COUNT = 8;
constexpr uint8_t QTR_SMOOTHING_PERCENT = 35;
constexpr uint16_t QTR_LINE_DETECT_ON_THRESHOLD = 650;
constexpr uint16_t QTR_LINE_DETECT_OFF_THRESHOLD = 450;
constexpr uint16_t QTR_DEMO_LINE_DETECT_ON_THRESHOLD = 300;
constexpr uint16_t QTR_DEMO_LINE_DETECT_OFF_THRESHOLD = 180;
constexpr bool QTR_DEMO_AUTO_POLARITY = true;
constexpr uint16_t QTR_SURFACE_DECISION_MARGIN = 120;

// Two-servo seed dispenser / planter. The upper gate isolates one seed; the
// lower gate releases it into the tube. D45 is used by the QTR-RC array in the
// integrated demo, so the lower servo uses D43 here.
constexpr int SEED_UPPER_SERVO_PIN = 44;
constexpr int SEED_LOWER_SERVO_PIN = 43;
constexpr int SEED_UPPER_CLOSED_ANGLE = 90;
constexpr int SEED_UPPER_OPEN_ANGLE = 0;
constexpr int SEED_LOWER_CLOSED_ANGLE = 0;
constexpr int SEED_LOWER_OPEN_ANGLE = 90;
constexpr int SEED_SERVO_MIN_PULSE_US = 500;
constexpr int SEED_SERVO_MAX_PULSE_US = 2500;
constexpr unsigned long SEED_SERVO_PERIOD_US = 20000;
constexpr unsigned long SEED_UPPER_OPEN_MS = 700;
constexpr unsigned long SEED_UPPER_CLOSE_MS = 1000;
constexpr unsigned long SEED_LOWER_OPEN_MS = 700;
constexpr unsigned long SEED_LOWER_CLOSE_MS = 500;

// HC-SR04 ultrasonic distance sensor test pins.
// Echo is a 5V signal on many HC-SR04 modules; use a resistor divider before
// feeding it into the 3.3V-only Arduino GIGA input pin.
constexpr uint8_t ULTRASONIC_TRIG_PIN = 28;
constexpr uint8_t ULTRASONIC_ECHO_PIN = 29;
constexpr float ULTRASONIC_MAX_DISTANCE_CM = 400.0f;
constexpr float ULTRASONIC_WALL_THRESHOLD_CM = 20.0f;

// WS1850S / M5Stack RFID2 I2C reader. If nothing is detected, try 0x3C.
constexpr uint8_t RFID_I2C_ADDRESS = 0x28;
constexpr int RFID_RESET_PIN = -1;

}
