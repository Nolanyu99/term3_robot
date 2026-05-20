#include <Arduino.h>

#include "robot_config.hpp"

namespace {

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long CALIBRATION_TIME_MS = 5000;
constexpr unsigned long STATUS_INTERVAL_MS = 100;
constexpr uint8_t SENSOR_COUNT = 9;
constexpr uint16_t LINE_POSITION_SCALE = 1000;
constexpr uint16_t INTERSECTION_THRESHOLD = 800;
constexpr uint8_t INTERSECTION_SENSOR_COUNT = 7;

const uint8_t sensor_pins[SENSOR_COUNT] = {2, 3, 4, 5, 8, 9, 10, 11, 12};

uint16_t raw_values[SENSOR_COUNT] = {};
uint16_t min_values[SENSOR_COUNT] = {};
uint16_t max_values[SENSOR_COUNT] = {};
uint16_t calibrated_values[SENSOR_COUNT] = {};

unsigned long calibration_start_ms = 0;
unsigned long last_status_ms = 0;
bool calibration_done = false;
bool line_detected = false;

void read_rc_discharge_times() {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        pinMode(sensor_pins[i], OUTPUT);
        digitalWrite(sensor_pins[i], HIGH);
    }

    delayMicroseconds(15);

    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        pinMode(sensor_pins[i], INPUT);
        raw_values[i] = robot_config::QTR_RC_TIMEOUT_US;
    }

    const unsigned long start_time_us = micros();
    while (micros() - start_time_us < robot_config::QTR_RC_TIMEOUT_US) {
        const uint16_t elapsed_us = static_cast<uint16_t>(micros() - start_time_us);
        for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
            if (raw_values[i] == robot_config::QTR_RC_TIMEOUT_US &&
                digitalRead(sensor_pins[i]) == LOW) {
                raw_values[i] = elapsed_us;
            }
        }
    }
}

void reset_calibration() {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        min_values[i] = robot_config::QTR_RC_TIMEOUT_US;
        max_values[i] = 0;
    }
}

void update_calibration() {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        if (raw_values[i] < min_values[i]) {
            min_values[i] = raw_values[i];
        }
        if (raw_values[i] > max_values[i]) {
            max_values[i] = raw_values[i];
        }
    }
}

void update_calibrated_values() {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        const uint16_t range = max_values[i] - min_values[i];
        uint16_t value = 0;
        if (range > 0 && raw_values[i] > min_values[i]) {
            value = static_cast<uint16_t>(
                min<uint32_t>(
                    1000,
                    (static_cast<uint32_t>(raw_values[i] - min_values[i]) * 1000) / range));
        }

        calibrated_values[i] = robot_config::QTR_LINE_IS_HIGH_RAW ? value : 1000 - value;
    }
}

uint16_t line_peak() {
    uint16_t peak = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        const uint16_t target_value =
            robot_config::QTR_FOLLOW_BLACK_LINE ? calibrated_values[i] : 1000 - calibrated_values[i];
        if (target_value > peak) {
            peak = target_value;
        }
    }
    return peak;
}

bool update_line_found() {
    const uint16_t peak = line_peak();
    if (line_detected) {
        line_detected = peak >= robot_config::QTR_LINE_DETECT_OFF_THRESHOLD;
    } else {
        line_detected = peak >= robot_config::QTR_LINE_DETECT_ON_THRESHOLD;
    }
    return line_detected;
}

bool intersection_detected() {
    uint8_t high_count = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        const uint16_t target_value =
            robot_config::QTR_FOLLOW_BLACK_LINE ? calibrated_values[i] : 1000 - calibrated_values[i];
        if (target_value >= INTERSECTION_THRESHOLD) {
            ++high_count;
        }
    }
    return high_count >= INTERSECTION_SENSOR_COUNT;
}

int32_t estimate_line_position() {
    uint32_t weighted_sum = 0;
    uint32_t sum = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        const uint16_t target_value =
            robot_config::QTR_FOLLOW_BLACK_LINE ? calibrated_values[i] : 1000 - calibrated_values[i];
        weighted_sum += static_cast<uint32_t>(target_value) * i * LINE_POSITION_SCALE;
        sum += target_value;
    }

    if (sum == 0) {
        return -1;
    }

    return static_cast<int32_t>(weighted_sum / sum);
}

void print_array(const char* label, const uint16_t* values) {
    Serial.print(label);
    Serial.print("=[");
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        if (i > 0) {
            Serial.print(',');
        }
        Serial.print(values[i]);
    }
    Serial.print(']');
}

void print_surfaces() {
    Serial.print("surface=[");
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        if (i > 0) {
            Serial.print(',');
        }
        const uint16_t target_value =
            robot_config::QTR_FOLLOW_BLACK_LINE ? calibrated_values[i] : 1000 - calibrated_values[i];
        Serial.print(target_value >= 500 ? 'B' : 'W');
    }
    Serial.print(']');
}

void print_status() {
    update_calibrated_values();

    if (!calibration_done) {
        Serial.print("calibrating ");
        print_array("raw", raw_values);
        Serial.println();
        return;
    }

    const bool found = update_line_found();
    const bool intersection = intersection_detected();
    const int32_t position = found ? estimate_line_position() : -1;
    const int32_t center = ((SENSOR_COUNT - 1) * LINE_POSITION_SCALE) / 2;

    print_array("raw", raw_values);
    Serial.print(' ');
    print_array("cal", calibrated_values);
    Serial.print(' ');
    print_surfaces();
    Serial.print(" peak=");
    Serial.print(line_peak());
    Serial.print(" found=");
    Serial.print(found ? 1 : 0);
    Serial.print(" intersection=");
    Serial.print(intersection ? 1 : 0);
    Serial.print(" line=");
    Serial.print(position);
    Serial.print(" error=");
    Serial.println(position >= 0 ? position - center : 0);
}

void print_pin_header() {
    Serial.println("=== Updated QTR-RC IR test ===");
    Serial.println("Pins: S1=D2 S2=D3 S3=D4 S4=D5 S5=D8 S6=D9 S7=D10 S8=D11 S9=D12");
    Serial.println("Move the array across both floor and black line for 5 seconds.");
    Serial.print("black_line_high_raw=");
    Serial.print(robot_config::QTR_LINE_IS_HIGH_RAW ? 1 : 0);
    Serial.print(" follow_black_line=");
    Serial.println(robot_config::QTR_FOLLOW_BLACK_LINE ? 1 : 0);
}

}  // namespace

void ir_test_updated_app_setup() {
    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    reset_calibration();
    calibration_start_ms = millis();
    last_status_ms = millis();
    line_detected = false;

    print_pin_header();
}

void ir_test_updated_app_loop() {
    const unsigned long now_ms = millis();
    read_rc_discharge_times();

    if (!calibration_done) {
        update_calibration();
        if (now_ms - calibration_start_ms >= CALIBRATION_TIME_MS) {
            calibration_done = true;
            Serial.println("Calibration complete.");
            print_array("min", min_values);
            Serial.print(' ');
            print_array("max", max_values);
            Serial.println();
        }
    }

    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;
        print_status();
    }
}
