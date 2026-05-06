#include <Arduino.h>

#include "robot_config.hpp"

namespace {

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long CALIBRATION_TIME_MS = 5000;
constexpr unsigned long STATUS_INTERVAL_MS = 500;
constexpr uint16_t ANALOG_MAX_VALUE = 4095;
constexpr uint16_t EMITTER_DISABLED_PIN = 255;

uint16_t raw_values[robot_config::QTR_SENSOR_COUNT] = {};
uint16_t min_values[robot_config::QTR_SENSOR_COUNT] = {};
uint16_t max_values[robot_config::QTR_SENSOR_COUNT] = {};
uint16_t calibrated_values[robot_config::QTR_SENSOR_COUNT] = {};

unsigned long calibration_start_ms = 0;
unsigned long last_status_ms = 0;
bool calibration_done = false;
bool calibration_filter_ready = false;
bool line_detected = false;

uint8_t analog_pin_index_for_sensor(size_t sensor_index) {
    return robot_config::QTR_FIRST_ANALOG_PIN + sensor_index;
}

uint16_t read_giga_analog_pin(uint8_t analog_pin_index) {
    switch (analog_pin_index) {
        case 0:
            return static_cast<uint16_t>(analogRead(A0));
        case 1:
            return static_cast<uint16_t>(analogRead(A1));
        case 2:
            return static_cast<uint16_t>(analogRead(A2));
        case 3:
            return static_cast<uint16_t>(analogRead(A3));
        case 4:
            return static_cast<uint16_t>(analogRead(A4));
        case 5:
            return static_cast<uint16_t>(analogRead(A5));
        case 6:
            return static_cast<uint16_t>(analogRead(A6));
        case 7:
            return static_cast<uint16_t>(analogRead(A7));
        case 8:
            return static_cast<uint16_t>(analogRead(A8));
        case 9:
            return static_cast<uint16_t>(analogRead(A9));
        case 10:
            return static_cast<uint16_t>(analogRead(A10));
        case 11:
            return static_cast<uint16_t>(analogRead(A11));
        case 12:
            return static_cast<uint16_t>(analogRead(A12));
        case 13:
            return static_cast<uint16_t>(analogRead(A13));
        default:
            return 0;
    }
}

void read_raw_values() {
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        uint32_t sum = 0;
        const uint8_t analog_pin_index = analog_pin_index_for_sensor(i);
        for (uint8_t sample = 0; sample < robot_config::QTR_RAW_SAMPLE_COUNT; ++sample) {
            sum += read_giga_analog_pin(analog_pin_index);
        }
        raw_values[i] = static_cast<uint16_t>(sum / robot_config::QTR_RAW_SAMPLE_COUNT);
    }
}

void reset_calibration() {
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        min_values[i] = ANALOG_MAX_VALUE;
        max_values[i] = 0;
    }
}

void update_calibration() {
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        if (raw_values[i] < min_values[i]) {
            min_values[i] = raw_values[i];
        }
        if (raw_values[i] > max_values[i]) {
            max_values[i] = raw_values[i];
        }
    }
}

void update_calibrated_values() {
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        const uint16_t range = max_values[i] - min_values[i];
        uint16_t value = 0;
        if (range > 0 && raw_values[i] > min_values[i]) {
            value = static_cast<uint16_t>(
                min<uint32_t>(1000, (static_cast<uint32_t>(raw_values[i] - min_values[i]) * 1000) / range));
        }

        const uint16_t instant_value = robot_config::QTR_LINE_IS_HIGH_RAW ? value : 1000 - value;
        if (!calibration_filter_ready) {
            calibrated_values[i] = instant_value;
        } else {
            calibrated_values[i] = static_cast<uint16_t>(
                ((static_cast<uint32_t>(calibrated_values[i]) * (100 - robot_config::QTR_SMOOTHING_PERCENT)) +
                 (static_cast<uint32_t>(instant_value) * robot_config::QTR_SMOOTHING_PERCENT)) /
                100);
        }
    }
    calibration_filter_ready = true;
}

int32_t estimate_line_position() {
    uint32_t weighted_sum = 0;
    uint32_t sum = 0;

    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        const uint16_t target_value = robot_config::QTR_FOLLOW_BLACK_LINE ? calibrated_values[i] : 1000 - calibrated_values[i];
        weighted_sum += static_cast<uint32_t>(target_value) * i * 1000;
        sum += target_value;
    }

    if (sum == 0) {
        return -1;
    }

    return static_cast<int32_t>(weighted_sum / sum);
}

bool update_line_found() {
    uint16_t peak = 0;
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        const uint16_t target_value = robot_config::QTR_FOLLOW_BLACK_LINE ? calibrated_values[i] : 1000 - calibrated_values[i];
        if (target_value > peak) {
            peak = target_value;
        }
    }

    if (line_detected) {
        line_detected = peak >= robot_config::QTR_LINE_DETECT_OFF_THRESHOLD;
    } else {
        line_detected = peak >= robot_config::QTR_LINE_DETECT_ON_THRESHOLD;
    }

    return line_detected;
}

void print_values(const char* label, const uint16_t* values) {
    Serial.print(label);
    Serial.print(": ");
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        if (i > 0) {
            Serial.print(' ');
        }
        Serial.print('S');
        Serial.print(i);
        Serial.print('=');
        Serial.print(values[i]);
    }
    Serial.println();
}

void print_surface_values() {
    Serial.print("surface: ");
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        if (i > 0) {
            Serial.print(' ');
        }

        Serial.print('S');
        Serial.print(i);
        Serial.print('=');

        const uint16_t black_value = calibrated_values[i];
        const uint16_t white_value = 1000 - calibrated_values[i];

        if (black_value >= white_value + robot_config::QTR_SURFACE_DECISION_MARGIN) {
            Serial.print("black");
        } else if (white_value >= black_value + robot_config::QTR_SURFACE_DECISION_MARGIN) {
            Serial.print("white");
        } else {
            Serial.print("unknown");
        }
    }
    Serial.println();
}

void print_pin_map() {
    Serial.println("QTR analog pin map:");
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        Serial.print("S");
        Serial.print(i);
        Serial.print("=A");
        Serial.print(analog_pin_index_for_sensor(i));
        if (i + 1 < robot_config::QTR_SENSOR_COUNT) {
            Serial.print(' ');
        }
    }
    Serial.println();

    if (robot_config::QTR_EMITTER_PIN == EMITTER_DISABLED_PIN) {
        Serial.println("Emitter control: disabled in firmware");
    } else {
        Serial.print("Emitter control: D");
        Serial.println(robot_config::QTR_EMITTER_PIN);
    }

    Serial.print("Detecting ");
    Serial.print(robot_config::QTR_FOLLOW_BLACK_LINE ? "black" : "white");
    Serial.print(" line, on_threshold=");
    Serial.print(robot_config::QTR_LINE_DETECT_ON_THRESHOLD);
    Serial.print(" off_threshold=");
    Serial.println(robot_config::QTR_LINE_DETECT_OFF_THRESHOLD);
}

void print_status() {
    update_calibrated_values();
    print_values("raw", raw_values);
    if (calibration_done) {
        print_values("cal", calibrated_values);
        print_surface_values();
        const bool found = update_line_found();
        digitalWrite(LED_BUILTIN, found ? HIGH : LOW);
        Serial.print("found=");
        Serial.println(found ? 1 : 0);
        Serial.print("line=");
        if (!found) {
            Serial.println("not detected");
        } else {
            Serial.println(estimate_line_position());
        }
    }
    Serial.println();
}

}  // namespace

void qtr_test_app_setup() {
    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    analogReadResolution(12);

    if (robot_config::QTR_EMITTER_PIN != EMITTER_DISABLED_PIN) {
        pinMode(robot_config::QTR_EMITTER_PIN, OUTPUT);
        digitalWrite(robot_config::QTR_EMITTER_PIN, HIGH);
    }
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    reset_calibration();
    calibration_filter_ready = false;
    line_detected = false;
    calibration_start_ms = millis();
    last_status_ms = millis();

    Serial.println();
    Serial.println("=== QTR reflectance sensor array test ===");
    Serial.println("Move the array across both the floor and the line for 5 seconds.");
    Serial.println("After calibration, raw/cal values and estimated line position are printed.");
    print_pin_map();
    Serial.println();
}

void qtr_test_app_loop() {
    const unsigned long now_ms = millis();
    read_raw_values();

    if (!calibration_done) {
        update_calibration();
        if (now_ms - calibration_start_ms >= CALIBRATION_TIME_MS) {
            calibration_done = true;
            digitalWrite(LED_BUILTIN, LOW);
            Serial.println("Calibration complete.");
            print_values("min", min_values);
            print_values("max", max_values);
            Serial.println();
        }
    }

    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;
        print_status();
    }
}
