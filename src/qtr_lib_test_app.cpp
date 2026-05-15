#include <Arduino.h>
#include <QTRSensors.h>

#include "robot_config.hpp"

namespace {

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long CALIBRATION_TIME_MS = 5000;
constexpr unsigned long STATUS_INTERVAL_MS = 100;
constexpr uint16_t LINE_DETECT_THRESHOLD = 200;

QTRSensors qtr;

const uint8_t sensor_pins[robot_config::QTR_SENSOR_COUNT] = {
    A0, A1, A2, A3, A4, A5, A6, A7, DAC_0,
};

uint16_t raw_values[robot_config::QTR_SENSOR_COUNT] = {};
uint16_t calibrated_values[robot_config::QTR_SENSOR_COUNT] = {};
unsigned long last_status_ms = 0;

void print_values(const char* label, const uint16_t* values) {
    Serial.print(label);
    Serial.print("=[");
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        if (i > 0) {
            Serial.print(',');
        }
        Serial.print(values[i]);
    }
    Serial.print(']');
}

uint16_t line_peak() {
    uint16_t peak = 0;
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        if (calibrated_values[i] > peak) {
            peak = calibrated_values[i];
        }
    }
    return peak;
}

void print_surfaces() {
    Serial.print("surface=[");
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        if (i > 0) {
            Serial.print(',');
        }
        Serial.print(calibrated_values[i] >= 500 ? 'B' : 'W');
    }
    Serial.print(']');
}

void print_pin_map() {
    Serial.println("QTRSensors analog library test");
    Serial.println("Pins: S1=A0 S2=A1 S3=A2 S4=A3 S5=A4 S6=A5 S7=A6 S8=A7 S9=DAC0");
    Serial.println("Do not use A8 here; GIGA A8 is special and cannot go in QTRSensors' uint8_t pin array.");
    Serial.print("Emitter: ");
    if (robot_config::QTR_EMITTER_PIN == 255) {
        Serial.println("disabled");
    } else {
        Serial.println(robot_config::QTR_EMITTER_PIN);
    }
}

}  // namespace

void qtr_lib_test_app_setup() {
    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    qtr.setTypeAnalog();
    qtr.setSensorPins(sensor_pins, robot_config::QTR_SENSOR_COUNT);
    qtr.setSamplesPerSensor(robot_config::QTR_RAW_SAMPLE_COUNT);
    if (robot_config::QTR_EMITTER_PIN != 255) {
        qtr.setEmitterPin(robot_config::QTR_EMITTER_PIN);
    }

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.println();
    print_pin_map();
    Serial.println("Move the array across both floor and black line for 5 seconds.");

    const unsigned long calibration_start_ms = millis();
    while (millis() - calibration_start_ms < CALIBRATION_TIME_MS) {
        qtr.calibrate();
        delay(20);
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Calibration complete.");
    print_values("min", qtr.calibrationOn.minimum);
    Serial.print(' ');
    print_values("max", qtr.calibrationOn.maximum);
    Serial.println();
    Serial.println("Columns: raw cal surface peak found line");
    last_status_ms = millis();
}

void qtr_lib_test_app_loop() {
    const unsigned long now_ms = millis();
    if (now_ms - last_status_ms < STATUS_INTERVAL_MS) {
        return;
    }
    last_status_ms = now_ms;

    qtr.read(raw_values);
    const uint16_t position = qtr.readLineBlack(calibrated_values);
    const uint16_t peak = line_peak();
    const bool found = peak >= LINE_DETECT_THRESHOLD;

    digitalWrite(LED_BUILTIN, found ? HIGH : LOW);
    print_values("raw", raw_values);
    Serial.print(' ');
    print_values("cal", calibrated_values);
    Serial.print(' ');
    print_surfaces();
    Serial.print(" peak=");
    Serial.print(peak);
    Serial.print(" found=");
    Serial.print(found ? 1 : 0);
    Serial.print(" line=");
    Serial.println(found ? static_cast<int32_t>(position) : -1);
}
