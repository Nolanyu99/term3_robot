#include <Arduino.h>

namespace {

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr uint8_t SENSOR_COUNT = 9;
const uint8_t sensor_pins[SENSOR_COUNT] = {45, 46, 47, 48, 49, 50, 51, 52, 53};
constexpr uint16_t TIMEOUT_US = 1000;

uint16_t sensor_values[SENSOR_COUNT] = {};

void print_pin_header() {
    Serial.println("SYSTEM BOOT: Library-bypass QTR-RC diagnostic started");
    Serial.println("Connect QTR outputs to D45-D53. Wave white paper / black line under the array.");
    Serial.println("P45\tP46\tP47\tP48\tP49\tP50\tP51\tP52\tP53");
    Serial.println("-------------------------------------------------------------------------");
}

void read_rc_discharge_times() {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        pinMode(sensor_pins[i], OUTPUT);
        digitalWrite(sensor_pins[i], HIGH);
    }

    delayMicroseconds(15);

    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        pinMode(sensor_pins[i], INPUT);
        sensor_values[i] = TIMEOUT_US;
    }

    const unsigned long start_time_us = micros();
    while (micros() - start_time_us < TIMEOUT_US) {
        const uint16_t elapsed_us = static_cast<uint16_t>(micros() - start_time_us);
        for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
            if (sensor_values[i] == TIMEOUT_US && digitalRead(sensor_pins[i]) == LOW) {
                sensor_values[i] = elapsed_us;
            }
        }
    }
}

void print_values() {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        Serial.print(sensor_values[i]);
        Serial.print('\t');
    }
    Serial.println();
}

}  // namespace

void ir_test_app_setup() {
    delay(5000);

    Serial.begin(9600);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    print_pin_header();
}

void ir_test_app_loop() {
    read_rc_discharge_times();
    print_values();
    delay(250);
}
