#include <Arduino.h>
#include <Motoron.h>
#include <Wire.h>

#include "robot_config.hpp"

namespace {

constexpr int16_t TEST_SPEED = 400;
constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long STATUS_INTERVAL_MS = 500;
constexpr uint8_t LEFT_MOTOR = 1;
constexpr uint8_t RIGHT_MOTOR = 2;

struct TestPhase {
    const char* name;
    int16_t left_speed;
    int16_t right_speed;
    unsigned long duration_ms;
};

const TestPhase phases[] = {
    {"both stopped", 0, 0, 2000},
    {"left forward", TEST_SPEED, 0, 999999},
    {"both stopped", 0, 0, 2000},
    {"left reverse", -TEST_SPEED, 0, 0},
    {"both stopped", 0, 0, 1000},
    {"right forward", 0, TEST_SPEED, 0},
    {"both stopped", 0, 0, 1000},
    {"right reverse", 0, -TEST_SPEED, 0},
    {"both stopped", 0, 0, 1000},
};

constexpr size_t PHASE_COUNT = sizeof(phases) / sizeof(phases[0]);

size_t phase_index = 0;
unsigned long phase_start_ms = 0;
unsigned long last_status_ms = 0;

MotoronI2C motoron;
TwoWire& motoron_bus = Wire1;

uint8_t scan_i2c_address() {
    Serial.println("Scanning I2C bus...");
    uint8_t first_address = 0;

    for (uint8_t address = 1; address < 127; ++address) {
        motoron_bus.beginTransmission(address);
        const uint8_t error = motoron_bus.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at 0x");
            if (address < 16) {
                Serial.print('0');
            }
            Serial.println(address, HEX);
            if (first_address == 0) {
                first_address = address;
            }
        }
    }

    if (first_address == 0) {
        Serial.println("No I2C devices found.");
    }

    return first_address;
}

void stop_motors() {
    motoron.setSpeed(LEFT_MOTOR, 0);
    motoron.setSpeed(RIGHT_MOTOR, 0);
    motoron.setSpeed(3, 0);
}

void begin_motoron() {
    motoron_bus.begin();
    motoron_bus.setClock(100000);
    motoron.setBus(&motoron_bus);

    const uint8_t detected_address = scan_i2c_address();
    if (detected_address != 0 && detected_address != motoron.getAddress()) {
        Serial.print("Using detected Motoron address 0x");
        if (detected_address < 16) {
            Serial.print('0');
        }
        Serial.println(detected_address, HEX);
        motoron.setAddress(detected_address);
    }

    motoron.reinitialize();
    delay(10);
    motoron.disableCrc();
    motoron.clearResetFlag();
    motoron.disableCommandTimeout();
    motoron.clearMotorFaultUnconditional();
    stop_motors();

    Serial.print("Motoron I2C last error=");
    Serial.println(motoron.getLastError());
}

void apply_phase() {
    const TestPhase& phase = phases[phase_index];
    motoron.setSpeed(LEFT_MOTOR, phase.left_speed);
    motoron.setSpeed(RIGHT_MOTOR, phase.right_speed);
    motoron.setSpeed(3, 0);

    Serial.println();
    Serial.print("Phase ");
    Serial.print(phase_index + 1);
    Serial.print('/');
    Serial.print(PHASE_COUNT);
    Serial.print(": ");
    Serial.println(phase.name);
}

void print_status() {
    const TestPhase& phase = phases[phase_index];
    Serial.print("running: ");
    Serial.print(phase.name);
    Serial.print(" m1_speed=");
    Serial.print(phase.left_speed);
    Serial.print(" m2_speed=");
    Serial.print(phase.right_speed);
    Serial.print(" motoron_error=");
    Serial.println(motoron.getLastError());
}

void print_pin_map() {
    Serial.println("Motoron M3S550 uses I2C, not D2/D4/D9 PWM pins.");
    Serial.println("M1A/M1B = motor 1, M2A/M2B = motor 2, M3A/M3B = motor 3.");
    Serial.println("Using Wire1 for the Arduino shield SDA/SCL pins.");
    Serial.println("On Arduino shield headers, make sure SDA/SCL, IOREF, 3.3V/5V, and GND are connected.");
}

}  // namespace

void motor_test_app_setup() {
    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== Motoron M3S550 motor test ===");
    Serial.println("Lift the wheels before running this test.");
    Serial.print("Test speed: ");
    Serial.println(TEST_SPEED);
    Serial.println("Encoder inputs are disabled in this Motoron output test.");
    print_pin_map();

    begin_motoron();

    phase_start_ms = millis();
    last_status_ms = millis();
    apply_phase();
}

void motor_test_app_loop() {
    const unsigned long now_ms = millis();

    if (now_ms - phase_start_ms >= phases[phase_index].duration_ms) {
        ++phase_index;
        phase_start_ms = now_ms;

        if (phase_index >= PHASE_COUNT) {
            stop_motors();
            Serial.println();
            Serial.println("Motor test cycle complete. Restarting.");
            phase_index = 0;
        }

        apply_phase();
    }

    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;
        print_status();
    }
}
