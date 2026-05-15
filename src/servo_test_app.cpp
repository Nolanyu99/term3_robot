#include <Arduino.h>

namespace {

constexpr int SERVO1_PIN = 44;  // upper gate: isolates one seed
constexpr int SERVO2_PIN = 45;  // lower gate: releases into tube

constexpr int UPPER_CLOSED = 90;
constexpr int UPPER_OPEN = 0;
constexpr int LOWER_CLOSED = 0;
constexpr int LOWER_OPEN = 90;

constexpr int SERVO_MIN_PULSE_US = 500;
constexpr int SERVO_MAX_PULSE_US = 2500;
constexpr unsigned long SERVO_PERIOD_US = 20000;

int upper_pulse_us = 1500;
int lower_pulse_us = 1500;
unsigned long last_servo_frame_us = 0;
unsigned long last_status_ms = 0;

int angleToPulseUs(int angle) {
    angle = constrain(angle, 0, 180);
    return map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
}

void serviceServoPulses() {
    const unsigned long now = micros();
    if (now - last_servo_frame_us < SERVO_PERIOD_US) {
        return;
    }
    last_servo_frame_us = now;

    digitalWrite(SERVO1_PIN, HIGH);
    delayMicroseconds(upper_pulse_us);
    digitalWrite(SERVO1_PIN, LOW);

    digitalWrite(SERVO2_PIN, HIGH);
    delayMicroseconds(lower_pulse_us);
    digitalWrite(SERVO2_PIN, LOW);
}

void waitWithServo(unsigned long ms) {
    const unsigned long start = millis();
    while (millis() - start < ms) {
        serviceServoPulses();
        delay(1);
    }
}

void writeUpperServo(int angle) {
    upper_pulse_us = angleToPulseUs(angle);
}

void writeLowerServo(int angle) {
    lower_pulse_us = angleToPulseUs(angle);
}

void closeBothGates() {
    writeUpperServo(UPPER_CLOSED);
    writeLowerServo(LOWER_CLOSED);
}

void dispenseOne() {
    Serial.println(F("Dispense cycle..."));

    writeUpperServo(UPPER_OPEN);
    waitWithServo(700);

    writeUpperServo(UPPER_CLOSED);
    waitWithServo(1000);

    writeLowerServo(LOWER_OPEN);
    waitWithServo(700);

    writeLowerServo(LOWER_CLOSED);
    waitWithServo(500);

    Serial.println(F("Done."));
}

}  // namespace

void servo_test_app_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
    }

    pinMode(SERVO1_PIN, OUTPUT);
    pinMode(SERVO2_PIN, OUTPUT);
    digitalWrite(SERVO1_PIN, LOW);
    digitalWrite(SERVO2_PIN, LOW);

    closeBothGates();
    waitWithServo(500);

    Serial.println(F("SG90 Seed Dispenser test"));
    Serial.println(F("Commands:"));
    Serial.println(F("  d           run a full dispense cycle"));
    Serial.println(F("  c           close both gates"));
}

void servo_test_app_loop() {
    serviceServoPulses();

    if (millis() - last_status_ms >= 2000) {
        last_status_ms = millis();
        Serial.println(F("Servo test ready. Send d=dispense, c=close."));
    }

    if (!Serial.available()) {
        return;
    }

    const char cmd = Serial.read();

    if (cmd == 'd') {
        dispenseOne();
    } else if (cmd == 'c') {
        closeBothGates();
        Serial.println(F("Both gates closed."));
    }
}
