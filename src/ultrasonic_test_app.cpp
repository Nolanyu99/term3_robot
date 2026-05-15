#include <Arduino.h>

#include "robot_config.hpp"

namespace {

constexpr unsigned long ECHO_TIMEOUT_US =
    static_cast<unsigned long>(robot_config::ULTRASONIC_MAX_DISTANCE_CM * 2.0f * 29.1f);

float read_distance_cm() {
    digitalWrite(robot_config::ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(robot_config::ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(robot_config::ULTRASONIC_TRIG_PIN, LOW);

    const unsigned long duration_us =
        pulseIn(robot_config::ULTRASONIC_ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
    if (duration_us == 0) {
        return -1.0f;
    }

    return (duration_us * 0.0343f) / 2.0f;
}

}  // namespace

void ultrasonic_test_app_setup() {
    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < 3000) {
    }

    pinMode(robot_config::ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(robot_config::ULTRASONIC_ECHO_PIN, INPUT);
    digitalWrite(robot_config::ULTRASONIC_TRIG_PIN, LOW);

    Serial.println("=== HC-SR04 distance test ===");
    Serial.print("Trig pin: D");
    Serial.println(robot_config::ULTRASONIC_TRIG_PIN);
    Serial.print("Echo pin: D");
    Serial.println(robot_config::ULTRASONIC_ECHO_PIN);
    Serial.println("Use a resistor divider on Echo before connecting to Arduino GIGA.");
    delay(100);
}

void ultrasonic_test_app_loop() {
    const float distance_cm = read_distance_cm();

    if (distance_cm < 0.0f) {
        Serial.println("Out of range");
    } else {
        Serial.print("Distance: ");
        Serial.print(distance_cm, 1);
        Serial.println(" cm");

        if (distance_cm < robot_config::ULTRASONIC_WALL_THRESHOLD_CM) {
            Serial.println("Object: Wall");
        }
    }

    delay(100);
}
