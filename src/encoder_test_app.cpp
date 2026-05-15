#include <Arduino.h>

#include "encoder_motor.hpp"
#include "robot_config.hpp"

namespace {

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long STATUS_INTERVAL_MS = 250;
constexpr bool TEST_LEFT_ENCODER = true;
constexpr bool TEST_RIGHT_ENCODER = true;
constexpr bool ENABLE_ENCODER_INTERRUPTS = false;

void print_interrupt_pin(const char* label, uint8_t pin) {
    Serial.print(label);
    Serial.print(" D");
    Serial.print(pin);
    Serial.print(" interrupt=");
    const int interrupt_number = digitalPinToInterrupt(pin);
    if (interrupt_number == NOT_AN_INTERRUPT) {
        Serial.println("not supported");
    } else {
        Serial.println(interrupt_number);
    }
}

EncoderMotor left_encoder(
    robot_config::LEFT_ENC_A,
    robot_config::LEFT_ENC_B,
    robot_config::LEFT_PWM,
    robot_config::LEFT_IN_A,
    robot_config::LEFT_IN_B,
    robot_config::LEFT_ENABLE,
    robot_config::MOTOR_GEAR_RATIO,
    robot_config::MOTOR_RAW_CPR);

EncoderMotor right_encoder(
    robot_config::RIGHT_ENC_A,
    robot_config::RIGHT_ENC_B,
    robot_config::RIGHT_PWM,
    robot_config::RIGHT_IN_A,
    robot_config::RIGHT_IN_B,
    robot_config::RIGHT_ENABLE,
    robot_config::MOTOR_GEAR_RATIO,
    robot_config::MOTOR_RAW_CPR);

unsigned long last_status_ms = 0;

void print_pin_map() {
    Serial.println("Encoder pin map:");
    Serial.print("LEFT_ENC_A=D");
    Serial.print(robot_config::LEFT_ENC_A);
    Serial.print(" LEFT_ENC_B=D");
    Serial.println(robot_config::LEFT_ENC_B);
    Serial.print("RIGHT_ENC_A=D");
    Serial.print(robot_config::RIGHT_ENC_A);
    Serial.print(" RIGHT_ENC_B=D");
    Serial.println(robot_config::RIGHT_ENC_B);
}

void print_status(float dt_s) {
    if (TEST_LEFT_ENCODER) {
        left_encoder.update_velocity(dt_s);
    }
    if (TEST_RIGHT_ENCODER) {
        right_encoder.update_velocity(dt_s);
    }

    if (TEST_LEFT_ENCODER) {
        Serial.print("left count=");
        Serial.print(left_encoder.count());
        Serial.print(" vel=");
        Serial.print(left_encoder.velocity_rad_s(), 3);
        Serial.print(" raw A=");
        Serial.print(digitalRead(robot_config::LEFT_ENC_A));
        Serial.print(" B=");
        Serial.print(digitalRead(robot_config::LEFT_ENC_B));
    } else {
        Serial.print("left disabled");
    }
    Serial.print(" | ");
    if (TEST_RIGHT_ENCODER) {
        Serial.print("right count=");
        Serial.print(right_encoder.count());
        Serial.print(" vel=");
        Serial.print(right_encoder.velocity_rad_s(), 3);
        Serial.print(" raw A=");
        Serial.print(digitalRead(robot_config::RIGHT_ENC_A));
        Serial.print(" B=");
        Serial.println(digitalRead(robot_config::RIGHT_ENC_B));
    } else {
        Serial.println("right disabled");
    }
}

}  // namespace

void encoder_test_app_setup() {
    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== Encoder test ===");
    Serial.println("USB power only is OK. Turn the wheels by hand.");
    Serial.print("Left encoder: ");
    Serial.println(TEST_LEFT_ENCODER ? "enabled" : "disabled");
    Serial.print("Right encoder: ");
    Serial.println(TEST_RIGHT_ENCODER ? "enabled" : "disabled");
    Serial.print("Encoder interrupts: ");
    Serial.println(ENABLE_ENCODER_INTERRUPTS ? "enabled" : "disabled raw-only");
    print_pin_map();
    print_interrupt_pin("LEFT_ENC_A", robot_config::LEFT_ENC_A);
    print_interrupt_pin("RIGHT_ENC_A", robot_config::RIGHT_ENC_A);

    if (TEST_LEFT_ENCODER) {
        Serial.println("Starting left encoder input...");
        if (ENABLE_ENCODER_INTERRUPTS) {
            left_encoder.begin_encoder_only();
        } else {
            left_encoder.begin_encoder_polling_only();
        }
        Serial.println("Left encoder input started.");
    }
    if (TEST_RIGHT_ENCODER) {
        Serial.println("Starting right encoder input...");
        if (ENABLE_ENCODER_INTERRUPTS) {
            right_encoder.begin_encoder_only();
        } else {
            right_encoder.begin_encoder_polling_only();
        }
        Serial.println("Right encoder input started.");
    }
    last_status_ms = millis();
}

void encoder_test_app_loop() {
    const unsigned long now_ms = millis();
    if (!ENABLE_ENCODER_INTERRUPTS) {
        if (TEST_LEFT_ENCODER) {
            left_encoder.poll_encoder();
        }
        if (TEST_RIGHT_ENCODER) {
            right_encoder.poll_encoder();
        }
    }

    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        const float dt_s = (now_ms - last_status_ms) / 1000.0f;
        last_status_ms = now_ms;
        print_status(dt_s);
    }
}
