#include <Arduino.h>

#include "encoder_motor.hpp"
#include "robot_config.hpp"

namespace {

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long STATUS_INTERVAL_MS = 250;

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
    left_encoder.update_velocity(dt_s);
    right_encoder.update_velocity(dt_s);

    Serial.print("left count=");
    Serial.print(left_encoder.count());
    Serial.print(" vel=");
    Serial.print(left_encoder.velocity_rad_s(), 3);
    Serial.print(" | right count=");
    Serial.print(right_encoder.count());
    Serial.print(" vel=");
    Serial.println(right_encoder.velocity_rad_s(), 3);
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
    print_pin_map();

    left_encoder.begin();
    right_encoder.begin();
    left_encoder.stop();
    right_encoder.stop();
    last_status_ms = millis();
}

void encoder_test_app_loop() {
    const unsigned long now_ms = millis();
    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        const float dt_s = (now_ms - last_status_ms) / 1000.0f;
        last_status_ms = now_ms;
        print_status(dt_s);
    }
}
