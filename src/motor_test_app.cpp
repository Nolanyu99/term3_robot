#include <Arduino.h>

#include "encoder_motor.hpp"
#include "robot_config.hpp"

namespace {

constexpr int TEST_PWM = 140;
constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long STATUS_INTERVAL_MS = 250;

EncoderMotor left_motor(
    robot_config::LEFT_ENC_A,
    robot_config::LEFT_ENC_B,
    robot_config::LEFT_PWM,
    robot_config::LEFT_IN_A,
    robot_config::LEFT_IN_B,
    robot_config::LEFT_ENABLE,
    robot_config::MOTOR_GEAR_RATIO,
    robot_config::MOTOR_RAW_CPR);

EncoderMotor right_motor(
    robot_config::RIGHT_ENC_A,
    robot_config::RIGHT_ENC_B,
    robot_config::RIGHT_PWM,
    robot_config::RIGHT_IN_A,
    robot_config::RIGHT_IN_B,
    robot_config::RIGHT_ENABLE,
    robot_config::MOTOR_GEAR_RATIO,
    robot_config::MOTOR_RAW_CPR);

struct TestPhase {
    const char* name;
    int left_power;
    int right_power;
    unsigned long duration_ms;
};

const TestPhase phases[] = {
    {"both stopped", 0, 0, 2000},
    {"left forward", TEST_PWM, 0, 2000},
    {"both stopped", 0, 0, 1000},
    {"left reverse", -TEST_PWM, 0, 2000},
    {"both stopped", 0, 0, 1000},
    {"right forward", 0, TEST_PWM, 2000},
    {"both stopped", 0, 0, 1000},
    {"right reverse", 0, -TEST_PWM, 2000},
    {"both stopped", 0, 0, 1000},
};

constexpr size_t PHASE_COUNT = sizeof(phases) / sizeof(phases[0]);

size_t phase_index = 0;
unsigned long phase_start_ms = 0;
unsigned long last_status_ms = 0;
bool test_finished = false;

void write_motor(uint8_t pwm_pin, uint8_t in_a_pin, uint8_t in_b_pin, int power) {
    if (power == 0) {
        analogWrite(pwm_pin, 0);
        digitalWrite(in_a_pin, LOW);
        digitalWrite(in_b_pin, LOW);
        return;
    }

    digitalWrite(in_a_pin, power > 0 ? HIGH : LOW);
    digitalWrite(in_b_pin, power > 0 ? LOW : HIGH);
    analogWrite(pwm_pin, abs(power));
}

void stop_motors() {
    write_motor(robot_config::LEFT_PWM, robot_config::LEFT_IN_A, robot_config::LEFT_IN_B, 0);
    write_motor(robot_config::RIGHT_PWM, robot_config::RIGHT_IN_A, robot_config::RIGHT_IN_B, 0);
}

void apply_phase() {
    const TestPhase& phase = phases[phase_index];
    write_motor(robot_config::LEFT_PWM, robot_config::LEFT_IN_A, robot_config::LEFT_IN_B, phase.left_power);
    write_motor(robot_config::RIGHT_PWM, robot_config::RIGHT_IN_A, robot_config::RIGHT_IN_B, phase.right_power);

    Serial.println();
    Serial.print("Phase ");
    Serial.print(phase_index + 1);
    Serial.print('/');
    Serial.print(PHASE_COUNT);
    Serial.print(": ");
    Serial.println(phase.name);
}

void print_status(float dt_s) {
    left_motor.update_velocity(dt_s);
    right_motor.update_velocity(dt_s);

    Serial.print("left count=");
    Serial.print(left_motor.count());
    Serial.print(" vel=");
    Serial.print(left_motor.velocity_rad_s(), 3);
    Serial.print(" | right count=");
    Serial.print(right_motor.count());
    Serial.print(" vel=");
    Serial.println(right_motor.velocity_rad_s(), 3);
}

void print_pin_map() {
    Serial.println("Pin map:");
    Serial.print("LEFT_ENC_A=D");
    Serial.print(robot_config::LEFT_ENC_A);
    Serial.print(" LEFT_ENC_B=D");
    Serial.print(robot_config::LEFT_ENC_B);
    Serial.print(" LEFT_PWM=D");
    Serial.print(robot_config::LEFT_PWM);
    Serial.print(" LEFT_IN_A=D");
    Serial.print(robot_config::LEFT_IN_A);
    Serial.print(" LEFT_IN_B=D");
    Serial.print(robot_config::LEFT_IN_B);
    Serial.print(" LEFT_ENABLE=D");
    Serial.println(robot_config::LEFT_ENABLE);

    Serial.print("RIGHT_ENC_A=D");
    Serial.print(robot_config::RIGHT_ENC_A);
    Serial.print(" RIGHT_ENC_B=D");
    Serial.print(robot_config::RIGHT_ENC_B);
    Serial.print(" RIGHT_PWM=D");
    Serial.print(robot_config::RIGHT_PWM);
    Serial.print(" RIGHT_IN_A=D");
    Serial.print(robot_config::RIGHT_IN_A);
    Serial.print(" RIGHT_IN_B=D");
    Serial.print(robot_config::RIGHT_IN_B);
    Serial.print(" RIGHT_ENABLE=D");
    Serial.println(robot_config::RIGHT_ENABLE);
}

}  // namespace

void motor_test_app_setup() {
    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== Motor test ===");
    Serial.println("Lift the wheels before running this test.");
    Serial.print("Test PWM: ");
    Serial.println(TEST_PWM);
    print_pin_map();

    left_motor.begin();
    right_motor.begin();
    stop_motors();

    phase_start_ms = millis();
    last_status_ms = millis();
    apply_phase();
}

void motor_test_app_loop() {
    const unsigned long now_ms = millis();

    if (!test_finished && now_ms - phase_start_ms >= phases[phase_index].duration_ms) {
        ++phase_index;
        phase_start_ms = now_ms;

        if (phase_index >= PHASE_COUNT) {
            stop_motors();
            test_finished = true;
            Serial.println();
            Serial.println("Motor test complete. Motors stopped.");
            return;
        }

        apply_phase();
    }

    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        const float dt_s = (now_ms - last_status_ms) / 1000.0f;
        last_status_ms = now_ms;
        print_status(dt_s);
    }
}
