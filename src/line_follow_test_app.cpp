#include <Arduino.h>
#include <Motoron.h>
#include <Wire.h>

#include "robot_config.hpp"

namespace {

enum class FollowState {
    Idle,
    FollowLine,
    CrossIntersection,
    LostLine,
};

enum class StartupPhase {
    WaitingBeforeCalibration,
    Calibrating,
    WaitingBeforeRun,
    Ready,
};

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long PRE_CALIBRATION_WAIT_MS = 2000;
constexpr unsigned long CALIBRATION_TIME_MS = 5000;
constexpr unsigned long POST_CALIBRATION_WAIT_MS = 3000;
constexpr unsigned long CONTROL_INTERVAL_MS = 30;
constexpr unsigned long STATUS_INTERVAL_MS = 250;
constexpr unsigned long INTERSECTION_CROSS_MS = 500;
constexpr unsigned long LOST_FORWARD_MS = 250;

constexpr uint8_t SENSOR_COUNT = 9;
constexpr uint16_t LINE_POSITION_SCALE = 1000;
constexpr int32_t LINE_CENTER = ((SENSOR_COUNT - 1) * LINE_POSITION_SCALE) / 2;
constexpr uint16_t INTERSECTION_THRESHOLD = 800;
constexpr uint8_t INTERSECTION_SENSOR_COUNT = 7;

constexpr uint8_t MOTOR_LEFT = 1;
constexpr uint8_t MOTOR_RIGHT = 2;
constexpr uint8_t MOTOR_AUX = 3;
constexpr int16_t BASE_SPEED = 400;
constexpr int16_t MAX_SPEED = 600;
constexpr int16_t LOST_SEARCH_SPEED = 300;
constexpr float LINE_KP = 0.055f;
constexpr int8_t LEFT_FORWARD_SIGN = 1;
constexpr int8_t RIGHT_FORWARD_SIGN = 1;
constexpr uint8_t RIGHT_MOTOR_TRIM_PERCENT = 95;
constexpr uint16_t MOTOR_ACCEL = 800;
constexpr uint16_t MOTOR_DECEL = 800;

const uint8_t sensor_pins[SENSOR_COUNT] = {2, 3, 4, 5, 8, 9, 10, 11, 12};

MotoronI2C motoron;

uint16_t raw_values[SENSOR_COUNT] = {};
uint16_t min_values[SENSOR_COUNT] = {};
uint16_t max_values[SENSOR_COUNT] = {};
uint16_t calibrated_values[SENSOR_COUNT] = {};

FollowState follow_state = FollowState::Idle;
StartupPhase startup_phase = StartupPhase::WaitingBeforeCalibration;
unsigned long startup_phase_start_ms = 0;
unsigned long state_start_ms = 0;
unsigned long last_control_ms = 0;
unsigned long last_status_ms = 0;
int32_t last_error = 0;
int16_t last_left_command = 0;
int16_t last_right_command = 0;
bool line_detected = false;
bool motoron_ready = false;
bool run_enabled = false;

constexpr int16_t trim_right_speed(int16_t speed) {
    return static_cast<int16_t>(
        (static_cast<int32_t>(speed) * RIGHT_MOTOR_TRIM_PERCENT) / 100);
}

const char* state_name() {
    switch (follow_state) {
        case FollowState::Idle:
            return "idle";
        case FollowState::FollowLine:
            return "follow";
        case FollowState::CrossIntersection:
            return "intersection";
        case FollowState::LostLine:
            return "lost";
        default:
            return "unknown";
    }
}

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

uint16_t target_value_at(uint8_t sensor_index) {
    return robot_config::QTR_FOLLOW_BLACK_LINE
               ? calibrated_values[sensor_index]
               : 1000 - calibrated_values[sensor_index];
}

uint16_t line_peak() {
    uint16_t peak = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        const uint16_t value = target_value_at(i);
        if (value > peak) {
            peak = value;
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
        if (target_value_at(i) >= INTERSECTION_THRESHOLD) {
            ++high_count;
        }
    }
    return high_count >= INTERSECTION_SENSOR_COUNT;
}

int32_t estimate_line_position() {
    uint32_t weighted_sum = 0;
    uint32_t sum = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        const uint16_t value = target_value_at(i);
        weighted_sum += static_cast<uint32_t>(value) * i * LINE_POSITION_SCALE;
        sum += value;
    }

    if (sum == 0) {
        return -1;
    }

    return static_cast<int32_t>(weighted_sum / sum);
}

void stop_motors() {
    last_left_command = 0;
    last_right_command = 0;
    if (!motoron_ready) {
        return;
    }
    motoron.setSpeed(MOTOR_LEFT, 0);
    motoron.setSpeed(MOTOR_RIGHT, 0);
    motoron.setSpeed(MOTOR_AUX, 0);
}

void set_motor_speeds(int16_t left_speed, int16_t right_speed) {
    last_left_command = constrain(left_speed, -MAX_SPEED, MAX_SPEED);
    last_right_command = constrain(right_speed, -MAX_SPEED, MAX_SPEED);

    if (!motoron_ready) {
        return;
    }

    motoron.setSpeed(MOTOR_LEFT, last_left_command);
    motoron.setSpeed(MOTOR_RIGHT, last_right_command);
    motoron.setSpeed(MOTOR_AUX, 0);

    if (motoron.getLastError() != 0) {
        motoron_ready = false;
        stop_motors();
        Serial.print("motoron_error=");
        Serial.println(motoron.getLastError());
    }
}

void drive_forward() {
    set_motor_speeds(
        LEFT_FORWARD_SIGN * BASE_SPEED,
        trim_right_speed(RIGHT_FORWARD_SIGN * BASE_SPEED));
}

void search_for_line() {
    if (last_error < 0) {
        set_motor_speeds(
            -LEFT_FORWARD_SIGN * LOST_SEARCH_SPEED,
            trim_right_speed(RIGHT_FORWARD_SIGN * LOST_SEARCH_SPEED));
    } else {
        set_motor_speeds(
            LEFT_FORWARD_SIGN * LOST_SEARCH_SPEED,
            trim_right_speed(-RIGHT_FORWARD_SIGN * LOST_SEARCH_SPEED));
    }
}

void follow_line(int32_t error) {
    const int16_t correction = static_cast<int16_t>(LINE_KP * static_cast<float>(error));
    const int16_t left_speed = LEFT_FORWARD_SIGN * (BASE_SPEED + correction);
    const int16_t right_speed = trim_right_speed(RIGHT_FORWARD_SIGN * (BASE_SPEED - correction));
    set_motor_speeds(left_speed, right_speed);
}

void set_follow_state(FollowState next_state) {
    if (follow_state == next_state) {
        return;
    }
    follow_state = next_state;
    state_start_ms = millis();
}

void configure_motor(uint8_t motor) {
    motoron.setMaxAcceleration(motor, MOTOR_ACCEL);
    motoron.setMaxDeceleration(motor, MOTOR_DECEL);
}

void begin_motoron() {
    Serial.println("Motoron: starting Wire1");
    Wire1.begin();
    Wire1.setClock(100000);

    motoron.setBus(&Wire1);
    motoron.reinitialize();
    delay(10);
    motoron.disableCrc();
    delay(10);
    motoron.clearResetFlag();
    motoron.clearMotorFaultUnconditional();
    motoron.setCommandTimeoutMilliseconds(2000);
    configure_motor(MOTOR_LEFT);
    configure_motor(MOTOR_RIGHT);
    configure_motor(MOTOR_AUX);

    motoron_ready = motoron.getLastError() == 0;
    Serial.print("motoron_ready=");
    Serial.print(motoron_ready ? 1 : 0);
    Serial.print(" error=");
    Serial.println(motoron.getLastError());
    stop_motors();
}

void update_line_following() {
    update_calibrated_values();
    const bool found = update_line_found();
    const bool intersection = found && intersection_detected();
    const int32_t position = found ? estimate_line_position() : -1;
    const int32_t error = position >= 0 ? position - LINE_CENTER : last_error;

    if (!run_enabled) {
        set_follow_state(FollowState::Idle);
        stop_motors();
        return;
    }

    if (!found) {
        set_follow_state(FollowState::LostLine);
        if (millis() - state_start_ms < LOST_FORWARD_MS) {
            drive_forward();
        } else {
            search_for_line();
        }
        return;
    }

    if (follow_state == FollowState::CrossIntersection) {
        if (millis() - state_start_ms < INTERSECTION_CROSS_MS) {
            drive_forward();
            return;
        }
        set_follow_state(FollowState::FollowLine);
    }

    if (intersection) {
        set_follow_state(FollowState::CrossIntersection);
        drive_forward();
        return;
    }

    last_error = error;
    set_follow_state(FollowState::FollowLine);
    follow_line(error);
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

void print_status() {
    const bool found = line_detected;
    const int32_t position = found ? estimate_line_position() : -1;
    const int32_t error = position >= 0 ? position - LINE_CENTER : last_error;

    Serial.print("run=");
    Serial.print(run_enabled ? 1 : 0);
    Serial.print(" state=");
    Serial.print(state_name());
    Serial.print(" motoron=");
    Serial.print(motoron_ready ? 1 : 0);
    Serial.print(" found=");
    Serial.print(found ? 1 : 0);
    Serial.print(" intersection=");
    Serial.print(found && intersection_detected() ? 1 : 0);
    Serial.print(" line=");
    Serial.print(position);
    Serial.print(" error=");
    Serial.print(error);
    Serial.print(" motor=");
    Serial.print(last_left_command);
    Serial.print(',');
    Serial.print(last_right_command);
    Serial.print(' ');
    print_array("cal", calibrated_values);
    Serial.println();
}

void print_calibration() {
    Serial.println("Calibration complete.");
    print_array("min", min_values);
    Serial.print(' ');
    print_array("max", max_values);
    Serial.println();
    Serial.println("Waiting 3 seconds before automatic line following.");
}

void process_serial_commands() {
    while (Serial.available() > 0) {
        const char command = static_cast<char>(Serial.read());
        if (command == 'g' || command == 'G') {
            run_enabled = true;
            set_follow_state(FollowState::FollowLine);
            Serial.println("line_follow=go");
        } else if (command == 'x' || command == 'X') {
            run_enabled = false;
            set_follow_state(FollowState::Idle);
            stop_motors();
            Serial.println("line_follow=stop");
        }
    }
}

}  // namespace

void line_follow_test_app_setup() {
    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== GIGA R1 IR + Simple Motoron line follow test ===");
    Serial.println("Wait 2s, move sensors over floor/line for 5s, then wait 3s for automatic start.");
    Serial.println("Pins: S1=D2 S2=D3 S3=D4 S4=D5 S5=D8 S6=D9 S7=D10 S8=D11 S9=D12");
    Serial.print("base=");
    Serial.print(BASE_SPEED);
    Serial.print(" kp=");
    Serial.println(LINE_KP, 4);

    reset_calibration();
    startup_phase = StartupPhase::WaitingBeforeCalibration;
    startup_phase_start_ms = millis();
    last_control_ms = millis();
    last_status_ms = millis();
    begin_motoron();
    Serial.println("startup=waiting_before_calibration");
}

void line_follow_test_app_loop() {
    process_serial_commands();
    read_rc_discharge_times();

    const unsigned long now_ms = millis();

    if (startup_phase == StartupPhase::WaitingBeforeCalibration) {
        stop_motors();
        if (now_ms - startup_phase_start_ms >= PRE_CALIBRATION_WAIT_MS) {
            startup_phase = StartupPhase::Calibrating;
            startup_phase_start_ms = now_ms;
            Serial.println("startup=calibrating_ir");
        }
        return;
    }

    if (startup_phase == StartupPhase::Calibrating) {
        update_calibration();
        stop_motors();
        if (now_ms - startup_phase_start_ms >= CALIBRATION_TIME_MS) {
            startup_phase = StartupPhase::WaitingBeforeRun;
            startup_phase_start_ms = now_ms;
            print_calibration();
        }
        return;
    }

    if (startup_phase == StartupPhase::WaitingBeforeRun) {
        stop_motors();
        if (now_ms - startup_phase_start_ms >= POST_CALIBRATION_WAIT_MS) {
            startup_phase = StartupPhase::Ready;
            run_enabled = true;
            set_follow_state(FollowState::FollowLine);
            Serial.println("line_follow=auto_start");
        }
        return;
    }

    if (now_ms - last_control_ms >= CONTROL_INTERVAL_MS) {
        last_control_ms = now_ms;
        update_line_following();
    }

    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;
        print_status();
    }
}
