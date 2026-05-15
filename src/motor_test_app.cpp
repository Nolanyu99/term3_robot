#include <Arduino.h>
#include <Motoron.h>
#include <Wire.h>

#include "encoder_motor.hpp"
#include "robot_config.hpp"

namespace {

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long CONTROL_INTERVAL_MS = 50;
constexpr unsigned long STATUS_INTERVAL_MS = 500;
constexpr unsigned long STOP_DURATION_MS = 800;
constexpr unsigned long STRAIGHT_DURATION_MS = 2000;
constexpr uint16_t MOTORON_ACCELERATION = 800;
constexpr uint16_t MOTORON_DECELERATION = 800;
constexpr uint16_t MOTORON_LOGIC_REFERENCE_MV = 3300;
constexpr uint8_t LEFT_MOTOR = 1;
constexpr uint8_t RIGHT_MOTOR = 2;
constexpr bool ENABLE_ENCODER_READBACK = true;
constexpr bool ENABLE_ENCODER_INTERRUPTS = false;
constexpr bool ENABLE_SPEED_CLOSED_LOOP = true;
constexpr float SPEED_PID_KP = 130.0f;
constexpr float SPEED_PID_KI = 45.0f;
constexpr float SPEED_PID_KD = 0.0f;
constexpr float SPEED_PID_INTEGRAL_LIMIT = 3.0f;
constexpr float STRAIGHT_SYNC_DELTA_KP = 3.0f;
constexpr float STRAIGHT_SYNC_TOTAL_KP = 0.15f;
constexpr int16_t STRAIGHT_SYNC_LIMIT = 90;
constexpr int16_t MOTORON_MAX_COMMAND = 800;
constexpr int16_t MIN_MOVING_COMMAND = 180;
constexpr uint8_t LEFT_COMMAND_TRIM_PERCENT = 100;
constexpr uint8_t RIGHT_COMMAND_TRIM_PERCENT = 30;
constexpr int8_t LEFT_ENCODER_DIRECTION = 1;
constexpr int8_t RIGHT_ENCODER_DIRECTION = 1;
constexpr float PI_F = 3.14159265f;
constexpr float WHEEL_DIAMETER_M = 0.080f;
constexpr float WHEEL_TRACK_M = 0.160f;
constexpr float TURN_CALIBRATION = 1.0f;
constexpr float TICKS_PER_WHEEL_REV = robot_config::MOTOR_GEAR_RATIO * robot_config::MOTOR_RAW_CPR;
constexpr int16_t SLOW_COMMAND = 300;
constexpr int16_t MEDIUM_COMMAND = 430;
constexpr int16_t FAST_COMMAND = 560;
constexpr int16_t TURN_COMMAND = 420;
constexpr float SLOW_TARGET_RAD_S = 0.45f;
constexpr float MEDIUM_TARGET_RAD_S = 0.70f;
constexpr float FAST_TARGET_RAD_S = 0.95f;
constexpr float TURN_TARGET_RAD_S = 0.55f;

enum class MotionKind {
    Stop,
    DriveTimed,
    TurnToTicks,
};

struct DemoStep {
    const char* name;
    MotionKind kind;
    int16_t left_command;
    int16_t right_command;
    float left_target_rad_s;
    float right_target_rad_s;
    unsigned long duration_ms;
    long target_ticks;
};

constexpr long turn_target_ticks(float turn_angle_rad) {
    return static_cast<long>(
        ((turn_angle_rad * WHEEL_TRACK_M * 0.5f) / (PI_F * WHEEL_DIAMETER_M)) *
        TICKS_PER_WHEEL_REV * TURN_CALIBRATION + 0.5f);
}

const DemoStep demo_steps[] = {
    {"stop", MotionKind::Stop, 0, 0, 0.0f, 0.0f, STOP_DURATION_MS, 0},
    {"forward slow", MotionKind::DriveTimed, SLOW_COMMAND, SLOW_COMMAND,
        SLOW_TARGET_RAD_S, SLOW_TARGET_RAD_S, STRAIGHT_DURATION_MS, 0},
    {"stop", MotionKind::Stop, 0, 0, 0.0f, 0.0f, STOP_DURATION_MS, 0},
    {"forward medium", MotionKind::DriveTimed, MEDIUM_COMMAND, MEDIUM_COMMAND,
        MEDIUM_TARGET_RAD_S, MEDIUM_TARGET_RAD_S, STRAIGHT_DURATION_MS, 0},
    {"stop", MotionKind::Stop, 0, 0, 0.0f, 0.0f, STOP_DURATION_MS, 0},
    {"forward fast", MotionKind::DriveTimed, FAST_COMMAND, FAST_COMMAND,
        FAST_TARGET_RAD_S, FAST_TARGET_RAD_S, STRAIGHT_DURATION_MS, 0},
    {"stop", MotionKind::Stop, 0, 0, 0.0f, 0.0f, STOP_DURATION_MS, 0},
    {"left 90", MotionKind::TurnToTicks, -TURN_COMMAND, TURN_COMMAND,
        -TURN_TARGET_RAD_S, TURN_TARGET_RAD_S, 0, turn_target_ticks(PI_F / 2.0f)},
    {"stop", MotionKind::Stop, 0, 0, 0.0f, 0.0f, STOP_DURATION_MS, 0},
    {"right 90", MotionKind::TurnToTicks, TURN_COMMAND, -TURN_COMMAND,
        TURN_TARGET_RAD_S, -TURN_TARGET_RAD_S, 0, turn_target_ticks(PI_F / 2.0f)},
    {"stop", MotionKind::Stop, 0, 0, 0.0f, 0.0f, STOP_DURATION_MS, 0},
    {"u-turn", MotionKind::TurnToTicks, -TURN_COMMAND, TURN_COMMAND,
        -TURN_TARGET_RAD_S, TURN_TARGET_RAD_S, 0, turn_target_ticks(PI_F)},
    {"final stop", MotionKind::Stop, 0, 0, 0.0f, 0.0f, 1500, 0},
};

constexpr size_t STEP_COUNT = sizeof(demo_steps) / sizeof(demo_steps[0]);

size_t step_index = 0;
unsigned long step_start_ms = 0;
unsigned long last_control_ms = 0;
unsigned long last_status_ms = 0;
long step_start_left_count = 0;
long step_start_right_count = 0;
long previous_sync_left_count = 0;
long previous_sync_right_count = 0;
int16_t current_left_command = 0;
int16_t current_right_command = 0;
int16_t current_sync_adjustment = 0;
int16_t left_feedforward_command = 0;
int16_t right_feedforward_command = 0;
float left_target_rad_s = 0.0f;
float right_target_rad_s = 0.0f;
uint8_t detected_motoron_address = 0;
bool motoron_ready = false;
uint16_t motoron_status_flags = 0;
uint32_t motoron_vin_mv = 0;

MotoronI2C motoron;
TwoWire* motoron_bus = &Wire1;
const char* motoron_bus_name = "Wire1";

PIDController left_speed_pid(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD);
PIDController right_speed_pid(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD);

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

float normalized_left_velocity_rad_s() {
    return LEFT_ENCODER_DIRECTION * left_encoder.velocity_rad_s();
}

float normalized_right_velocity_rad_s() {
    return RIGHT_ENCODER_DIRECTION * right_encoder.velocity_rad_s();
}

long normalized_left_count() {
    return LEFT_ENCODER_DIRECTION * left_encoder.count();
}

long normalized_right_count() {
    return RIGHT_ENCODER_DIRECTION * right_encoder.count();
}

long turn_progress_ticks() {
    const long left_delta = normalized_left_count() - step_start_left_count;
    const long right_delta = normalized_right_count() - step_start_right_count;
    return (abs(left_delta) + abs(right_delta)) / 2;
}

int16_t scale_command(int16_t command, uint8_t trim_percent) {
    int16_t scaled = static_cast<int16_t>((static_cast<int32_t>(command) * trim_percent) / 100);
    if (scaled > 0 && scaled < MIN_MOVING_COMMAND) {
        scaled = MIN_MOVING_COMMAND;
    } else if (scaled < 0 && scaled > -MIN_MOVING_COMMAND) {
        scaled = -MIN_MOVING_COMMAND;
    }
    return constrain(scaled, -MOTORON_MAX_COMMAND, MOTORON_MAX_COMMAND);
}

uint8_t scan_i2c_address(TwoWire& bus, const char* bus_name) {
    Serial.print("Scanning I2C bus ");
    Serial.print(bus_name);
    Serial.println("...");
    uint8_t first_address = 0;

    for (uint8_t address = 1; address < 127; ++address) {
        bus.beginTransmission(address);
        const uint8_t error = bus.endTransmission();
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

bool select_motoron_bus() {
    motoron_bus->begin();
    motoron_bus->setClock(100000);
    motoron.setBus(motoron_bus);
    detected_motoron_address = scan_i2c_address(*motoron_bus, motoron_bus_name);
    return detected_motoron_address != 0;
}

void stop_motors() {
    if (!motoron_ready) {
        return;
    }

    motoron.setSpeedNow(LEFT_MOTOR, 0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    motoron.setSpeedNow(3, 0);
}

void reset_speed_controllers() {
    left_speed_pid.set_integral_limit(SPEED_PID_INTEGRAL_LIMIT);
    right_speed_pid.set_integral_limit(SPEED_PID_INTEGRAL_LIMIT);
    left_speed_pid.set_target(left_target_rad_s);
    right_speed_pid.set_target(right_target_rad_s);
    left_speed_pid.reset();
    right_speed_pid.reset();
}

void send_motor_commands(int16_t left_command, int16_t right_command) {
    current_left_command = scale_command(left_command, LEFT_COMMAND_TRIM_PERCENT);
    current_right_command = scale_command(right_command, RIGHT_COMMAND_TRIM_PERCENT);

    if (!motoron_ready) {
        current_left_command = 0;
        current_right_command = 0;
        return;
    }

    motoron.setSpeedNow(LEFT_MOTOR, current_left_command);
    motoron.setSpeedNow(RIGHT_MOTOR, current_right_command);
    motoron.setSpeedNow(3, 0);
    if (motoron.getLastError() != 0) {
        motoron_ready = false;
        current_left_command = 0;
        current_right_command = 0;
    }
}

void print_hex16(uint16_t value) {
    if (value < 0x1000) {
        Serial.print('0');
    }
    if (value < 0x0100) {
        Serial.print('0');
    }
    if (value < 0x0010) {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

void print_motoron_alert_bits(uint16_t flags) {
    bool printed = false;
    auto print_alert = [&printed](const char* text) {
        if (printed) {
            Serial.print(',');
        }
        Serial.print(text);
        printed = true;
    };

    if (flags & (1 << MOTORON_STATUS_FLAG_RESET)) {
        print_alert("RESET");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_NO_POWER)) {
        print_alert("NO_POWER");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_NO_POWER_LATCHED)) {
        print_alert("NO_POWER_LATCH");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_MOTOR_FAULTING)) {
        print_alert("FAULT");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_MOTOR_FAULT_LATCHED)) {
        print_alert("FAULT_LATCH");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT)) {
        print_alert("TIMEOUT");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT_LATCHED)) {
        print_alert("TIMEOUT_LATCH");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_PROTOCOL_ERROR)) {
        print_alert("PROTO");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_CRC_ERROR)) {
        print_alert("CRC");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_ERROR_ACTIVE)) {
        print_alert("ERROR");
    }

    if (!printed) {
        Serial.print("ok");
    }
}

bool refresh_motoron_status() {
    if (detected_motoron_address == 0) {
        motoron_ready = false;
        return false;
    }

    motoron_status_flags = motoron.getStatusFlags();
    if (motoron.getLastError() != 0) {
        motoron_ready = false;
        return false;
    }

    motoron_vin_mv = motoron.getVinVoltageMv(
        MOTORON_LOGIC_REFERENCE_MV,
        MotoronVinSenseType::Motoron550);
    if (motoron.getLastError() != 0) {
        motoron_ready = false;
        return false;
    }

    motoron_ready = true;
    return true;
}

void configure_motor(uint8_t motor) {
    motoron.setMaxAcceleration(motor, MOTORON_ACCELERATION);
    motoron.setMaxDeceleration(motor, MOTORON_DECELERATION);
}

void configure_motoron() {
    motoron.clearResetFlag();
    motoron.disableCommandTimeout();
    configure_motor(LEFT_MOTOR);
    configure_motor(RIGHT_MOTOR);
    configure_motor(3);
    motoron.clearMotorFaultUnconditional();
}

void begin_motoron() {
    if (!select_motoron_bus()) {
        motoron_ready = false;
        Serial.println("Motoron not ready: no I2C device found.");
        return;
    }

    if (detected_motoron_address != motoron.getAddress()) {
        Serial.print("Using detected Motoron address 0x");
        if (detected_motoron_address < 16) {
            Serial.print('0');
        }
        Serial.println(detected_motoron_address, HEX);
        motoron.setAddress(detected_motoron_address);
    }

    motoron.reinitialize();
    delay(10);
    motoron.disableCrc();
    configure_motoron();
    Serial.print("Motoron I2C last error=");
    Serial.println(motoron.getLastError());
    if (motoron.getLastError() != 0) {
        motoron_ready = false;
        Serial.println("Motoron not ready: initialization command failed.");
        return;
    }

    uint16_t product_id = 0;
    uint16_t firmware_version = 0;
    motoron.getFirmwareVersion(&product_id, &firmware_version);
    if (motoron.getLastError() != 0) {
        motoron_ready = false;
        Serial.println("Motoron not ready: firmware read failed.");
        return;
    }

    Serial.print("Motoron product=0x");
    print_hex16(product_id);
    Serial.print(" firmware=0x");
    print_hex16(firmware_version);
    Serial.println();
    Serial.print("Motoron accel/decel=");
    Serial.print(MOTORON_ACCELERATION);
    Serial.print('/');
    Serial.println(MOTORON_DECELERATION);

    refresh_motoron_status();
    stop_motors();
}

void apply_step() {
    const DemoStep& step = demo_steps[step_index];
    step_start_left_count = normalized_left_count();
    step_start_right_count = normalized_right_count();
    previous_sync_left_count = step_start_left_count;
    previous_sync_right_count = step_start_right_count;
    current_sync_adjustment = 0;
    left_feedforward_command = step.left_command;
    right_feedforward_command = step.right_command;
    left_target_rad_s = step.left_target_rad_s;
    right_target_rad_s = step.right_target_rad_s;
    reset_speed_controllers();
    send_motor_commands(left_feedforward_command, right_feedforward_command);

    Serial.println();
    Serial.print("step ");
    Serial.print(step_index + 1);
    Serial.print('/');
    Serial.print(STEP_COUNT);
    Serial.print(' ');
    Serial.print(step.name);
    Serial.print(" cmd=");
    Serial.print(current_left_command);
    Serial.print(',');
    Serial.print(current_right_command);
    if (step.kind == MotionKind::TurnToTicks) {
        Serial.print(" target_ticks=");
        Serial.print(step.target_ticks);
    }
    if (ENABLE_SPEED_CLOSED_LOOP) {
        Serial.print(" target=");
        Serial.print(left_target_rad_s, 3);
        Serial.print(',');
        Serial.print(right_target_rad_s, 3);
    }
    Serial.println();
    if (!motoron_ready) {
        Serial.println("Motoron not ready; speed commands skipped.");
    }
}

bool current_step_complete(unsigned long now_ms) {
    const DemoStep& step = demo_steps[step_index];
    if (step.kind == MotionKind::TurnToTicks) {
        return turn_progress_ticks() >= step.target_ticks;
    }
    return now_ms - step_start_ms >= step.duration_ms;
}

void update_speed_control(float dt_s) {
    if (ENABLE_ENCODER_READBACK) {
        left_encoder.update_velocity(dt_s);
        right_encoder.update_velocity(dt_s);
    }

    if (!ENABLE_SPEED_CLOSED_LOOP) {
        return;
    }

    if (left_target_rad_s == 0.0f && right_target_rad_s == 0.0f) {
        send_motor_commands(0, 0);
        return;
    }

    left_speed_pid.set_target(left_target_rad_s);
    right_speed_pid.set_target(right_target_rad_s);

    current_sync_adjustment = 0;
    const DemoStep& step = demo_steps[step_index];
    if (step.kind == MotionKind::DriveTimed) {
        const long left_count = normalized_left_count();
        const long right_count = normalized_right_count();
        const long left_delta = left_count - previous_sync_left_count;
        const long right_delta = right_count - previous_sync_right_count;
        const long delta_error = left_delta - right_delta;
        const long total_error =
            (left_count - step_start_left_count) - (right_count - step_start_right_count);
        previous_sync_left_count = left_count;
        previous_sync_right_count = right_count;

        const float raw_sync_adjustment =
            STRAIGHT_SYNC_DELTA_KP * delta_error + STRAIGHT_SYNC_TOTAL_KP * total_error;
        current_sync_adjustment = static_cast<int16_t>(constrain(
            raw_sync_adjustment,
            -static_cast<float>(STRAIGHT_SYNC_LIMIT),
            static_cast<float>(STRAIGHT_SYNC_LIMIT)));
    }

    const int16_t left_command = static_cast<int16_t>(
        left_feedforward_command +
        left_speed_pid.update(normalized_left_velocity_rad_s(), dt_s) -
        current_sync_adjustment);
    const int16_t right_command = static_cast<int16_t>(
        right_feedforward_command +
        right_speed_pid.update(normalized_right_velocity_rad_s(), dt_s) +
        current_sync_adjustment);
    send_motor_commands(left_command, right_command);
}

void print_status() {
    const DemoStep& step = demo_steps[step_index];
    if (!motoron_ready && detected_motoron_address != 0) {
        configure_motoron();
    }
    refresh_motoron_status();

    Serial.print("run ");
    Serial.print(step.name);
    Serial.print(" cmd=");
    Serial.print(current_left_command);
    Serial.print(',');
    Serial.print(current_right_command);
    Serial.print(" ready=");
    Serial.print(motoron_ready ? 'Y' : 'N');
    if (ENABLE_ENCODER_READBACK) {
        Serial.print(" cnt=");
        Serial.print(normalized_left_count());
        Serial.print(',');
        Serial.print(normalized_right_count());
        Serial.print(" vel=");
        Serial.print(normalized_left_velocity_rad_s(), 2);
        Serial.print(',');
        Serial.print(normalized_right_velocity_rad_s(), 3);
        if (ENABLE_SPEED_CLOSED_LOOP) {
            if (step.kind == MotionKind::DriveTimed) {
                Serial.print(" sync=");
                Serial.print(current_sync_adjustment);
            }
        }
        if (step.kind == MotionKind::TurnToTicks) {
            Serial.print(" turn=");
            Serial.print(turn_progress_ticks());
            Serial.print('/');
            Serial.print(step.target_ticks);
        }
    } else {
        Serial.print(" enc=off");
    }
    Serial.print(" vin=");
    Serial.print(motoron_vin_mv);
    Serial.print("mV flags=");
    print_motoron_alert_bits(motoron_status_flags);
    Serial.print(" stat=0x");
    print_hex16(motoron_status_flags);
    Serial.print(" err=");
    Serial.println(motoron.getLastError());
}

void print_pin_map() {
    Serial.println("Motoron M3S550 uses I2C, not D2/D4/D9 PWM pins.");
    Serial.println("M1A/M1B = motor 1, M2A/M2B = motor 2, M3A/M3B = motor 3.");
    Serial.println("Using Wire1 for the Motoron I2C connection.");
    Serial.println("On Arduino shield headers, make sure SDA/SCL, IOREF, 3.3V/5V, and GND are connected.");
    Serial.print("Encoder left A/B = D");
    Serial.print(robot_config::LEFT_ENC_A);
    Serial.print("/D");
    Serial.println(robot_config::LEFT_ENC_B);
    Serial.print("Encoder right A/B = D");
    Serial.print(robot_config::RIGHT_ENC_A);
    Serial.print("/D");
    Serial.println(robot_config::RIGHT_ENC_B);
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
    Serial.println("Demo: slow/medium/fast forward, left 90, right 90, U-turn.");
    Serial.print("Wheel diameter m: ");
    Serial.println(WHEEL_DIAMETER_M, 3);
    Serial.print("Wheel track m: ");
    Serial.println(WHEEL_TRACK_M, 3);
    Serial.print("90 deg turn target ticks: ");
    Serial.println(turn_target_ticks(PI_F / 2.0f));
    Serial.print("180 deg turn target ticks: ");
    Serial.println(turn_target_ticks(PI_F));
    Serial.print("Motoron accel/decel: ");
    Serial.print(MOTORON_ACCELERATION);
    Serial.print('/');
    Serial.println(MOTORON_DECELERATION);
    Serial.print("Motoron max/min moving command: ");
    Serial.print(MOTORON_MAX_COMMAND);
    Serial.print('/');
    Serial.println(MIN_MOVING_COMMAND);
    Serial.print("Command trims L/R percent: ");
    Serial.print(LEFT_COMMAND_TRIM_PERCENT);
    Serial.print('/');
    Serial.println(RIGHT_COMMAND_TRIM_PERCENT);
    Serial.print("Encoder readback: ");
    Serial.println(ENABLE_ENCODER_READBACK ? "enabled" : "disabled");
    Serial.print("Speed closed loop: ");
    Serial.println(ENABLE_SPEED_CLOSED_LOOP ? "enabled" : "disabled");
    print_pin_map();

    if (ENABLE_ENCODER_READBACK) {
        if (ENABLE_ENCODER_INTERRUPTS) {
            left_encoder.begin_encoder_only();
            right_encoder.begin_encoder_only();
        } else {
            left_encoder.begin_encoder_polling_only();
            right_encoder.begin_encoder_polling_only();
        }
    }
    begin_motoron();

    step_start_ms = millis();
    last_control_ms = millis();
    last_status_ms = millis();
    apply_step();
}

void motor_test_app_loop() {
    const unsigned long now_ms = millis();
    if (ENABLE_ENCODER_READBACK && !ENABLE_ENCODER_INTERRUPTS) {
        left_encoder.poll_encoder();
        right_encoder.poll_encoder();
    }

    if (current_step_complete(now_ms)) {
        ++step_index;
        step_start_ms = now_ms;

        if (step_index >= STEP_COUNT) {
            stop_motors();
            Serial.println();
            Serial.println("Motor test demo complete. Restarting.");
            step_index = 0;
        }

        apply_step();
    }

    if (now_ms - last_control_ms >= CONTROL_INTERVAL_MS) {
        const float dt_s = (now_ms - last_control_ms) / 1000.0f;
        last_control_ms = now_ms;
        update_speed_control(dt_s);
    }

    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;
        print_status();
    }
}
