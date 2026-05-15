#include <Arduino.h>
#include <MFRC522_I2C.h>
#include <Motoron.h>
#include <Wire.h>

#include "robot_config.hpp"
#include "serial_logger.hpp"
#include "task_scheduler.hpp"

namespace {

constexpr unsigned long BUTTON_DEBOUNCE_MS = 50;
constexpr unsigned long MOTOR_UPDATE_INTERVAL_MS = 50;
constexpr unsigned long QTR_UPDATE_INTERVAL_MS = 100;
constexpr unsigned long STATUS_INTERVAL_MS = 500;
constexpr unsigned long STOP_LED_BLINK_INTERVAL_MS = 250;
constexpr unsigned long QTR_CALIBRATION_TIME_MS = 5000;

constexpr uint8_t MOTOR_LEFT = 1;
constexpr uint8_t MOTOR_RIGHT = 2;
constexpr uint8_t MOTOR_AUX = 3;
constexpr int16_t LOW_SPEED = 350;
constexpr int16_t MAX_SPEED = 600;
constexpr int16_t TURN_SPEED = 600;
constexpr uint8_t RIGHT_MOTOR_TRIM_PERCENT = 95;
constexpr uint16_t MOTOR_ACCEL = 800;
constexpr uint16_t MOTOR_DECEL = 800;

constexpr unsigned long STOP_DURATION_MS = 700;
constexpr unsigned long FORWARD_DURATION_MS = 2000;
constexpr unsigned long TURN_90_DURATION_MS = 1500;
constexpr unsigned long U_TURN_DURATION_MS = TURN_90_DURATION_MS * 2;

constexpr unsigned long ULTRASONIC_ECHO_TIMEOUT_US =
    static_cast<unsigned long>(robot_config::ULTRASONIC_MAX_DISTANCE_CM * 2.0f * 29.1f);

struct DemoStep {
    const char* name;
    int16_t left_speed;
    int16_t right_speed;
    unsigned long duration_ms;
};

enum class SeedDispenserState {
    Idle,
    UpperOpen,
    UpperClosing,
    LowerOpen,
    LowerClosing,
};

constexpr int16_t trim_right_speed(int16_t speed) {
    return static_cast<int16_t>(
        (static_cast<int32_t>(speed) * RIGHT_MOTOR_TRIM_PERCENT) / 100);
}

const DemoStep demo_steps[] = {
    {"forward low", -LOW_SPEED, trim_right_speed(LOW_SPEED), FORWARD_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
    {"forward high", -MAX_SPEED, trim_right_speed(MAX_SPEED), FORWARD_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
    {"left turn", TURN_SPEED, trim_right_speed(TURN_SPEED), TURN_90_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
    {"right turn", -TURN_SPEED, trim_right_speed(-TURN_SPEED), TURN_90_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
    {"u-turn", TURN_SPEED, trim_right_speed(TURN_SPEED), U_TURN_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
};

constexpr size_t STEP_COUNT = sizeof(demo_steps) / sizeof(demo_steps[0]);

const uint8_t qtr_rc_sensor_pins[robot_config::QTR_SENSOR_COUNT] = {
    45, 46, 47, 48, 49, 50, 51, 52, 53,
};

TaskScheduler scheduler;
MotoronI2C motoron;
MFRC522_I2C rfid(robot_config::RFID_I2C_ADDRESS, robot_config::RFID_RESET_PIN, &Wire1);

bool motoron_ready = false;
bool rfid_ready = false;
bool rfid_seen = false;
bool stopped_by_button = robot_config::ENABLE_MECHANICAL_KILL_BUTTON;
bool revive_button_pressed = false;
bool red_blink_on = true;
char last_rfid_uid[32] = "none";

int last_button_reading = HIGH;
int stable_button_state = HIGH;
int last_revive_button_reading = HIGH;
int stable_revive_button_state = HIGH;

unsigned long last_button_change_ms = 0;
unsigned long last_revive_button_change_ms = 0;
unsigned long step_start_ms = 0;
unsigned long stop_started_ms = 0;
bool was_stopped = true;
size_t step_index = 0;
uint16_t qtr_raw_values[robot_config::QTR_SENSOR_COUNT] = {};
uint16_t qtr_min_values[robot_config::QTR_SENSOR_COUNT] = {};
uint16_t qtr_max_values[robot_config::QTR_SENSOR_COUNT] = {};
uint16_t qtr_calibrated_values[robot_config::QTR_SENSOR_COUNT] = {};
unsigned long qtr_calibration_start_ms = 0;
bool qtr_calibration_done = false;
bool qtr_filter_ready = false;
bool qtr_line_detected = false;
SeedDispenserState seed_dispenser_state = SeedDispenserState::Idle;
int seed_upper_pulse_us = 1500;
int seed_lower_pulse_us = 1500;
unsigned long seed_last_servo_frame_us = 0;
unsigned long seed_state_start_ms = 0;

bool robot_stopped() {
    return stopped_by_button;
}

bool i2c_device_present(TwoWire& bus, uint8_t address) {
    bus.beginTransmission(address);
    return bus.endTransmission() == 0;
}

bool rfid_firmware_version_valid(byte version) {
    return version == 0x15 ||
           version == 0x90 ||
           version == 0x91 ||
           version == 0x92 ||
           version == 0xB2;
}

uint8_t rgb_level(bool on) {
    if (robot_config::TOP_RGB_COMMON_ANODE) {
        return on ? 0 : 255;
    }
    return on ? 255 : 0;
}

void write_top_rgb(bool red_on, bool green_on, bool blue_on) {
    if (!robot_config::ENABLE_TOP_RGB) {
        return;
    }
    if (robot_config::TOP_RGB_RED_PIN >= 0) {
        analogWrite(robot_config::TOP_RGB_RED_PIN, rgb_level(red_on));
    }
    if (robot_config::TOP_RGB_GREEN_PIN >= 0) {
        analogWrite(robot_config::TOP_RGB_GREEN_PIN, rgb_level(green_on));
    }
    if (robot_config::TOP_RGB_BLUE_PIN >= 0) {
        analogWrite(robot_config::TOP_RGB_BLUE_PIN, rgb_level(blue_on));
    }
}

void render_status_led() {
    if (!robot_config::ENABLE_TOP_RGB) {
        return;
    }

    if (revive_button_pressed) {
        write_top_rgb(false, true, false);
        return;
    }

    if (robot_stopped()) {
        write_top_rgb(red_blink_on, false, false);
        return;
    }

    write_top_rgb(true, false, false);
}

void stop_motors() {
    if (!motoron_ready) {
        return;
    }
    motoron.setSpeed(MOTOR_LEFT, 0);
    motoron.setSpeed(MOTOR_RIGHT, 0);
    motoron.setSpeed(MOTOR_AUX, 0);
}

void set_motor_speeds(int16_t left_speed, int16_t right_speed) {
    if (!motoron_ready) {
        return;
    }
    motoron.setSpeed(MOTOR_LEFT, left_speed);
    motoron.setSpeed(MOTOR_RIGHT, right_speed);
    motoron.setSpeed(MOTOR_AUX, 0);

    if (motoron.getLastError() != 0) {
        motoron_ready = false;
        Serial.print("motoron_error=");
        Serial.println(motoron.getLastError());
    }
}

void configure_motoron_motor(uint8_t motor) {
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
    configure_motoron_motor(MOTOR_LEFT);
    configure_motoron_motor(MOTOR_RIGHT);
    configure_motoron_motor(MOTOR_AUX);

    motoron_ready = motoron.getLastError() == 0;
    Serial.print("motoron_ready=");
    Serial.print(motoron_ready ? 1 : 0);
    Serial.print(" error=");
    Serial.println(motoron.getLastError());
    stop_motors();
}

void begin_rfid() {
    if (!robot_config::ENABLE_RFID_STATUS) {
        return;
    }

    Wire1.begin();
    Wire1.setClock(100000);
    delay(10);

    const byte firmware_version = rfid.PCD_ReadRegister(rfid.VersionReg);
    const bool present = i2c_device_present(Wire1, robot_config::RFID_I2C_ADDRESS);

    Serial.print("rfid_addr=0x");
    if (robot_config::RFID_I2C_ADDRESS < 0x10) {
        Serial.print('0');
    }
    Serial.print(robot_config::RFID_I2C_ADDRESS, HEX);
    Serial.print(" present=");
    Serial.print(present ? 1 : 0);
    Serial.print(" firmware=0x");
    if (firmware_version < 0x10) {
        Serial.print('0');
    }
    Serial.print(firmware_version, HEX);

    if (!present || !rfid_firmware_version_valid(firmware_version)) {
        rfid_ready = false;
        Serial.println(" ready=0");
        return;
    }

    // This WS1850S unit reports VersionReg=0x15 but can loop forever in the
    // library soft reset path. Apply the runtime setup while skipping reset.
    rfid.PCD_WriteRegister(rfid.TModeReg, 0x80);
    rfid.PCD_WriteRegister(rfid.TPrescalerReg, 0xA9);
    rfid.PCD_WriteRegister(rfid.TReloadRegH, 0x03);
    rfid.PCD_WriteRegister(rfid.TReloadRegL, 0xE8);
    rfid.PCD_WriteRegister(rfid.TxASKReg, 0x40);
    rfid.PCD_WriteRegister(rfid.ModeReg, 0x3D);
    rfid.PCD_AntennaOn();
    const byte firmware_version_after_init = rfid.PCD_ReadRegister(rfid.VersionReg);
    rfid_ready = rfid_firmware_version_valid(firmware_version_after_init);
    Serial.print(" firmware_after_init=0x");
    if (firmware_version_after_init < 0x10) {
        Serial.print('0');
    }
    Serial.print(firmware_version_after_init, HEX);
    Serial.print(" ready=");
    Serial.println(rfid_ready ? 1 : 0);
}

void update_rfid() {
    if (!robot_config::ENABLE_RFID_STATUS || !rfid_ready) {
        return;
    }

    if (!rfid.PICC_IsNewCardPresent()) {
        return;
    }
    if (!rfid.PICC_ReadCardSerial()) {
        return;
    }

    size_t offset = 0;
    for (byte i = 0; i < rfid.uid.size && offset + 3 < sizeof(last_rfid_uid); ++i) {
        const int written = snprintf(
            last_rfid_uid + offset,
            sizeof(last_rfid_uid) - offset,
            "%02X",
            rfid.uid.uidByte[i]);
        if (written <= 0) {
            break;
        }
        offset += static_cast<size_t>(written);
        if (i + 1 < rfid.uid.size && offset + 2 < sizeof(last_rfid_uid)) {
            last_rfid_uid[offset++] = ' ';
            last_rfid_uid[offset] = '\0';
        }
    }

    rfid_seen = true;
    Serial.print("RFID UID: ");
    Serial.println(last_rfid_uid);
    rfid.PICC_HaltA();
}

void print_demo_step() {
    const DemoStep& step = demo_steps[step_index];
    Serial.print("mode=");
    Serial.print(step.name);
    Serial.print(" step=");
    Serial.print(step_index + 1);
    Serial.print('/');
    Serial.print(STEP_COUNT);
    Serial.print(" motor=");
    Serial.print(step.left_speed);
    Serial.print(',');
    Serial.print(step.right_speed);
    Serial.print(" duration_ms=");
    Serial.println(step.duration_ms);
}

void apply_demo_step() {
    step_start_ms = millis();
    print_demo_step();

    if (robot_stopped()) {
        stop_motors();
        return;
    }

    const DemoStep& step = demo_steps[step_index];
    set_motor_speeds(step.left_speed, step.right_speed);
}

void update_stop_transition() {
    const bool stopped = robot_stopped();
    const unsigned long now_ms = millis();

    if (stopped && !was_stopped) {
        stop_started_ms = now_ms;
        stop_motors();
    } else if (!stopped && was_stopped) {
        step_start_ms += now_ms - stop_started_ms;
        const DemoStep& step = demo_steps[step_index];
        set_motor_speeds(step.left_speed, step.right_speed);
    }

    was_stopped = stopped;
}

void update_demo_motion() {
    update_stop_transition();
    if (robot_stopped()) {
        stop_motors();
        return;
    }

    const unsigned long now_ms = millis();
    const DemoStep& step = demo_steps[step_index];
    if (now_ms - step_start_ms >= step.duration_ms) {
        step_index = (step_index + 1) % STEP_COUNT;
        apply_demo_step();
        return;
    }

    set_motor_speeds(step.left_speed, step.right_speed);
}

void update_mechanical_kill_button() {
    if (!robot_config::ENABLE_MECHANICAL_KILL_BUTTON ||
        robot_config::MECHANICAL_KILL_BUTTON_PIN < 0) {
        return;
    }

    const int reading = digitalRead(robot_config::MECHANICAL_KILL_BUTTON_PIN);
    const unsigned long now_ms = millis();

    if (reading != last_button_reading) {
        last_button_change_ms = now_ms;
        last_button_reading = reading;
    }

    if (now_ms - last_button_change_ms < BUTTON_DEBOUNCE_MS || reading == stable_button_state) {
        return;
    }

    stable_button_state = reading;
    if (stable_button_state == LOW) {
        stopped_by_button = !stopped_by_button;
        red_blink_on = robot_stopped();
        update_stop_transition();
        render_status_led();
        Serial.print("button_stop=");
        Serial.println(stopped_by_button ? 1 : 0);
    }
}

void update_revive_button() {
    if (!robot_config::ENABLE_REVIVE_BUTTON || robot_config::REVIVE_BUTTON_PIN < 0) {
        return;
    }

    const int reading = digitalRead(robot_config::REVIVE_BUTTON_PIN);
    const unsigned long now_ms = millis();

    if (reading != last_revive_button_reading) {
        last_revive_button_change_ms = now_ms;
        last_revive_button_reading = reading;
    }

    if (now_ms - last_revive_button_change_ms < BUTTON_DEBOUNCE_MS ||
        reading == stable_revive_button_state) {
        return;
    }

    stable_revive_button_state = reading;
    revive_button_pressed = stable_revive_button_state == LOW;
    render_status_led();
    Serial.print("revive_button=");
    Serial.println(revive_button_pressed ? 1 : 0);
}

void update_status_led() {
    if (!robot_config::ENABLE_TOP_RGB) {
        return;
    }

    if (!robot_stopped()) {
        red_blink_on = false;
        render_status_led();
        return;
    }

    if (!revive_button_pressed) {
        red_blink_on = !red_blink_on;
    }
    render_status_led();
}

int angle_to_seed_servo_pulse_us(int angle) {
    angle = constrain(angle, 0, 180);
    return map(
        angle,
        0,
        180,
        robot_config::SEED_SERVO_MIN_PULSE_US,
        robot_config::SEED_SERVO_MAX_PULSE_US);
}

void write_seed_upper_servo(int angle) {
    seed_upper_pulse_us = angle_to_seed_servo_pulse_us(angle);
}

void write_seed_lower_servo(int angle) {
    seed_lower_pulse_us = angle_to_seed_servo_pulse_us(angle);
}

const char* seed_dispenser_state_name() {
    switch (seed_dispenser_state) {
        case SeedDispenserState::Idle:
            return "idle";
        case SeedDispenserState::UpperOpen:
            return "upper_open";
        case SeedDispenserState::UpperClosing:
            return "upper_closing";
        case SeedDispenserState::LowerOpen:
            return "lower_open";
        case SeedDispenserState::LowerClosing:
            return "lower_closing";
        default:
            return "unknown";
    }
}

void service_seed_servo_pulses() {
    if (!robot_config::ENABLE_SEED_DISPENSER ||
        robot_config::SEED_UPPER_SERVO_PIN < 0 ||
        robot_config::SEED_LOWER_SERVO_PIN < 0) {
        return;
    }

    const unsigned long now_us = micros();
    if (now_us - seed_last_servo_frame_us < robot_config::SEED_SERVO_PERIOD_US) {
        return;
    }
    seed_last_servo_frame_us = now_us;

    digitalWrite(robot_config::SEED_UPPER_SERVO_PIN, HIGH);
    delayMicroseconds(seed_upper_pulse_us);
    digitalWrite(robot_config::SEED_UPPER_SERVO_PIN, LOW);

    digitalWrite(robot_config::SEED_LOWER_SERVO_PIN, HIGH);
    delayMicroseconds(seed_lower_pulse_us);
    digitalWrite(robot_config::SEED_LOWER_SERVO_PIN, LOW);
}

void close_seed_gates() {
    write_seed_upper_servo(robot_config::SEED_UPPER_CLOSED_ANGLE);
    write_seed_lower_servo(robot_config::SEED_LOWER_CLOSED_ANGLE);
    seed_dispenser_state = SeedDispenserState::Idle;
}

void start_seed_dispense_cycle() {
    if (!robot_config::ENABLE_SEED_DISPENSER) {
        Serial.println("seed_dispenser=disabled");
        return;
    }

    if (seed_dispenser_state != SeedDispenserState::Idle) {
        Serial.println("seed_dispenser=busy");
        return;
    }

    write_seed_lower_servo(robot_config::SEED_LOWER_CLOSED_ANGLE);
    write_seed_upper_servo(robot_config::SEED_UPPER_OPEN_ANGLE);
    seed_dispenser_state = SeedDispenserState::UpperOpen;
    seed_state_start_ms = millis();
    Serial.println("seed_dispenser=dispense_start");
}

void update_seed_dispenser() {
    if (!robot_config::ENABLE_SEED_DISPENSER ||
        seed_dispenser_state == SeedDispenserState::Idle) {
        return;
    }

    const unsigned long now_ms = millis();
    const unsigned long elapsed_ms = now_ms - seed_state_start_ms;

    switch (seed_dispenser_state) {
        case SeedDispenserState::UpperOpen:
            if (elapsed_ms >= robot_config::SEED_UPPER_OPEN_MS) {
                write_seed_upper_servo(robot_config::SEED_UPPER_CLOSED_ANGLE);
                seed_dispenser_state = SeedDispenserState::UpperClosing;
                seed_state_start_ms = now_ms;
            }
            break;
        case SeedDispenserState::UpperClosing:
            if (elapsed_ms >= robot_config::SEED_UPPER_CLOSE_MS) {
                write_seed_lower_servo(robot_config::SEED_LOWER_OPEN_ANGLE);
                seed_dispenser_state = SeedDispenserState::LowerOpen;
                seed_state_start_ms = now_ms;
            }
            break;
        case SeedDispenserState::LowerOpen:
            if (elapsed_ms >= robot_config::SEED_LOWER_OPEN_MS) {
                write_seed_lower_servo(robot_config::SEED_LOWER_CLOSED_ANGLE);
                seed_dispenser_state = SeedDispenserState::LowerClosing;
                seed_state_start_ms = now_ms;
            }
            break;
        case SeedDispenserState::LowerClosing:
            if (elapsed_ms >= robot_config::SEED_LOWER_CLOSE_MS) {
                seed_dispenser_state = SeedDispenserState::Idle;
                Serial.println("seed_dispenser=dispense_done");
            }
            break;
        case SeedDispenserState::Idle:
            break;
    }
}

void process_serial_commands() {
    while (Serial.available() > 0) {
        const char cmd = static_cast<char>(Serial.read());
        if (cmd == 'd' || cmd == 'D') {
            start_seed_dispense_cycle();
        } else if (cmd == 'c' || cmd == 'C') {
            close_seed_gates();
            Serial.println("seed_dispenser=closed");
        }
    }
}

void begin_seed_dispenser() {
    if (!robot_config::ENABLE_SEED_DISPENSER) {
        return;
    }

    pinMode(robot_config::SEED_UPPER_SERVO_PIN, OUTPUT);
    pinMode(robot_config::SEED_LOWER_SERVO_PIN, OUTPUT);
    digitalWrite(robot_config::SEED_UPPER_SERVO_PIN, LOW);
    digitalWrite(robot_config::SEED_LOWER_SERVO_PIN, LOW);
    close_seed_gates();
    seed_last_servo_frame_us = 0;
    Serial.println("Seed dispenser ready. Serial commands: d=dispense, c=close");
}

void read_qtr_raw_values() {
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        pinMode(qtr_rc_sensor_pins[i], OUTPUT);
        digitalWrite(qtr_rc_sensor_pins[i], HIGH);
    }

    delayMicroseconds(15);

    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        pinMode(qtr_rc_sensor_pins[i], INPUT);
        qtr_raw_values[i] = robot_config::QTR_RC_TIMEOUT_US;
    }

    const unsigned long start_time_us = micros();
    while (micros() - start_time_us < robot_config::QTR_RC_TIMEOUT_US) {
        const uint16_t elapsed_us =
            static_cast<uint16_t>(micros() - start_time_us);
        for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
            if (qtr_raw_values[i] == robot_config::QTR_RC_TIMEOUT_US &&
                digitalRead(qtr_rc_sensor_pins[i]) == LOW) {
                qtr_raw_values[i] = elapsed_us;
            }
        }
    }
}

void reset_qtr_calibration() {
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        qtr_min_values[i] = robot_config::QTR_RC_TIMEOUT_US;
        qtr_max_values[i] = 0;
    }
}

void update_qtr_calibration() {
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        if (qtr_raw_values[i] < qtr_min_values[i]) {
            qtr_min_values[i] = qtr_raw_values[i];
        }
        if (qtr_raw_values[i] > qtr_max_values[i]) {
            qtr_max_values[i] = qtr_raw_values[i];
        }
    }
}

void update_qtr_calibrated_values() {
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        const uint16_t range = qtr_max_values[i] - qtr_min_values[i];
        uint16_t value = 0;
        if (range > 0 && qtr_raw_values[i] > qtr_min_values[i]) {
            value = static_cast<uint16_t>(
                min<uint32_t>(
                    1000,
                    (static_cast<uint32_t>(qtr_raw_values[i] - qtr_min_values[i]) * 1000) / range));
        }

        const uint16_t instant_value = robot_config::QTR_LINE_IS_HIGH_RAW ? value : 1000 - value;
        if (!qtr_filter_ready) {
            qtr_calibrated_values[i] = instant_value;
        } else {
            qtr_calibrated_values[i] = static_cast<uint16_t>(
                ((static_cast<uint32_t>(qtr_calibrated_values[i]) *
                  (100 - robot_config::QTR_SMOOTHING_PERCENT)) +
                 (static_cast<uint32_t>(instant_value) * robot_config::QTR_SMOOTHING_PERCENT)) /
                100);
        }
    }
    qtr_filter_ready = true;
}

bool update_qtr_line_found() {
    uint16_t peak = 0;
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        const uint16_t target_value =
            robot_config::QTR_FOLLOW_BLACK_LINE ? qtr_calibrated_values[i] : 1000 - qtr_calibrated_values[i];
        if (target_value > peak) {
            peak = target_value;
        }
    }

    if (qtr_line_detected) {
        qtr_line_detected = peak >= robot_config::QTR_LINE_DETECT_OFF_THRESHOLD;
    } else {
        qtr_line_detected = peak >= robot_config::QTR_LINE_DETECT_ON_THRESHOLD;
    }

    return qtr_line_detected;
}

uint16_t qtr_line_peak() {
    uint16_t peak = 0;
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        const uint16_t target_value =
            robot_config::QTR_FOLLOW_BLACK_LINE ? qtr_calibrated_values[i] : 1000 - qtr_calibrated_values[i];
        if (target_value > peak) {
            peak = target_value;
        }
    }
    return peak;
}

int32_t estimate_qtr_line_position() {
    uint32_t weighted_sum = 0;
    uint32_t sum = 0;

    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        const uint16_t target_value =
            robot_config::QTR_FOLLOW_BLACK_LINE ? qtr_calibrated_values[i] : 1000 - qtr_calibrated_values[i];
        weighted_sum += static_cast<uint32_t>(target_value) * i * 1000;
        sum += target_value;
    }

    if (sum == 0) {
        return -1;
    }

    return static_cast<int32_t>(weighted_sum / sum);
}

void print_qtr_compact_values(const char* label, const uint16_t* values) {
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

void print_qtr_surfaces() {
    Serial.print("surface=[");
    for (size_t i = 0; i < robot_config::QTR_SENSOR_COUNT; ++i) {
        if (i > 0) {
            Serial.print(',');
        }

        const uint16_t black_value = qtr_calibrated_values[i];
        const uint16_t white_value = 1000 - qtr_calibrated_values[i];

        if (black_value >= white_value + robot_config::QTR_SURFACE_DECISION_MARGIN) {
            Serial.print('B');
        } else {
            Serial.print('W');
        }
    }
    Serial.print(']');
}

void update_qtr_status() {
    if (!robot_config::ENABLE_QTR_STATUS) {
        return;
    }

    read_qtr_raw_values();

    if (!qtr_calibration_done) {
        update_qtr_calibration();
        if (millis() - qtr_calibration_start_ms >= QTR_CALIBRATION_TIME_MS) {
            qtr_calibration_done = true;
            qtr_filter_ready = false;
            Serial.println("QTR calibration complete");
            print_qtr_compact_values("qtr_min", qtr_min_values);
            Serial.print(' ');
            print_qtr_compact_values("qtr_max", qtr_max_values);
            Serial.println();
        }
        return;
    }

    update_qtr_calibrated_values();
    qtr_line_detected = update_qtr_line_found();
}

void begin_qtr() {
    if (!robot_config::ENABLE_QTR_STATUS) {
        return;
    }

    if (robot_config::QTR_EMITTER_PIN != 255) {
        pinMode(robot_config::QTR_EMITTER_PIN, OUTPUT);
        digitalWrite(robot_config::QTR_EMITTER_PIN, HIGH);
    }

    reset_qtr_calibration();
    qtr_filter_ready = false;
    qtr_line_detected = false;
    qtr_calibration_done = false;
    qtr_calibration_start_ms = millis();

    Serial.println("QTR RC bypass: pins=D45,D46,D47,D48,D49,D50,D51,D52,D53");
    Serial.print("QTR RC bypass: emitter=");
    if (robot_config::QTR_EMITTER_PIN == 255) {
        Serial.println("disabled");
    } else {
        Serial.println(robot_config::QTR_EMITTER_PIN);
    }
    Serial.println("QTR: move array across floor and strip for 5 seconds");
}

void print_qtr_status() {
    if (!robot_config::ENABLE_QTR_STATUS) {
        Serial.print("qtr=disabled");
        return;
    }

    update_qtr_status();

    Serial.print(qtr_calibration_done ? "qtr " : "qtr_calibrating ");
    print_qtr_compact_values("raw", qtr_raw_values);
    if (!qtr_calibration_done) {
        return;
    }

    Serial.print(' ');
    print_qtr_compact_values("cal", qtr_calibrated_values);
    Serial.print(' ');
    print_qtr_surfaces();
    Serial.print(" peak=");
    Serial.print(qtr_line_peak());
    Serial.print(" found=");
    Serial.print(qtr_line_detected ? 1 : 0);
    Serial.print(" line=");
    Serial.print(estimate_qtr_line_position());
}

float read_distance_cm() {
    digitalWrite(robot_config::ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(robot_config::ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(robot_config::ULTRASONIC_TRIG_PIN, LOW);

    const unsigned long duration_us =
        pulseIn(robot_config::ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_ECHO_TIMEOUT_US);
    if (duration_us == 0) {
        return -1.0f;
    }

    return (duration_us * 0.0343f) / 2.0f;
}

void print_ultrasonic_status() {
    if (!robot_config::ENABLE_ULTRASONIC_STATUS) {
        Serial.print("distance=disabled");
        return;
    }

    const float distance_cm = read_distance_cm();
    Serial.print("distance_cm=");
    if (distance_cm < 0.0f) {
        Serial.print("out_of_range");
    } else {
        Serial.print(distance_cm, 1);
    }
}

void print_rfid_status() {
    if (!robot_config::ENABLE_RFID_STATUS) {
        Serial.print("rfid=disabled");
        return;
    }

    update_rfid();

    Serial.print("rfid_ready=");
    Serial.print(rfid_ready ? 1 : 0);
    Serial.print(" rfid_uid=");
    Serial.print(rfid_seen ? last_rfid_uid : "none");
}

void print_seed_dispenser_status() {
    if (!robot_config::ENABLE_SEED_DISPENSER) {
        Serial.print("planter=disabled");
        return;
    }

    Serial.print("planter=");
    Serial.print(seed_dispenser_state_name());
}

void print_status() {
    const DemoStep& step = demo_steps[step_index];

    Serial.print("state=");
    Serial.print(robot_stopped() ? "stopped" : "running");
    Serial.print(" button_stop=");
    Serial.print(stopped_by_button ? 1 : 0);
    Serial.print(" revive=");
    Serial.print(revive_button_pressed ? 1 : 0);
    Serial.print(" motoron=");
    Serial.print(motoron_ready ? 1 : 0);
    Serial.print(" mode=");
    Serial.print(step.name);
    Serial.print(' ');
    print_qtr_status();
    Serial.print(' ');
    print_seed_dispenser_status();
    Serial.print(' ');
    print_ultrasonic_status();
    Serial.print(' ');
    print_rfid_status();
    Serial.println();
}

void print_feature_config() {
    Serial.print("features top_rgb=");
    Serial.print(robot_config::ENABLE_TOP_RGB ? 1 : 0);
    Serial.print(" mechanical_kill=");
    Serial.print(robot_config::ENABLE_MECHANICAL_KILL_BUTTON ? 1 : 0);
    Serial.print(" revive=");
    Serial.print(robot_config::ENABLE_REVIVE_BUTTON ? 1 : 0);
    Serial.print(" qtr=");
    Serial.print(robot_config::ENABLE_QTR_STATUS ? 1 : 0);
    Serial.print(" ultrasonic=");
    Serial.print(robot_config::ENABLE_ULTRASONIC_STATUS ? 1 : 0);
    Serial.print(" seed_dispenser=");
    Serial.print(robot_config::ENABLE_SEED_DISPENSER ? 1 : 0);
    Serial.print(" rfid=");
    Serial.println(robot_config::ENABLE_RFID_STATUS ? 1 : 0);
}

}  // namespace

void robot_app_setup() {
    log_begin();
    LOG_INFO("term3_robot integrated trial demo starting");
    print_feature_config();

    if (robot_config::ENABLE_TOP_RGB) {
        if (robot_config::TOP_RGB_RED_PIN >= 0) {
            pinMode(robot_config::TOP_RGB_RED_PIN, OUTPUT);
        }
        if (robot_config::TOP_RGB_GREEN_PIN >= 0) {
            pinMode(robot_config::TOP_RGB_GREEN_PIN, OUTPUT);
        }
        if (robot_config::TOP_RGB_BLUE_PIN >= 0) {
            pinMode(robot_config::TOP_RGB_BLUE_PIN, OUTPUT);
        }
    }

    if (robot_config::ENABLE_MECHANICAL_KILL_BUTTON &&
        robot_config::MECHANICAL_KILL_BUTTON_PIN >= 0) {
        pinMode(robot_config::MECHANICAL_KILL_BUTTON_PIN, INPUT_PULLUP);
        last_button_reading = digitalRead(robot_config::MECHANICAL_KILL_BUTTON_PIN);
        stable_button_state = last_button_reading;
        last_button_change_ms = millis();
    }

    if (robot_config::ENABLE_REVIVE_BUTTON && robot_config::REVIVE_BUTTON_PIN >= 0) {
        pinMode(robot_config::REVIVE_BUTTON_PIN, INPUT_PULLUP);
        last_revive_button_reading = digitalRead(robot_config::REVIVE_BUTTON_PIN);
        stable_revive_button_state = last_revive_button_reading;
        revive_button_pressed = stable_revive_button_state == LOW;
        last_revive_button_change_ms = millis();
    }

    begin_qtr();
    begin_seed_dispenser();

    if (robot_config::ENABLE_ULTRASONIC_STATUS) {
        pinMode(robot_config::ULTRASONIC_TRIG_PIN, OUTPUT);
        pinMode(robot_config::ULTRASONIC_ECHO_PIN, INPUT);
        digitalWrite(robot_config::ULTRASONIC_TRIG_PIN, LOW);
    }

    render_status_led();
    begin_motoron();
    begin_rfid();

    step_start_ms = millis();
    stop_started_ms = millis();
    was_stopped = robot_stopped();
    print_demo_step();

    scheduler.add(MOTOR_UPDATE_INTERVAL_MS, update_demo_motion, "motion");
    scheduler.add(1, service_seed_servo_pulses, "seed_servo");
    scheduler.add(20, update_seed_dispenser, "seed_dispense");
    scheduler.add(20, process_serial_commands, "serial");
    scheduler.add(QTR_UPDATE_INTERVAL_MS, update_qtr_status, "qtr");
    scheduler.add(10, update_mechanical_kill_button, "kill_button");
    scheduler.add(10, update_revive_button, "revive_button");
    scheduler.add(STOP_LED_BLINK_INTERVAL_MS, update_status_led, "status_led");
    scheduler.add(STATUS_INTERVAL_MS, print_status, "status");
    scheduler.reset();
}

void robot_app_loop() {
    scheduler.tick(millis());
}
