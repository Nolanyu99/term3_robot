#include <Arduino.h>
#include <Motoron.h>
#include <Wire.h>

// =====================================================
// Single-file Arduino IDE racetrack line follower
// Board: Arduino GIGA R1 WiFi
// Motor driver: Motoron over Wire1
// Sensor array: 9 QTR-RC style reflectance sensors
// =====================================================

namespace robot_config {

// QTR-RC reflectance sensor array.
constexpr size_t QTR_SENSOR_COUNT = 9;
constexpr uint16_t QTR_RC_TIMEOUT_US = 1000;

// Set to 255 if the emitter LEDs are wired directly to power.
constexpr uint8_t QTR_EMITTER_PIN = 255;

// Flip this after checking raw values if black tape gives lower readings.
constexpr bool QTR_LINE_IS_HIGH_RAW = true;
constexpr bool QTR_FOLLOW_BLACK_LINE = true;
constexpr uint16_t QTR_LINE_DETECT_ON_THRESHOLD = 650;
constexpr uint16_t QTR_LINE_DETECT_OFF_THRESHOLD = 450;

// Other pins/settings kept from the original config for reference.
constexpr bool ENABLE_TOP_RGB = true;
constexpr int TOP_RGB_RED_PIN = 39;
constexpr int TOP_RGB_BLUE_PIN = 37;
constexpr int TOP_RGB_GREEN_PIN = 35;
constexpr bool TOP_RGB_COMMON_ANODE = false;
constexpr int MECHANICAL_KILL_BUTTON_PIN = 33;
constexpr int REVIVE_BUTTON_PIN = 13;
constexpr uint8_t RFID_I2C_ADDRESS = 0x28;
constexpr int RFID_RESET_PIN = -1;

}  // namespace robot_config

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
constexpr unsigned long POST_CALIBRATION_WAIT_MS = 1500;

// Faster loop reduces late corrections and overshoot.
constexpr unsigned long CONTROL_INTERVAL_MS = 10;
constexpr unsigned long STATUS_INTERVAL_MS = 250;
constexpr unsigned long INTERSECTION_CROSS_MS = 220;
constexpr unsigned long LOST_FORWARD_MS = 70;

constexpr uint8_t SENSOR_COUNT = 9;
constexpr uint16_t LINE_POSITION_SCALE = 1000;
constexpr int32_t LINE_CENTER = ((SENSOR_COUNT - 1) * LINE_POSITION_SCALE) / 2;
constexpr uint16_t INTERSECTION_THRESHOLD = 830;
constexpr uint8_t INTERSECTION_SENSOR_COUNT = 7;

constexpr uint8_t MOTOR_LEFT = 1;
constexpr uint8_t MOTOR_RIGHT = 2;
constexpr uint8_t MOTOR_AUX = 3;

// Speed tuning. Increase FAST_BASE_SPEED gradually if the robot is stable.
constexpr int16_t FAST_BASE_SPEED = 520;
constexpr int16_t SLOW_BASE_SPEED = 330;
constexpr int16_t MAX_SPEED = 720;
constexpr int16_t MIN_FORWARD_SPEED = 160;
constexpr int16_t LOST_SEARCH_SPEED = 310;

// PID tuning for centred, less overshooting line following.
// error range is roughly -4000 to +4000.
constexpr float LINE_KP = 0.115f;
constexpr float LINE_KD = 0.46f;
constexpr float LINE_KI = 0.0008f;
constexpr int32_t INTEGRAL_LIMIT = 16000;
constexpr int16_t CORRECTION_LIMIT = 560;

// Higher value = smoother motor commands, lower value = more responsive.
constexpr uint8_t MOTOR_SMOOTHING_PERCENT = 35;

constexpr int8_t LEFT_FORWARD_SIGN = 1;
constexpr int8_t RIGHT_FORWARD_SIGN = 1;
constexpr uint8_t RIGHT_MOTOR_TRIM_PERCENT = 95;
constexpr uint16_t MOTOR_ACCEL = 1600;
constexpr uint16_t MOTOR_DECEL = 2200;

// Original app pin map.
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
int32_t error_integral = 0;
int16_t last_left_command = 0;
int16_t last_right_command = 0;
bool line_detected = false;
bool motoron_ready = false;
bool run_enabled = false;

constexpr int16_t trim_right_speed(int16_t speed) {
  return static_cast<int16_t>((static_cast<int32_t>(speed) * RIGHT_MOTOR_TRIM_PERCENT) / 100);
}

const char* state_name() {
  switch (follow_state) {
    case FollowState::Idle: return "idle";
    case FollowState::FollowLine: return "follow";
    case FollowState::CrossIntersection: return "intersection";
    case FollowState::LostLine: return "lost";
    default: return "unknown";
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
      if (raw_values[i] == robot_config::QTR_RC_TIMEOUT_US && digitalRead(sensor_pins[i]) == LOW) {
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
    if (raw_values[i] < min_values[i]) min_values[i] = raw_values[i];
    if (raw_values[i] > max_values[i]) max_values[i] = raw_values[i];
  }
}

void update_calibrated_values() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint16_t range = max_values[i] - min_values[i];
    uint16_t value = 0;
    if (range > 0 && raw_values[i] > min_values[i]) {
      const uint32_t scaled = (static_cast<uint32_t>(raw_values[i] - min_values[i]) * 1000) / range;
      value = static_cast<uint16_t>(scaled > 1000 ? 1000 : scaled);
    }
    calibrated_values[i] = robot_config::QTR_LINE_IS_HIGH_RAW ? value : 1000 - value;
  }
}

uint16_t target_value_at(uint8_t sensor_index) {
  return robot_config::QTR_FOLLOW_BLACK_LINE ? calibrated_values[sensor_index] : 1000 - calibrated_values[sensor_index];
}

uint16_t line_peak() {
  uint16_t peak = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint16_t value = target_value_at(i);
    if (value > peak) peak = value;
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
    if (target_value_at(i) >= INTERSECTION_THRESHOLD) ++high_count;
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

  if (sum == 0) return -1;
  return static_cast<int32_t>(weighted_sum / sum);
}

void stop_motors() {
  last_left_command = 0;
  last_right_command = 0;
  if (!motoron_ready) return;
  motoron.setSpeed(MOTOR_LEFT, 0);
  motoron.setSpeed(MOTOR_RIGHT, 0);
  motoron.setSpeed(MOTOR_AUX, 0);
}

int16_t smooth_command(int16_t previous, int16_t target) {
  return static_cast<int16_t>(
    ((static_cast<int32_t>(previous) * MOTOR_SMOOTHING_PERCENT) +
     (static_cast<int32_t>(target) * (100 - MOTOR_SMOOTHING_PERCENT))) / 100);
}

void set_motor_speeds(int16_t left_speed, int16_t right_speed, bool smooth = true) {
  int16_t left_target = constrain(left_speed, -MAX_SPEED, MAX_SPEED);
  int16_t right_target = constrain(right_speed, -MAX_SPEED, MAX_SPEED);

  if (smooth) {
    left_target = smooth_command(last_left_command, left_target);
    right_target = smooth_command(last_right_command, right_target);
  }

  last_left_command = left_target;
  last_right_command = right_target;

  if (!motoron_ready) return;

  motoron.setSpeed(MOTOR_LEFT, last_left_command);
  motoron.setSpeed(MOTOR_RIGHT, last_right_command);
  motoron.setSpeed(MOTOR_AUX, 0);

  if (motoron.getLastError() != 0) {
    const uint16_t error = motoron.getLastError();
    motoron_ready = false;
    stop_motors();
    Serial.print("motoron_error=");
    Serial.println(error);
  }
}

void drive_forward(int16_t speed = FAST_BASE_SPEED) {
  set_motor_speeds(
    LEFT_FORWARD_SIGN * speed,
    trim_right_speed(RIGHT_FORWARD_SIGN * speed));
}

void search_for_line() {
  error_integral = 0;
  if (last_error < 0) {
    set_motor_speeds(
      -LEFT_FORWARD_SIGN * LOST_SEARCH_SPEED,
      trim_right_speed(RIGHT_FORWARD_SIGN * LOST_SEARCH_SPEED),
      false);
  } else {
    set_motor_speeds(
      LEFT_FORWARD_SIGN * LOST_SEARCH_SPEED,
      trim_right_speed(-RIGHT_FORWARD_SIGN * LOST_SEARCH_SPEED),
      false);
  }
}

int16_t adaptive_base_speed(int32_t abs_error, int32_t abs_derivative) {
  int32_t speed = FAST_BASE_SPEED;

  // Slow down on curves and sudden changes, stay fast on straight sections.
  speed -= abs_error / 10;
  speed -= abs_derivative / 18;

  return constrain(static_cast<int16_t>(speed), SLOW_BASE_SPEED, FAST_BASE_SPEED);
}

void follow_line(int32_t error) {
  const int32_t derivative = error - last_error;
  error_integral = constrain(error_integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  int16_t correction = static_cast<int16_t>(
    (LINE_KP * static_cast<float>(error)) +
    (LINE_KD * static_cast<float>(derivative)) +
    (LINE_KI * static_cast<float>(error_integral)));
  correction = constrain(correction, -CORRECTION_LIMIT, CORRECTION_LIMIT);

  const int16_t base_speed = adaptive_base_speed(abs(error), abs(derivative));

  int16_t left_output = base_speed + correction;
  int16_t right_output = base_speed - correction;

  // Keep both tracks moving forward except during lost-line recovery. This is
  // usually faster and prevents spin-overshoot on racetrack bends.
  left_output = constrain(left_output, MIN_FORWARD_SPEED, MAX_SPEED);
  right_output = constrain(right_output, MIN_FORWARD_SPEED, MAX_SPEED);

  set_motor_speeds(
    LEFT_FORWARD_SIGN * left_output,
    trim_right_speed(RIGHT_FORWARD_SIGN * right_output));

  last_error = error;
}

void set_follow_state(FollowState next_state) {
  if (follow_state == next_state) return;
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
  Wire1.setClock(400000);

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
      drive_forward(SLOW_BASE_SPEED);
    } else {
      search_for_line();
    }
    return;
  }

  if (follow_state == FollowState::CrossIntersection) {
    if (millis() - state_start_ms < INTERSECTION_CROSS_MS) {
      drive_forward(FAST_BASE_SPEED);
      return;
    }
    set_follow_state(FollowState::FollowLine);
  }

  if (intersection) {
    set_follow_state(FollowState::CrossIntersection);
    error_integral = 0;
    drive_forward(FAST_BASE_SPEED);
    return;
  }

  set_follow_state(FollowState::FollowLine);
  follow_line(error);
}

void print_array(const char* label, const uint16_t* values) {
  Serial.print(label);
  Serial.print("=[");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i > 0) Serial.print(',');
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
  Serial.println("Waiting briefly before automatic line following.");
}

void process_serial_commands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'g' || command == 'G') {
      run_enabled = true;
      error_integral = 0;
      set_follow_state(FollowState::FollowLine);
      Serial.println("line_follow=go");
    } else if (command == 'x' || command == 'X' || command == '0') {
      run_enabled = false;
      error_integral = 0;
      set_follow_state(FollowState::Idle);
      stop_motors();
      Serial.println("line_follow=stop");
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  const unsigned long serial_wait_start_ms = millis();
  while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }

  Serial.println();
  Serial.println("=== GIGA R1 fast centred racetrack line follower ===");
  Serial.println("Move sensors over floor and line during calibration.");
  Serial.println("Commands: g=start, x/0=stop");
  Serial.println("Pins: S1=D2 S2=D3 S3=D4 S4=D5 S5=D8 S6=D9 S7=D10 S8=D11 S9=D12");
  Serial.print("fast_base=");
  Serial.print(FAST_BASE_SPEED);
  Serial.print(" kp=");
  Serial.print(LINE_KP, 4);
  Serial.print(" kd=");
  Serial.print(LINE_KD, 4);
  Serial.print(" ki=");
  Serial.println(LINE_KI, 6);

  reset_calibration();
  startup_phase = StartupPhase::WaitingBeforeCalibration;
  startup_phase_start_ms = millis();
  last_control_ms = millis();
  last_status_ms = millis();
  begin_motoron();
  Serial.println("startup=waiting_before_calibration");
}

void loop() {
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
      error_integral = 0;
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
