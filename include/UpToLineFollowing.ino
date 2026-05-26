#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <math.h>
#include <MFRC522_I2C.h>

// =====================================================
// Integrated robot_config.hpp
// =====================================================

namespace robot_config {

constexpr const char* WIFI_SSID = "PhaseSpaceNetwork_2.4G";
constexpr const char* WIFI_PASSWORD = "8igMacNet";
constexpr uint16_t UDP_PORT = 4210;

constexpr bool ENABLE_TOP_RGB = true;
constexpr bool ENABLE_MECHANICAL_KILL_BUTTON = true;
constexpr bool ENABLE_REVIVE_BUTTON = true;
constexpr bool ENABLE_QTR_STATUS = true;
constexpr bool ENABLE_ULTRASONIC_STATUS = true;
constexpr bool ENABLE_RFID_STATUS = true;
constexpr bool ENABLE_SEED_DISPENSER = true;

constexpr int TOP_RGB_RED_PIN = 39;
constexpr int TOP_RGB_BLUE_PIN = 37;
constexpr int TOP_RGB_GREEN_PIN = 35;
constexpr bool TOP_RGB_COMMON_ANODE = false;

constexpr int MECHANICAL_KILL_BUTTON_PIN = 33;
constexpr int REVIVE_BUTTON_PIN = 13;

constexpr uint8_t LEFT_ENC_A = 22;
constexpr uint8_t LEFT_ENC_B = 23;
constexpr uint8_t RIGHT_ENC_A = 24;
constexpr uint8_t RIGHT_ENC_B = 25;

constexpr uint8_t LEFT_IN_A = 2;
constexpr uint8_t LEFT_IN_B = 4;
constexpr uint8_t LEFT_PWM = 9;
constexpr uint8_t LEFT_ENABLE = 6;
constexpr uint8_t RIGHT_IN_A = 7;
constexpr uint8_t RIGHT_IN_B = 8;
constexpr uint8_t RIGHT_PWM = 10;
constexpr uint8_t RIGHT_ENABLE = 12;

constexpr float MOTOR_GEAR_RATIO = 100.0f;
constexpr float MOTOR_RAW_CPR = 24.0f;

constexpr uint8_t QTR_FIRST_ANALOG_PIN = 0;
constexpr size_t QTR_SENSOR_COUNT = 9;
constexpr uint16_t QTR_RC_TIMEOUT_US = 1000;
constexpr uint8_t QTR_EMITTER_PIN = 255;

constexpr bool QTR_LINE_IS_HIGH_RAW = true;
constexpr bool QTR_FOLLOW_BLACK_LINE = true;
constexpr uint8_t QTR_RAW_SAMPLE_COUNT = 8;
constexpr uint8_t QTR_SMOOTHING_PERCENT = 35;
constexpr uint16_t QTR_LINE_DETECT_ON_THRESHOLD = 650;
constexpr uint16_t QTR_LINE_DETECT_OFF_THRESHOLD = 450;
constexpr uint16_t QTR_DEMO_LINE_DETECT_ON_THRESHOLD = 300;
constexpr uint16_t QTR_DEMO_LINE_DETECT_OFF_THRESHOLD = 180;
constexpr bool QTR_DEMO_AUTO_POLARITY = true;
constexpr uint16_t QTR_SURFACE_DECISION_MARGIN = 120;

constexpr int SEED_UPPER_SERVO_PIN = 36;
constexpr int SEED_LOWER_SERVO_PIN = 38;
constexpr int SEED_UPPER_CLOSED_ANGLE = 90;
constexpr int SEED_UPPER_OPEN_ANGLE = 0;
constexpr int SEED_LOWER_CLOSED_ANGLE = 0;
constexpr int SEED_LOWER_OPEN_ANGLE = 90;
constexpr int SEED_SERVO_MIN_PULSE_US = 500;
constexpr int SEED_SERVO_MAX_PULSE_US = 2500;
constexpr unsigned long SEED_SERVO_PERIOD_US = 20000;
constexpr unsigned long SEED_UPPER_OPEN_MS = 700;
constexpr unsigned long SEED_UPPER_CLOSE_MS = 1000;
constexpr unsigned long SEED_LOWER_OPEN_MS = 700;
constexpr unsigned long SEED_LOWER_CLOSE_MS = 500;

constexpr uint8_t ULTRASONIC_TRIG_PIN = 52;
constexpr uint8_t ULTRASONIC_ECHO_PIN = 53;
constexpr float ULTRASONIC_MAX_DISTANCE_CM = 400.0f;
constexpr float ULTRASONIC_WALL_THRESHOLD_CM = 20.0f;

constexpr uint8_t RFID_I2C_ADDRESS = 0x28;
constexpr int RFID_RESET_PIN = -1;

}

// =====================================================
// State
// =====================================================

enum class FollowState {
  Idle,
  FollowLine,
  CrossIntersection,
  LostLine,
};

// =====================================================
// Timing constants
// =====================================================

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 5000;
constexpr unsigned long CALIBRATION_TIME_MS = 5000;
constexpr unsigned long CONTROL_INTERVAL_MS = 30;
constexpr unsigned long LINE_STATUS_INTERVAL_MS = 250;
constexpr unsigned long INTERSECTION_CROSS_MS = 500;
constexpr unsigned long LOST_FORWARD_MS = 250;
constexpr unsigned long RFID_SCAN_COOLDOWN_MS = 1500;

// =====================================================
// Line sensor section
// =====================================================

constexpr uint8_t SENSOR_COUNT = 9;
constexpr uint16_t LINE_POSITION_SCALE = 1000;
constexpr int32_t LINE_CENTER = ((SENSOR_COUNT - 1) * LINE_POSITION_SCALE) / 2;
constexpr uint16_t INTERSECTION_THRESHOLD = 800;
constexpr uint8_t INTERSECTION_SENSOR_COUNT = 7;

const uint8_t sensor_pins[SENSOR_COUNT] = {2, 3, 4, 5, 8, 9, 10, 11, 12};

uint16_t raw_values[SENSOR_COUNT] = {};
uint16_t min_values[SENSOR_COUNT] = {};
uint16_t max_values[SENSOR_COUNT] = {};
uint16_t calibrated_values[SENSOR_COUNT] = {};

FollowState follow_state = FollowState::Idle;

unsigned long calibration_start_ms = 0;
unsigned long state_start_ms = 0;
unsigned long last_control_ms = 0;
unsigned long last_line_status_ms = 0;

int32_t last_error = 0;

bool calibration_done = false;
bool line_detected = false;
bool run_enabled = true;

// =====================================================
// Motor section
// =====================================================

MotoronI2C mc;

const uint8_t MOTOR_LEFT  = 1;
const uint8_t MOTOR_RIGHT = 2;
const uint8_t MOTOR_AUX   = 3;

const double MOTOR_RATIO = 1.44 / 1.4681;
const long TURN_90_COUNTS = 1540;

const int16_t TURN_SPEED = 250;
constexpr int16_t BASE_SPEED = 250;
constexpr int16_t LOST_SEARCH_SPEED = 200;

const int16_t MAX_SPEED_R = 800;
const int16_t MAX_SPEED_L = 800;

constexpr float LINE_KP = 0.055f;

constexpr int8_t LEFT_FORWARD_SIGN = 1;
constexpr int8_t RIGHT_FORWARD_SIGN = 1;

const uint16_t ACCEL = 800;
const uint16_t DECEL = 800;

bool motoron_ready = false;

int16_t last_left_command = 0;
int16_t last_right_command = 0;

unsigned long lastPrintTime = 0;
unsigned long lastScanTime = 0;

// =====================================================
// Encoder section
// =====================================================

const uint8_t ENC_LEFT_A  = 28;
const uint8_t ENC_LEFT_B  = 26;
const uint8_t ENC_RIGHT_A = 22;
const uint8_t ENC_RIGHT_B = 24;

const double wheel_diameter = 38.35;

volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

// =====================================================
// LED and button section
// =====================================================

const int redPin = robot_config::TOP_RGB_RED_PIN;
const int greenPin = robot_config::TOP_RGB_GREEN_PIN;
const int bluePin = robot_config::TOP_RGB_BLUE_PIN;

const int OffButtonPin = robot_config::MECHANICAL_KILL_BUTTON_PIN;
const int RevButtonPin = robot_config::REVIVE_BUTTON_PIN;

int previousStateRed = 0;
bool running = true;
bool Reviving = false;
int previousOffButtonPressed = 1;

// =====================================================
// RFID section
// =====================================================

constexpr uint8_t RFID_ADDR = robot_config::RFID_I2C_ADDRESS;

MFRC522_I2C rfid(RFID_ADDR, robot_config::RFID_RESET_PIN, &Wire1);

// =====================================================
// Servo section
// =====================================================

constexpr int SERVO1_PIN = robot_config::SEED_UPPER_SERVO_PIN;
constexpr int SERVO2_PIN = robot_config::SEED_LOWER_SERVO_PIN;

constexpr int UPPER_CLOSED = robot_config::SEED_UPPER_CLOSED_ANGLE;
constexpr int UPPER_OPEN = robot_config::SEED_UPPER_OPEN_ANGLE;
constexpr int LOWER_CLOSED = robot_config::SEED_LOWER_CLOSED_ANGLE;
constexpr int LOWER_OPEN = robot_config::SEED_LOWER_OPEN_ANGLE;

constexpr int SERVO_MIN_PULSE_US = robot_config::SEED_SERVO_MIN_PULSE_US;
constexpr int SERVO_MAX_PULSE_US = robot_config::SEED_SERVO_MAX_PULSE_US;
constexpr unsigned long SERVO_PERIOD_US = robot_config::SEED_SERVO_PERIOD_US;

int upper_pulse_us = 1500;
int lower_pulse_us = 1500;

int seeds = 5;

unsigned long last_servo_frame_us = 0;

// =====================================================
// Helper functions
// =====================================================

void checkpoint(const char *message)
{
  // Serial.println(message);
  // Serial.flush();
}

// =====================================================
// Encoder functions
// =====================================================

void readLeftEncoder()
{
  if (digitalRead(ENC_LEFT_A) == digitalRead(ENC_LEFT_B)) {
    encoderLeftCount--;
  } else {
    encoderLeftCount++;
  }
}

void readRightEncoder()
{
  if (digitalRead(ENC_RIGHT_A) == digitalRead(ENC_RIGHT_B)) {
    encoderRightCount++;
  } else {
    encoderRightCount--;
  }
}

long getLeftEncoder()
{
  noInterrupts();
  long value = encoderLeftCount;
  interrupts();
  return value;
}

long getRightEncoder()
{
  noInterrupts();
  long value = encoderRightCount;
  interrupts();
  return value;
}

void printEncoders()
{
  const long left = getLeftEncoder();
  const long right = getRightEncoder();

  Serial.print("Left encoder: ");
  Serial.print(left);
  Serial.print(" | Right encoder: ");
  Serial.println(right);

  Serial.print("Error: ");
  Serial.println(left - right);

  Serial.print("Estimated distance traveled (L): ");
  Serial.println((left * M_PI * wheel_diameter) / (12 * 100));

  Serial.print("Estimated distance traveled (R): ");
  Serial.println((right * M_PI * wheel_diameter) / (12 * 100));
}

// =====================================================
// Motor functions
// =====================================================

void setMotors(int16_t leftSpeed, int16_t rightSpeed)
{
  leftSpeed  = constrain(leftSpeed,  -MAX_SPEED_L, MAX_SPEED_L);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED_R, MAX_SPEED_R);

  last_left_command = leftSpeed;
  last_right_command = rightSpeed;

  if (!motoron_ready) {
    return;
  }

  mc.setSpeed(MOTOR_LEFT, leftSpeed);
  mc.setSpeed(MOTOR_RIGHT, static_cast<int16_t>(rightSpeed * MOTOR_RATIO));
  mc.setSpeed(MOTOR_AUX, 0);

  if (mc.getLastError() != 0) {
    motoron_ready = false;
    Serial.print("motoron_error=");
    Serial.println(mc.getLastError());
  }
}

void stopMotors()
{
  setMotors(0, 0);
}

void configureMotor(uint8_t motor)
{
  mc.setMaxAcceleration(motor, ACCEL);
  mc.setMaxDeceleration(motor, DECEL);
}

void beginMotoron()
{
  checkpoint("Starting Wire1");

  Wire1.begin();
  Wire1.setClock(100000);

  mc.setBus(&Wire1);

  mc.reinitialize();
  delay(10);

  mc.disableCrc();
  delay(10);

  mc.clearResetFlag();
  mc.clearMotorFaultUnconditional();
  mc.setCommandTimeoutMilliseconds(2000);

  configureMotor(MOTOR_LEFT);
  configureMotor(MOTOR_RIGHT);
  configureMotor(MOTOR_AUX);

  motoron_ready = mc.getLastError() == 0;

  Serial.print("motoron_ready=");
  Serial.print(motoron_ready ? 1 : 0);
  Serial.print(" error=");
  Serial.println(mc.getLastError());

  stopMotors();
}

void drive_forward()
{
  setMotors(
    LEFT_FORWARD_SIGN * BASE_SPEED,
    RIGHT_FORWARD_SIGN * BASE_SPEED
  );
}

void search_for_line()
{
  if (last_error < 0) {
    setMotors(
      -LEFT_FORWARD_SIGN * LOST_SEARCH_SPEED,
      RIGHT_FORWARD_SIGN * LOST_SEARCH_SPEED
    );
  } else {
    setMotors(
      LEFT_FORWARD_SIGN * LOST_SEARCH_SPEED,
      -RIGHT_FORWARD_SIGN * LOST_SEARCH_SPEED
    );
  }
}

void follow_line(int32_t error)
{
  const int16_t correction = static_cast<int16_t>(LINE_KP * static_cast<float>(error));

  const int16_t left_speed = LEFT_FORWARD_SIGN * (BASE_SPEED + correction);
  const int16_t right_speed = RIGHT_FORWARD_SIGN * (BASE_SPEED - correction);

  setMotors(left_speed, right_speed);
}

// Forward declaration because centreAfterRFID uses it.
void serviceServoPulses();

void centreAfterRFID()
{
  int L_base = getLeftEncoder();
  int R_base = getRightEncoder();

  while ((getLeftEncoder() - L_base < 250) || (getRightEncoder() - R_base < 250)) {
    setMotors(BASE_SPEED, BASE_SPEED);
    serviceServoPulses();
  }

  stopMotors();
}

void centreAfterIR()
{
  int L_base = getLeftEncoder();
  int R_base = getRightEncoder();

  while ((getLeftEncoder() - L_base < 270) || (getRightEncoder() - R_base < 270)) {
    setMotors(BASE_SPEED, BASE_SPEED);
    serviceServoPulses();
  }

  stopMotors();
}

void turnLeft90()
{
  // 153*pi/4 = 38.35*theta/1200
  int L_base = getLeftEncoder();
  int R_base = getRightEncoder();

  //long theta = (M_PI * 153 * 1200)/(4*38.35);

  while ((getLeftEncoder() - L_base < TURN_90_COUNTS) || (getRightEncoder() - R_base > -TURN_90_COUNTS)) {
    setMotors(-TURN_SPEED, TURN_SPEED);
    printEncoders();
    serviceServoPulses();
  }

  stopMotors();
}

void turnRight90()
{
  // 153*pi/4 = 38.35*theta/1200
  int L_base = getLeftEncoder();
  int R_base = getRightEncoder();

  //long theta = (M_PI * 153 * 1200)/(4*38.35);

  while ((getLeftEncoder() - L_base > -TURN_90_COUNTS) || (getRightEncoder() - R_base < TURN_90_COUNTS)) {
    setMotors(TURN_SPEED, -TURN_SPEED);
    serviceServoPulses();
  }

  stopMotors();
}

// =====================================================
// LED functions
// =====================================================

void Flash(int previousStateRed)
{
  if (previousStateRed % 20 <= 9) {
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
  } else {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
  }
}

void Green()
{
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, LOW);
}

void Red()
{
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);
}

// =====================================================
// RFID functions
// =====================================================

bool rfid_firmware_version_valid(byte version)
{
  return version == 0x15 ||
         version == 0x90 ||
         version == 0x91 ||
         version == 0x92 ||
         version == 0xB2;
}

void print_hex2(byte value)
{
  if (value < 0x10) {
    Serial.print('0');
  }

  Serial.print(value, HEX);
}

void init_reader_without_soft_reset()
{
  rfid.PCD_WriteRegister(rfid.TModeReg, 0x80);
  rfid.PCD_WriteRegister(rfid.TPrescalerReg, 0xA9);
  rfid.PCD_WriteRegister(rfid.TReloadRegH, 0x03);
  rfid.PCD_WriteRegister(rfid.TReloadRegL, 0xE8);
  rfid.PCD_WriteRegister(rfid.TxASKReg, 0x40);
  rfid.PCD_WriteRegister(rfid.ModeReg, 0x3D);
  rfid.PCD_AntennaOn();
}

bool i2c_device_present(uint8_t address)
{
  Wire1.beginTransmission(address);
  return Wire1.endTransmission() == 0;
}

// =====================================================
// Servo functions
// =====================================================

int angleToPulseUs(int angle)
{
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
}

void serviceServoPulses()
{
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

void waitWithServo(unsigned long ms)
{
  const unsigned long start = millis();

  while (millis() - start < ms) {
    serviceServoPulses();
    delay(1);
  }
}

void writeUpperServo(int angle)
{
  upper_pulse_us = angleToPulseUs(angle);
}

void writeLowerServo(int angle)
{
  lower_pulse_us = angleToPulseUs(angle);
}

void closeBothGates()
{
  writeUpperServo(UPPER_CLOSED);
  writeLowerServo(LOWER_CLOSED);
}

void dispenseOne()
{
  Serial.println(F("Dispense cycle..."));

  writeUpperServo(UPPER_OPEN);
  waitWithServo(robot_config::SEED_UPPER_OPEN_MS);

  writeUpperServo(UPPER_CLOSED);
  waitWithServo(robot_config::SEED_UPPER_CLOSE_MS);

  writeLowerServo(LOWER_OPEN);
  waitWithServo(robot_config::SEED_LOWER_OPEN_MS);

  writeLowerServo(LOWER_CLOSED);
  waitWithServo(robot_config::SEED_LOWER_CLOSE_MS);

  Serial.println(F("Done."));
}

// =====================================================
// Line-following functions
// =====================================================

const char* state_name()
{
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

void read_rc_discharge_times()
{
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

void reset_calibration()
{
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    min_values[i] = robot_config::QTR_RC_TIMEOUT_US;
    max_values[i] = 0;
  }
}

void update_calibration()
{
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (raw_values[i] < min_values[i]) {
      min_values[i] = raw_values[i];
    }

    if (raw_values[i] > max_values[i]) {
      max_values[i] = raw_values[i];
    }
  }
}

void update_calibrated_values()
{
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint16_t range = max_values[i] - min_values[i];

    uint16_t value = 0;

    if (range > 0 && raw_values[i] > min_values[i]) {
      value = static_cast<uint16_t>(
        min<uint32_t>(
          1000,
          (static_cast<uint32_t>(raw_values[i] - min_values[i]) * 1000) / range
        )
      );
    }

    calibrated_values[i] = robot_config::QTR_LINE_IS_HIGH_RAW ? value : 1000 - value;
  }
}

uint16_t target_value_at(uint8_t sensor_index)
{
  return robot_config::QTR_FOLLOW_BLACK_LINE
           ? calibrated_values[sensor_index]
           : 1000 - calibrated_values[sensor_index];
}

uint16_t line_peak()
{
  uint16_t peak = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint16_t value = target_value_at(i);

    if (value > peak) {
      peak = value;
    }
  }

  return peak;
}

bool update_line_found()
{
  const uint16_t peak = line_peak();

  if (line_detected) {
    line_detected = peak >= robot_config::QTR_LINE_DETECT_OFF_THRESHOLD;
  } else {
    line_detected = peak >= robot_config::QTR_LINE_DETECT_ON_THRESHOLD;
  }

  return line_detected;
}

bool intersection_detected()
{
  uint8_t high_count = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (target_value_at(i) >= INTERSECTION_THRESHOLD) {
      ++high_count;
    }
  }

  return high_count >= INTERSECTION_SENSOR_COUNT;
}

int32_t estimate_line_position()
{
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

void set_follow_state(FollowState next_state)
{
  if (follow_state == next_state) {
    return;
  }

  follow_state = next_state;
  state_start_ms = millis();
}

void update_line_following()
{
  update_calibrated_values();

  const bool found = update_line_found();
  const bool intersection = found && intersection_detected();

  const int32_t position = found ? estimate_line_position() : -1;
  const int32_t error = position >= 0 ? position - LINE_CENTER : last_error;

  if (!run_enabled) {
    set_follow_state(FollowState::Idle);
    stopMotors();
    return;
  }

  if (!found) {
    set_follow_state(FollowState::LostLine);

    centreAfterIR();

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

void print_array(const char* label, const uint16_t* values)
{
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

void print_line_status()
{
  const bool found = line_detected;
  const int32_t position = found ? estimate_line_position() : -1;
  const int32_t error = position >= 0 ? position - LINE_CENTER : last_error;

  Serial.print("running=");
  Serial.print(running ? 1 : 0);

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

void print_calibration()
{
  Serial.println("Calibration complete.");

  print_array("min", min_values);
  Serial.print(' ');
  print_array("max", max_values);
  Serial.println();

  Serial.println("Line following ready.");
}

// =====================================================
// Robot behaviour
// =====================================================

void plant()
{
  closeBothGates();

  centreAfterRFID();

  stopMotors();
  delay(50);

  dispenseOne();

  delay(50);
}

// =====================================================
// Setup
// =====================================================

void setup()
{
  Serial.begin(115200);

  unsigned long startTime = millis();

  while (!Serial && millis() - startTime < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }

  delay(1000);

  Serial.println();
  Serial.println("=== GIGA R1 merged line-following + RFID + seed dispenser robot ===");

  beginMotoron();

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), readRightEncoder, CHANGE);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(OffButtonPin, INPUT_PULLUP);
  pinMode(RevButtonPin, INPUT_PULLUP);

  if (i2c_device_present(RFID_ADDR)) {
    Serial.println("RFID found at 0x28 on SDA1/SCL1");
  } else {
    Serial.println("No I2C device found at 0x28 on SDA1/SCL1");
    Serial.println("Check VCC, GND, SDA1, and SCL1 wiring");
    return;
  }

  byte version = rfid.PCD_ReadRegister(rfid.VersionReg);

  if (rfid_firmware_version_valid(version)) {
    Serial.print("RFID firmware version OK: 0x");
    print_hex2(version);
    Serial.println();
  } else {
    Serial.print("Device responded, but firmware version is not recognised: 0x");
    print_hex2(version);
    Serial.println();
    return;
  }

  init_reader_without_soft_reset();

  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);

  digitalWrite(SERVO1_PIN, LOW);
  digitalWrite(SERVO2_PIN, LOW);

  closeBothGates();
  waitWithServo(500);

  reset_calibration();

  calibration_start_ms = millis();
  last_control_ms = millis();
  last_line_status_ms = millis();

  Serial.println("Move sensors over floor and line for 5 seconds to calibrate.");
  Serial.println("Then robot will begin line following automatically if running is enabled.");
}

// =====================================================
// Loop
// =====================================================

void loop()
{
  serviceServoPulses();

  int OffButtonPressed = digitalRead(OffButtonPin);
  int RevButtonPressed = digitalRead(RevButtonPin);

  Reviving = false;

  if (OffButtonPressed == LOW && previousOffButtonPressed != LOW) {
    running = !running;
    delay(50);
  } else if (RevButtonPressed == LOW) {
    Reviving = true;
    delay(50);
  }

  previousOffButtonPressed = OffButtonPressed;

  read_rc_discharge_times();

  const unsigned long now_ms = millis();

  if (!calibration_done) {
    update_calibration();
    stopMotors();

    if (now_ms - calibration_start_ms >= CALIBRATION_TIME_MS) {
      calibration_done = true;
      print_calibration();
    }

    return;
  }

  if (!running) {
    run_enabled = false;
    Flash(previousStateRed);
    previousStateRed++;
    stopMotors();
    delay(25);
    return;
  }

  run_enabled = true;

  if (Reviving) {
    Green();
    stopMotors();
    delay(500);
    return;
  }

  if (rfid.PICC_IsNewCardPresent() &&
      rfid.PICC_ReadCardSerial() &&
      millis() - lastScanTime >= RFID_SCAN_COOLDOWN_MS) {

    Serial.print("UID: ");

    for (byte i = 0; i < rfid.uid.size; i++) {
      print_hex2(rfid.uid.uidByte[i]);
      Serial.print(' ');
    }

    Serial.println();

    stopMotors();
    delay(100);

    rfid.PICC_HaltA();

    if (seeds > 0) {
      plant();
      lastScanTime = millis();
      seeds--;
    }

    return;
  }

  Red();

  if (now_ms - last_control_ms >= CONTROL_INTERVAL_MS) {
    last_control_ms = now_ms;
    update_line_following();
  }

  if (now_ms - last_line_status_ms >= LINE_STATUS_INTERVAL_MS) {
    last_line_status_ms = now_ms;
    print_line_status();
  }

  if (millis() - lastPrintTime >= 1000) {
    lastPrintTime = millis();
    // printEncoders();
  }
}