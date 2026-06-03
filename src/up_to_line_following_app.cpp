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

/*byte arenaMap = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                   {1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1},
                   {1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1},
                   {1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1},
                   {1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1},
                   {1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1},
                   [1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1},
                   {1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1},
                   {1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1},
                   {1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1},
                   {1, 4, 1, 1, 1, 1, 1, 1, 0, 5, 1}};*/

// =====================================================
// State
// =====================================================

enum class FollowState {
  Idle,
  FollowLine,
  CrossIntersection,
  LostLine,
};

enum class JunctionType {
  None,
  Straight,
  LeftTurn,
  RightTurn,
  TIntersection,
  WideIntersection,
  Lost
};

enum class JunctionDecision {
  Straight,
  Left,
  Right,
};

constexpr JunctionDecision T_INTERSECTION_DECISION = JunctionDecision::Right;
constexpr JunctionDecision WIDE_INTERSECTION_DECISION = JunctionDecision::Right;

JunctionType last_junction_type = JunctionType::None;
unsigned long last_junction_detect_ms = 0;

// Forward declarations for Arduino .ino auto-prototype safety
const char* junctionTypeName(JunctionType type);
bool sensor_active(uint8_t index);
uint8_t count_active_range(uint8_t start_index, uint8_t end_index);
JunctionType detect_junction_type();
uint16_t line_peak();
uint16_t target_value_at(uint8_t sensor_index);
int32_t estimate_line_position();
void read_rc_discharge_times();
void update_calibrated_values();
void set_follow_state(FollowState next_state);
void follow_line(int32_t error);
void reset_turn_angle();
void update_turn_angle();
bool imuTurnDegrees(float target_degrees, int8_t direction);
void serviceServoPulses();
void centreAfterRFID();
void print_hex2(byte value);
bool lineCenteredForTurn();

enum class StartupCalState {
  CalibratingIR,
  WaitingForStillIMU,
  CalibratingIMU,
  Ready
};

StartupCalState startup_cal_state = StartupCalState::CalibratingIR;
bool startup_blocked = false;

constexpr unsigned long IMU_STILL_WAIT_MS = 3000;
unsigned long imu_still_start_ms = 0;

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
// IMU section - DFRobot SEN0140 / ITG320x gyro support
// =====================================================

constexpr unsigned long IMU_PRINT_INTERVAL_MS = 250;
constexpr unsigned long TURN_TIMEOUT_MS = 8000;
constexpr uint16_t GYRO_BIAS_SAMPLE_COUNT = 200;
constexpr unsigned long GYRO_BIAS_SAMPLE_DELAY_MS = 5;

constexpr float ITG320X_LSB_PER_DPS = 14.375f;
constexpr float GYRO_Z_DEADBAND_DPS = 0.4f;

constexpr float TURN_90_TARGET_DEG = 85.0f;//degree
constexpr float TURN_180_TARGET_DEG = 180.0f;
constexpr float CORNER_IMU_APPROACH_DEG = 70.0f;
constexpr float TURN_LINE_SETTLE_DEG = 12.0f;
constexpr uint8_t CORNER_CENTER_STABLE_SAMPLES = 4;
constexpr unsigned long CORNER_IGNORE_LINE_MS = 250;
constexpr unsigned long CORNER_FIND_LINE_TIMEOUT_MS = 5000;

constexpr uint8_t ADXL345_ADDRESS = 0x53;
constexpr uint8_t ADXL345_DEVID = 0x00;
constexpr uint8_t ADXL345_POWER_CTL = 0x2D;
constexpr uint8_t ADXL345_DATA_FORMAT = 0x31;

constexpr uint8_t ITG320X_ADDRESS = 0x68;
constexpr uint8_t ITG320X_WHO_AM_I = 0x00;
constexpr uint8_t ITG320X_SMPLRT_DIV = 0x15;
constexpr uint8_t ITG320X_DLPF_FS = 0x16;
constexpr uint8_t ITG320X_TEMP_OUT_H = 0x1B;
constexpr uint8_t ITG320X_PWR_MGM = 0x3E;

constexpr uint8_t HMC5883L_ADDRESS = 0x1E;
constexpr uint8_t HMC5883L_CONFIG_A = 0x00;
constexpr uint8_t HMC5883L_CONFIG_B = 0x01;
constexpr uint8_t HMC5883L_MODE = 0x02;
constexpr uint8_t HMC5883L_ID_A = 0x0A;

constexpr uint8_t QMC5883L_ADDRESS = 0x0D;
constexpr uint8_t QMC5883L_CONTROL_1 = 0x09;
constexpr uint8_t QMC5883L_SET_RESET = 0x0B;

constexpr uint8_t BMP280_ADDRESS_1 = 0x76;
constexpr uint8_t BMP280_ADDRESS_2 = 0x77;
constexpr uint8_t BMP280_CHIP_ID = 0xD0;
constexpr uint8_t BMP280_RESET = 0xE0;
constexpr uint8_t BMP280_CTRL_MEAS = 0xF4;
constexpr uint8_t BMP280_CONFIG = 0xF5;

TwoWire* imu_bus = &Wire;
const char* imu_bus_name = "Wire D20/D21";

bool adxl345_ready = false;
bool itg320x_ready = false;
bool hmc5883l_ready = false;
bool bmp280_ready = false;
bool compass_is_qmc5883 = false;

uint8_t bmp280_address = BMP280_ADDRESS_1;

float gyro_z_bias_dps = 0.0f;
float turn_angle_deg = 0.0f;

unsigned long last_gyro_update_us = 0;
unsigned long turn_start_ms = 0;
unsigned long last_imu_print_ms = 0;

struct Axis3 {
  int16_t x;
  int16_t y;
  int16_t z;
};

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
const long TURN_180_COUNTS = 3070;

const int16_t TURN_SPEED = 250;
const int16_t TURN_ALIGN_SPEED = 150;
constexpr int16_t BASE_SPEED = 250;
constexpr int16_t LOST_SEARCH_SPEED = 200;

constexpr bool ENABLE_OPEN_FIELD_TEST4 = true;
constexpr long OPEN_FIELD_COUNTS_PER_NODE = 2600L;
constexpr float OPEN_FIELD_FIRST_STRAIGHT_NODES = 2.0f;
constexpr float OPEN_FIELD_SIDE_STRAIGHT_NODES = 1.0f;
constexpr float OPEN_FIELD_FINAL_STRAIGHT_NODES = 2.0f;
constexpr int16_t OPEN_FIELD_BASE_SPEED = 280;
constexpr float OPEN_FIELD_ENCODER_KP = 0.08f;
constexpr float OPEN_FIELD_HEADING_KP = 0.0f;
constexpr int16_t OPEN_FIELD_MAX_CORRECTION = 80;
constexpr unsigned long OPEN_FIELD_TIMEOUT_PER_NODE_MS = 4500;
constexpr bool OPEN_FIELD_USE_RFID_CENTERING = true;
constexpr bool OPEN_FIELD_REQUIRE_TARGET_RFID = false;
constexpr int16_t OPEN_FIELD_RFID_SEARCH_SPEED = 220;
constexpr long OPEN_FIELD_RFID_IGNORE_START_COUNTS = OPEN_FIELD_COUNTS_PER_NODE / 6L;
constexpr long OPEN_FIELD_RFID_MIN_GAP_COUNTS = (OPEN_FIELD_COUNTS_PER_NODE * 2L) / 5L;
constexpr long OPEN_FIELD_RFID_FALLBACK_EXTRA_COUNTS = OPEN_FIELD_COUNTS_PER_NODE;
constexpr unsigned long OPEN_FIELD_RFID_SCAN_COOLDOWN_MS = 200;

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
bool open_field_test4_done = false;
bool open_field_test4_running = false;

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

bool motionAbortRequested()
{
  if (digitalRead(OffButtonPin) != LOW) {
    return false;
  }

  running = false;
  run_enabled = false;
  stopMotors();
  Serial.println("motion_abort=button");
  return true;
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

long absLong(long value)
{
  return value < 0 ? -value : value;
}

long averageTravelCounts(long left_base, long right_base)
{
  const long left_delta = absLong(getLeftEncoder() - left_base);
  const long right_delta = absLong(getRightEncoder() - right_base);

  return (left_delta + right_delta) / 2;
}

bool readOpenFieldRfidNode()
{
  if (!OPEN_FIELD_USE_RFID_CENTERING) {
    return false;
  }

  if (millis() - lastScanTime < OPEN_FIELD_RFID_SCAN_COOLDOWN_MS) {
    return false;
  }

  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return false;
  }

  Serial.print("open_field_rfid_uid=");

  for (byte i = 0; i < rfid.uid.size; ++i) {
    print_hex2(rfid.uid.uidByte[i]);
    Serial.print(' ');
  }

  Serial.println();

  rfid.PICC_HaltA();
  lastScanTime = millis();

  return true;
}

bool driveStraightCounts(long target_counts, uint8_t target_rfid_nodes, const char* label)
{
  if (!motoron_ready) {
    Serial.println("open_field_error=motoron_not_ready");
    return false;
  }

  if (target_counts <= 0) {
    return true;
  }

  const long left_base = getLeftEncoder();
  const long right_base = getRightEncoder();
  const unsigned long start_ms = millis();
  const long max_counts =
    OPEN_FIELD_USE_RFID_CENTERING && target_rfid_nodes > 0
      ? target_counts + OPEN_FIELD_RFID_FALLBACK_EXTRA_COUNTS
      : target_counts;
  const unsigned long timeout_ms =
    static_cast<unsigned long>(
      (static_cast<float>(max_counts) / static_cast<float>(OPEN_FIELD_COUNTS_PER_NODE)) *
      OPEN_FIELD_TIMEOUT_PER_NODE_MS
    ) + 1500;

  unsigned long last_status_ms = 0;
  uint8_t rfid_nodes_seen = 0;
  long last_rfid_moved_counts = 0;

  reset_turn_angle();

  Serial.print("open_field_drive_start=");
  Serial.print(label);
  Serial.print(" target_counts=");
  Serial.print(target_counts);
  Serial.print(" target_rfid_nodes=");
  Serial.println(target_rfid_nodes);

  while (averageTravelCounts(left_base, right_base) < max_counts) {
    serviceServoPulses();
    update_turn_angle();

    if (motionAbortRequested()) {
      return false;
    }

    const long left_delta = absLong(getLeftEncoder() - left_base);
    const long right_delta = absLong(getRightEncoder() - right_base);
    const long moved_counts = (left_delta + right_delta) / 2;
    const long encoder_error = left_delta - right_delta;
    const bool searching_for_rfid =
      OPEN_FIELD_USE_RFID_CENTERING &&
      target_rfid_nodes > 0 &&
      rfid_nodes_seen < target_rfid_nodes &&
      moved_counts >= target_counts;
    const int16_t drive_speed =
      searching_for_rfid ? OPEN_FIELD_RFID_SEARCH_SPEED : OPEN_FIELD_BASE_SPEED;

    const int16_t correction = constrain(
      static_cast<int16_t>(
        (OPEN_FIELD_ENCODER_KP * static_cast<float>(encoder_error)) +
        (OPEN_FIELD_HEADING_KP * turn_angle_deg)
      ),
      -OPEN_FIELD_MAX_CORRECTION,
      OPEN_FIELD_MAX_CORRECTION
    );

    setMotors(
      LEFT_FORWARD_SIGN * (drive_speed - correction),
      RIGHT_FORWARD_SIGN * (drive_speed + correction)
    );

    if (OPEN_FIELD_USE_RFID_CENTERING &&
        target_rfid_nodes > 0 &&
        moved_counts >= OPEN_FIELD_RFID_IGNORE_START_COUNTS &&
        moved_counts - last_rfid_moved_counts >= OPEN_FIELD_RFID_MIN_GAP_COUNTS &&
        readOpenFieldRfidNode()) {
      stopMotors();

      ++rfid_nodes_seen;

      Serial.print("open_field_rfid_node=");
      Serial.print(label);
      Serial.print(" seen=");
      Serial.print(rfid_nodes_seen);
      Serial.print("/");
      Serial.println(target_rfid_nodes);

      centreAfterRFID();
      reset_turn_angle();
      last_rfid_moved_counts = averageTravelCounts(left_base, right_base);

      if (rfid_nodes_seen >= target_rfid_nodes) {
        stopMotors();
        Serial.print("open_field_drive_done=");
        Serial.print(label);
        Serial.print(" reason=rfid moved=");
        Serial.println(averageTravelCounts(left_base, right_base));
        delay(100);
        return true;
      }
    }

    const unsigned long now_ms = millis();

    if (now_ms - last_status_ms >= 500) {
      last_status_ms = now_ms;

      Serial.print("open_field_drive_progress=");
      Serial.print(label);
      Serial.print(" moved=");
      Serial.print(moved_counts);
      Serial.print(" left=");
      Serial.print(left_delta);
      Serial.print(" right=");
      Serial.print(right_delta);
      Serial.print(" rfid=");
      Serial.print(rfid_nodes_seen);
      Serial.print("/");
      Serial.print(target_rfid_nodes);
      Serial.print(" search=");
      Serial.print(searching_for_rfid ? 1 : 0);
      Serial.print(" corr=");
      Serial.println(correction);
    }

    if (now_ms - start_ms >= timeout_ms) {
      stopMotors();
      Serial.print("open_field_drive_timeout=");
      Serial.println(label);
      return false;
    }
  }

  stopMotors();

  Serial.print("open_field_drive_done=");
  Serial.print(label);
  Serial.print(" reason=counts");
  Serial.print(" moved=");
  Serial.print(averageTravelCounts(left_base, right_base));
  Serial.print(" rfid=");
  Serial.print(rfid_nodes_seen);
  Serial.print("/");
  Serial.println(target_rfid_nodes);

  delay(100);

  if (OPEN_FIELD_REQUIRE_TARGET_RFID &&
      OPEN_FIELD_USE_RFID_CENTERING &&
      target_rfid_nodes > 0 &&
      rfid_nodes_seen < target_rfid_nodes) {
    Serial.print("open_field_drive_failed_missing_rfid=");
    Serial.println(label);
    return false;
  }

  return true;
}

bool driveStraightNodes(float nodes, const char* label)
{
  const long target_counts =
    static_cast<long>((nodes * static_cast<float>(OPEN_FIELD_COUNTS_PER_NODE)) + 0.5f);
  const uint8_t target_rfid_nodes = static_cast<uint8_t>(nodes + 0.5f);

  return driveStraightCounts(target_counts, target_rfid_nodes, label);
}

void runOpenFieldTest4()
{
  open_field_test4_running = true;
  set_follow_state(FollowState::Idle);

  Serial.println("open_field_test4_start");
  Serial.print("open_field_counts_per_node=");
  Serial.println(OPEN_FIELD_COUNTS_PER_NODE);

  bool ok = true;

  ok = ok && driveStraightNodes(OPEN_FIELD_FIRST_STRAIGHT_NODES, "leg1");
  ok = ok && imuTurnDegrees(TURN_90_TARGET_DEG, -1);
  ok = ok && driveStraightNodes(OPEN_FIELD_SIDE_STRAIGHT_NODES, "leg2");
  ok = ok && imuTurnDegrees(TURN_90_TARGET_DEG, 1);
  ok = ok && driveStraightNodes(OPEN_FIELD_FINAL_STRAIGHT_NODES, "leg3");

  stopMotors();

  open_field_test4_done = true;
  open_field_test4_running = false;

  Serial.print("open_field_test4_result=");
  Serial.println(ok ? "done" : "failed_or_aborted");
}

// Forward declaration because centreAfterRFID uses it.
void serviceServoPulses();

void centreAfterRFID()
{
  int L_base = getLeftEncoder();
  int R_base = getRightEncoder();
  const unsigned long start_ms = millis();

  while ((getLeftEncoder() - L_base < 250) || (getRightEncoder() - R_base < 250)) {
    setMotors(BASE_SPEED, BASE_SPEED);
    serviceServoPulses();
    if (millis() - start_ms > 1500) {
      Serial.println("centre_after_rfid_timeout");
      break;
    }
  }

  stopMotors();
}

void centreAfterIR()
{
  int L_base = getLeftEncoder();
  int R_base = getRightEncoder();
  const unsigned long start_ms = millis();

  while ((getLeftEncoder() - L_base < 300) || (getRightEncoder() - R_base < 300)) {
    setMotors(BASE_SPEED, BASE_SPEED);
    serviceServoPulses();
    if (millis() - start_ms > 1500) {
      Serial.println("centre_after_ir_timeout");
      break;
    }
  }

  stopMotors();
}

// =====================================================
// IMU-based turning functions
// =====================================================

bool imuTurnDegrees(float target_degrees, int8_t direction)
{
  if (!motoron_ready) {
    Serial.println("turn_error=motoron_not_ready");
    return false;
  }

  if (!itg320x_ready) {
    Serial.println("turn_error=gyro_not_ready");
    return false;
  }

  direction = direction >= 0 ? 1 : -1;

  reset_turn_angle();

  const unsigned long start_ms = millis();

  Serial.print("imu_turn_start direction=");
  Serial.print(direction > 0 ? "left" : "right");
  Serial.print(" target=");
  Serial.println(target_degrees, 1);

  while (fabsf(turn_angle_deg) < target_degrees) {
    serviceServoPulses();
    update_turn_angle();

    if (motionAbortRequested()) {
      return false;
    }

    if (direction > 0) {
      setMotors(-LEFT_FORWARD_SIGN * TURN_SPEED,
                 RIGHT_FORWARD_SIGN * TURN_SPEED);
    } else {
      setMotors( LEFT_FORWARD_SIGN * TURN_SPEED,
                -RIGHT_FORWARD_SIGN * TURN_SPEED);
    }

    if (millis() - start_ms >= TURN_TIMEOUT_MS) {
      Serial.print("turn_timeout angle=");
      Serial.println(turn_angle_deg, 1);
      stopMotors();
      return false;
    }
  }

  stopMotors();

  Serial.print("imu_turn_done angle=");
  Serial.println(turn_angle_deg, 1);

  delay(100);

  return true;
}

void turnLeft90()
{
  imuTurnDegrees(TURN_90_TARGET_DEG, 1);
}

void turnRight90()
{
  imuTurnDegrees(TURN_90_TARGET_DEG, -1);
}

void turnLeft180()
{
  imuTurnDegrees(TURN_180_TARGET_DEG, 1);
}

void turnRight180()
{
  imuTurnDegrees(TURN_180_TARGET_DEG, -1);
}

bool slowTurnUntilCenteredLine(int8_t direction, unsigned long timeout_ms)
{
  direction = direction >= 0 ? 1 : -1;

  const unsigned long start_ms = millis();
  uint8_t centered_samples = 0;

  while (millis() - start_ms < timeout_ms) {
    serviceServoPulses();

    if (direction > 0) {
      setMotors(-LEFT_FORWARD_SIGN * TURN_ALIGN_SPEED,
                 RIGHT_FORWARD_SIGN * TURN_ALIGN_SPEED);
    } else {
      setMotors( LEFT_FORWARD_SIGN * TURN_ALIGN_SPEED,
                -RIGHT_FORWARD_SIGN * TURN_ALIGN_SPEED);
    }

    read_rc_discharge_times();
    update_calibrated_values();

    const bool can_accept_line = millis() - start_ms >= CORNER_IGNORE_LINE_MS;

    if (can_accept_line && lineCenteredForTurn()) {
      centered_samples++;
    } else {
      centered_samples = 0;
    }

    if (centered_samples >= CORNER_CENTER_STABLE_SAMPLES) {
      stopMotors();
      last_error = 0;
      line_detected = true;
      set_follow_state(FollowState::FollowLine);
      follow_line(0);
      Serial.println("corner_turn_result=line_centered");
      return true;
    }
  }

  stopMotors();
  line_detected = false;
  set_follow_state(FollowState::LostLine);
  Serial.println("corner_turn_result=line_not_found");
  return false;
}

bool cornerTurnToLine(int8_t direction)
{
  direction = direction >= 0 ? 1 : -1;

  Serial.print("corner_qtr_turn_start=");
  Serial.println(direction > 0 ? "left" : "right");

  centreAfterIR();

  return slowTurnUntilCenteredLine(direction, CORNER_FIND_LINE_TIMEOUT_MS);
}

void driveStraightThroughJunction()
{
  centreAfterIR();
  set_follow_state(FollowState::CrossIntersection);
  drive_forward();
}

void handleTIntersectionDecision(JunctionDecision decision)
{
  if (decision == JunctionDecision::Left) {
    cornerTurnToLine(1);
    return;
  }

  if (decision == JunctionDecision::Right) {
    cornerTurnToLine(-1);
    return;
  }

  driveStraightThroughJunction();
}

// =====================================================
// Line-detecting turn functions
// =====================================================

const long TURN_360_COUNTS = TURN_180_COUNTS * 2L;

constexpr int32_t TURN_LINE_CENTER_TOLERANCE = 350;
constexpr uint16_t TURN_LINE_MIN_PEAK = robot_config::QTR_LINE_DETECT_ON_THRESHOLD;
constexpr uint8_t TURN_CENTER_STABLE_SAMPLES = 4;
constexpr unsigned long TURN_WITH_LINE_TIMEOUT_MS = 8000;

// Ignore the starting line for part of the turn.
// This prevents the robot from instantly detecting the line it is already on.
const long TURN_90_ACCEPT_AFTER_COUNTS = TURN_90_COUNTS / 2;
const long TURN_180_ACCEPT_AFTER_COUNTS = TURN_180_COUNTS - (TURN_90_COUNTS / 2);

long averageTurnMovementCounts(long left_base, long right_base)
{
  const long left_delta = abs(getLeftEncoder() - left_base);
  const long right_delta = abs(getRightEncoder() - right_base);

  return (left_delta + right_delta) / 2;
}

bool lineCenteredForTurn()
{
  const uint16_t peak = line_peak();

  if (peak < TURN_LINE_MIN_PEAK) {
    return false;
  }

  const int32_t position = estimate_line_position();

  if (position < 0) {
    return false;
  }

  const int32_t error = position - LINE_CENTER;

  if (abs(error) > TURN_LINE_CENTER_TOLERANCE) {
    return false;
  }

  const uint16_t center_value = target_value_at(4);

  uint16_t max_other_value = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i == 4) {
      continue;
    }

    const uint16_t value = target_value_at(i);

    if (value > max_other_value) {
      max_other_value = value;
    }
  }

  // The middle sensor should be the strongest, or effectively tied with the strongest.
  // The small tolerance avoids rejecting a good line because of sensor noise.
  return center_value + 50 >= max_other_value;
}

bool turnUntilCenteredLine(int8_t direction, long accept_after_counts)
{
  direction = direction >= 0 ? 1 : -1;

  const long left_base = getLeftEncoder();
  const long right_base = getRightEncoder();

  const unsigned long start_ms = millis();

  uint8_t centered_samples = 0;

  Serial.print("turn_with_lines_start=");
  Serial.println(direction > 0 ? "left" : "right");

  while (averageTurnMovementCounts(left_base, right_base) < TURN_360_COUNTS) {
    serviceServoPulses();

    if (direction > 0) {
      setMotors(-TURN_SPEED, TURN_SPEED);
    } else {
      setMotors(TURN_SPEED, -TURN_SPEED);
    }

    read_rc_discharge_times();
    update_calibrated_values();

    const long moved_counts = averageTurnMovementCounts(left_base, right_base);

    if (moved_counts >= accept_after_counts && lineCenteredForTurn()) {
      centered_samples++;
    } else {
      centered_samples = 0;
    }

    if (centered_samples >= TURN_CENTER_STABLE_SAMPLES) {
      stopMotors();

      last_error = 0;
      line_detected = true;
      set_follow_state(FollowState::FollowLine);

      Serial.println("turn_with_lines_result=line_centered");

      // Hand control back to the normal line-following correction.
      // If you later rename follow_line() to follow_line_pid(), call that here instead.
      follow_line(0);

      return true;
    }

    if (millis() - start_ms >= TURN_WITH_LINE_TIMEOUT_MS) {
      break;
    }
  }

  stopMotors();

  line_detected = false;
  set_follow_state(FollowState::LostLine);

  Serial.println("turn_with_lines_result=line_not_found_lost");

  return false;
}

void turnLeft90WithLines()
{
  turnUntilCenteredLine(1, TURN_90_ACCEPT_AFTER_COUNTS);
}

void turnRight90WithLines()
{
  turnUntilCenteredLine(-1, TURN_90_ACCEPT_AFTER_COUNTS);
}

void turnLeft180WithLines()
{
  turnUntilCenteredLine(1, TURN_180_ACCEPT_AFTER_COUNTS);
}

void turnRight180WithLines()
{
  turnUntilCenteredLine(-1, TURN_180_ACCEPT_AFTER_COUNTS);
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

void Yellow()
{
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, LOW);
}

void Blue()
{
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, HIGH);
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
// IMU helper functions
// =====================================================

/*void processIMUSerialCommands()
{
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());

    if (command == 'c' || command == 'C') {
      stopMotors();
      calibrate_gyro_z_bias();
    } else if (command == 'z' || command == 'Z') {
      reset_turn_angle();
      Serial.println("turn_angle_reset=1");
    } else if (command == 'l' || command == 'L') {
      turnLeft90();
    } else if (command == 'r' || command == 'R') {
      turnRight90();
    }
  }
}*/

bool imu_write_register(TwoWire& bus, uint8_t address, uint8_t reg, uint8_t value)
{
  bus.beginTransmission(address);
  bus.write(reg);
  bus.write(value);
  return bus.endTransmission() == 0;
}

bool imu_read_registers(
  TwoWire& bus,
  uint8_t address,
  uint8_t start_reg,
  uint8_t* buffer,
  uint8_t length)
{
  bus.beginTransmission(address);
  bus.write(start_reg);

  if (bus.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t received = bus.requestFrom(address, length);

  if (received != length) {
    while (bus.available() > 0) {
      bus.read();
    }

    return false;
  }

  for (uint8_t i = 0; i < length; ++i) {
    buffer[i] = static_cast<uint8_t>(bus.read());
  }

  return true;
}

bool imu_read_u8(TwoWire& bus, uint8_t address, uint8_t reg, uint8_t& value)
{
  return imu_read_registers(bus, address, reg, &value, 1);
}

bool imu_probe_address(TwoWire& bus, uint8_t address)
{
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

void imu_print_hex_byte(uint8_t value)
{
  if (value < 0x10) {
    Serial.print('0');
  }

  Serial.print(value, HEX);
}

uint8_t imu_count_expected_devices(TwoWire& bus)
{
  uint8_t count = 0;

  const uint8_t expected_addresses[] = {
    HMC5883L_ADDRESS,
    QMC5883L_ADDRESS,
    ADXL345_ADDRESS,
    ITG320X_ADDRESS,
    BMP280_ADDRESS_1,
    BMP280_ADDRESS_2,
  };

  for (const uint8_t address : expected_addresses) {
    if (imu_probe_address(bus, address)) {
      ++count;
    }
  }

  return count;
}

void select_best_imu_i2c_bus()
{
  imu_bus = &Wire;
  imu_bus_name = "Wire D20/D21";

  uint8_t best_score = imu_count_expected_devices(Wire);

  const uint8_t wire1_score = imu_count_expected_devices(Wire1);

  if (wire1_score > best_score) {
    imu_bus = &Wire1;
    imu_bus_name = "Wire1 SDA1/SCL1";
    best_score = wire1_score;
  }

#if WIRE_HOWMANY > 2
  const uint8_t wire2_score = imu_count_expected_devices(Wire2);

  if (wire2_score > best_score) {
    imu_bus = &Wire2;
    imu_bus_name = "Wire2 D9/D8";
    best_score = wire2_score;
  }
#endif

  Serial.print("IMU bus=");
  Serial.print(imu_bus_name);
  Serial.print(" expected_devices=");
  Serial.println(best_score);
}

bool setup_adxl345()
{
  uint8_t device_id = 0;

  if (!imu_read_u8(*imu_bus, ADXL345_ADDRESS, ADXL345_DEVID, device_id)) {
    Serial.println("ADXL345: not found");
    return false;
  }

  Serial.print("ADXL345 id=0x");
  imu_print_hex_byte(device_id);
  Serial.println(device_id == 0xE5 ? " ok" : " unexpected");

  return imu_write_register(*imu_bus, ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 0x08) &&
         imu_write_register(*imu_bus, ADXL345_ADDRESS, ADXL345_POWER_CTL, 0x08);
}

bool setup_itg320x()
{
  uint8_t who_am_i = 0;

  if (!imu_read_u8(*imu_bus, ITG320X_ADDRESS, ITG320X_WHO_AM_I, who_am_i)) {
    Serial.println("ITG320x gyro: not found");
    return false;
  }

  Serial.print("ITG320x gyro whoami=0x");
  imu_print_hex_byte(who_am_i);
  Serial.println();

  return imu_write_register(*imu_bus, ITG320X_ADDRESS, ITG320X_PWR_MGM, 0x00) &&
         imu_write_register(*imu_bus, ITG320X_ADDRESS, ITG320X_SMPLRT_DIV, 0x07) &&
         imu_write_register(*imu_bus, ITG320X_ADDRESS, ITG320X_DLPF_FS, 0x1E);
}

bool setup_hmc5883l()
{
  uint8_t id[3] = {};

  if (imu_read_registers(*imu_bus, HMC5883L_ADDRESS, HMC5883L_ID_A, id, sizeof(id))) {
    compass_is_qmc5883 = false;

    Serial.print("HMC5883L compass id=");
    Serial.write(id[0]);
    Serial.write(id[1]);
    Serial.write(id[2]);
    Serial.println();

    return imu_write_register(*imu_bus, HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x70) &&
           imu_write_register(*imu_bus, HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0x20) &&
           imu_write_register(*imu_bus, HMC5883L_ADDRESS, HMC5883L_MODE, 0x00);
  }

  if (imu_probe_address(*imu_bus, QMC5883L_ADDRESS)) {
    compass_is_qmc5883 = true;

    Serial.println("QMC5883L compass found");

    return imu_write_register(*imu_bus, QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01) &&
           imu_write_register(*imu_bus, QMC5883L_ADDRESS, QMC5883L_CONTROL_1, 0x1D);
  }

  Serial.println("Compass: not found");
  return false;
}

bool setup_bmp280()
{
  const uint8_t candidates[] = {BMP280_ADDRESS_1, BMP280_ADDRESS_2};

  for (const uint8_t address : candidates) {
    uint8_t chip_id = 0;

    if (imu_read_u8(*imu_bus, address, BMP280_CHIP_ID, chip_id)) {
      bmp280_address = address;

      Serial.print("BMP280 address=0x");
      imu_print_hex_byte(address);
      Serial.print(" id=0x");
      imu_print_hex_byte(chip_id);
      Serial.println(chip_id == 0x58 ? " ok" : " unexpected");

      const bool reset_sent = imu_write_register(*imu_bus, address, BMP280_RESET, 0xB6);
      delay(5);

      return reset_sent &&
             imu_write_register(*imu_bus, address, BMP280_CONFIG, 0xA0) &&
             imu_write_register(*imu_bus, address, BMP280_CTRL_MEAS, 0x27);
    }
  }

  Serial.println("BMP280: not found");
  return false;
}

bool read_itg320x(Axis3& gyro, float& temperature_c)
{
  uint8_t data[8] = {};

  if (!imu_read_registers(*imu_bus, ITG320X_ADDRESS, ITG320X_TEMP_OUT_H, data, sizeof(data))) {
    return false;
  }

  const int16_t raw_temp =
    static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);

  gyro.x = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
  gyro.y = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);
  gyro.z = static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) | data[7]);

  temperature_c = 35.0f + (static_cast<float>(raw_temp) + 13200.0f) / 280.0f;

  return true;
}

float gyro_z_to_dps(int16_t raw_z)
{
  return static_cast<float>(raw_z) / ITG320X_LSB_PER_DPS;
}

float corrected_gyro_z_dps(int16_t raw_z)
{
  float value = gyro_z_to_dps(raw_z) - gyro_z_bias_dps;

  if (value > -GYRO_Z_DEADBAND_DPS && value < GYRO_Z_DEADBAND_DPS) {
    value = 0.0f;
  }

  return value;
}

void reset_turn_angle()
{
  turn_angle_deg = 0.0f;
  last_gyro_update_us = micros();
}

void update_turn_angle()
{
  if (!itg320x_ready) {
    return;
  }

  const unsigned long now_us = micros();

  if (last_gyro_update_us == 0) {
    last_gyro_update_us = now_us;
    return;
  }

  Axis3 gyro = {};
  float temperature_c = 0.0f;

  if (!read_itg320x(gyro, temperature_c)) {
    last_gyro_update_us = now_us;
    return;
  }

  const float dt_s = static_cast<float>(now_us - last_gyro_update_us) / 1000000.0f;

  last_gyro_update_us = now_us;

  turn_angle_deg += corrected_gyro_z_dps(gyro.z) * dt_s;
}

void calibrate_gyro_z_bias()
{
  if (!itg320x_ready) {
    return;
  }

  Serial.println("Keep IMU still: calibrating gyro Z bias...");

  float sum_dps = 0.0f;
  uint16_t sample_count = 0;

  for (uint16_t i = 0; i < GYRO_BIAS_SAMPLE_COUNT; ++i) {
    Axis3 gyro = {};
    float temperature_c = 0.0f;

    if (read_itg320x(gyro, temperature_c)) {
      sum_dps += gyro_z_to_dps(gyro.z);
      ++sample_count;
    }

    delay(GYRO_BIAS_SAMPLE_DELAY_MS);
  }

  if (sample_count > 0) {
    gyro_z_bias_dps = sum_dps / static_cast<float>(sample_count);
  }

  reset_turn_angle();

  Serial.print("gyro_z_bias_dps=");
  Serial.println(gyro_z_bias_dps, 3);
}

void beginIMU()
{
  Wire.begin();
  Wire.setClock(100000);

  // Wire1 is already used by Motoron/RFID in your code, but this makes sure it is active.
  Wire1.begin();
  Wire1.setClock(100000);

#if WIRE_HOWMANY > 2
  Wire2.begin();
  Wire2.setClock(100000);
#endif

  delay(100);

  select_best_imu_i2c_bus();

  adxl345_ready = setup_adxl345();
  itg320x_ready = setup_itg320x();
  hmc5883l_ready = setup_hmc5883l();
  bmp280_ready = setup_bmp280();

  Serial.println("IMU detected. Gyro bias calibration will happen after IR calibration.");

  Serial.print("IMU ready=");
  Serial.print(adxl345_ready ? 'A' : '-');
  Serial.print(itg320x_ready ? 'G' : '-');
  Serial.print(hmc5883l_ready ? 'M' : '-');
  Serial.print(bmp280_ready ? 'B' : '-');
  Serial.println();

  if (!adxl345_ready && !itg320x_ready && !hmc5883l_ready && !bmp280_ready) {
    Serial.println("No IMU devices found. Check VCC, GND, SDA/SCL and common ground.");
  }
}

void printIMUStatus()
{
  Axis3 gyro = {};
  float gyro_temp_c = 0.0f;

  Serial.print(" imu_ready=");
  Serial.print(adxl345_ready ? 'A' : '-');
  Serial.print(itg320x_ready ? 'G' : '-');
  Serial.print(hmc5883l_ready ? 'M' : '-');
  Serial.print(bmp280_ready ? 'B' : '-');

  if (itg320x_ready && read_itg320x(gyro, gyro_temp_c)) {
    Serial.print(" gyro_z=");
    Serial.print(gyro_z_to_dps(gyro.z), 2);

    Serial.print(" gyro_z_corr=");
    Serial.print(corrected_gyro_z_dps(gyro.z), 2);

    Serial.print(" turn_deg=");
    Serial.print(turn_angle_deg, 1);
  }
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

const char* junctionTypeName(JunctionType type)
{
  switch (type) {
    case JunctionType::None:
      return "none";
    case JunctionType::Straight:
      return "straight";
    case JunctionType::LeftTurn:
      return "left_turn";
    case JunctionType::RightTurn:
      return "right_turn";
    case JunctionType::TIntersection:
      return "t_intersection";
    case JunctionType::WideIntersection:
      return "wide_intersection";
    case JunctionType::Lost:
      return "lost";
    default:
      return "unknown";
  }
}

bool sensor_active(uint8_t index)
{
  return target_value_at(index) >= INTERSECTION_THRESHOLD;
}

uint8_t count_active_range(uint8_t start_index, uint8_t end_index)
{
  uint8_t count = 0;

  for (uint8_t i = start_index; i <= end_index; ++i) {
    if (sensor_active(i)) {
      count++;
    }
  }

  return count;
}

JunctionType detect_junction_type()
{
  const uint8_t left_count = count_active_range(0, 2);
  const uint8_t center_count = count_active_range(3, 5);
  const uint8_t right_count = count_active_range(6, 8);

  const bool left_active = left_count >= 2;
  const bool center_active = center_count >= 1;
  const bool right_active = right_count >= 2;

  const uint8_t total_active = left_count + center_count + right_count;

  if (total_active == 0) {
    return JunctionType::Lost;
  }

  if (left_active && center_active && right_active) {
    if (total_active >= 7) {
      return JunctionType::WideIntersection;
    }

    return JunctionType::TIntersection;
  }

  if (left_active && center_active && !right_active) {
    return JunctionType::LeftTurn;
  }

  if (!left_active && center_active && right_active) {
    return JunctionType::RightTurn;
  }

  // At a sharp 90-degree corner the center sensors can leave the line before
  // the side sensors do. Treat side-only detections as corner entries.
  if (left_active && !center_active && !right_active) {
    return JunctionType::LeftTurn;
  }

  if (!left_active && !center_active && right_active) {
    return JunctionType::RightTurn;
  }

  if (!left_active && center_active && !right_active) {
    return JunctionType::Straight;
  }

  return JunctionType::None;
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
  const JunctionType junction = found ? detect_junction_type() : JunctionType::Lost;

  const bool left_turn = junction == JunctionType::LeftTurn;
  const bool right_turn = junction == JunctionType::RightTurn;
  const bool t_intersection = junction == JunctionType::TIntersection;
  const bool wide_intersection = junction == JunctionType::WideIntersection;

  const bool special_junction =
    left_turn ||
    right_turn ||
    t_intersection ||
    wide_intersection;

  const int32_t position = found ? estimate_line_position() : -1;
  const int32_t error = position >= 0 ? position - LINE_CENTER : last_error;

  if (!run_enabled) {
    set_follow_state(FollowState::Idle);
    stopMotors();
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

  if (special_junction && millis() - last_junction_detect_ms > 600) {
    last_junction_type = junction;
    last_junction_detect_ms = millis();

    Serial.print("junction=");
    Serial.println(junctionTypeName(junction));

    set_follow_state(FollowState::CrossIntersection);

    if (junction == JunctionType::LeftTurn) {
      Serial.println("Detected left turn.");
      cornerTurnToLine(1);
      return;
    }

    if (junction == JunctionType::RightTurn) {
      Serial.println("Detected right turn.");
      stopMotors();
      cornerTurnToLine(-1);
      return;
    }

    if (junction == JunctionType::TIntersection) {
      Serial.println("Detected T intersection.");
      stopMotors();
      handleTIntersectionDecision(T_INTERSECTION_DECISION);
      return;
    }

    if (junction == JunctionType::WideIntersection) {
      Serial.println("Detected wide intersection.");
      stopMotors();
      handleTIntersectionDecision(WIDE_INTERSECTION_DECISION);
      return;
    }
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

  Serial.print(" junction=");
  if (found) {
    Serial.print(junctionTypeName(detect_junction_type()));
  } else {
    Serial.print("lost");
  }

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

  printIMUStatus();

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

/*byte estimatePositionMap()
{
  for (int rows = 0; rows > 10; rows++) {
    for (int cols = 0; cols > 10; cols++) {
      if (arenaMap[rows][cols] == 5) { // 5 is our robot
        break
      }
    }
    if (arenaMap[rows][cols] == 5) { // 5 is our robot
        break
    }
  }
}*/

// =====================================================
// Setup
// =====================================================

void up_to_line_following_app_setup()
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

  beginIMU();

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
    startup_blocked = true;
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
    startup_blocked = true;
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

  startup_cal_state = StartupCalState::CalibratingIR;
  calibration_start_ms = millis();
  last_control_ms = millis();
  last_line_status_ms = millis();

  Serial.println("IR calibration: move sensors over floor and line for 5 seconds.");
  Serial.println("Then place the robot flat and still when the LED turns yellow.");
}

// =====================================================
// Loop
// =====================================================

void up_to_line_following_app_loop()
{
  if (startup_blocked) {
    stopMotors();
    Red();
    delay(25);
    return;
  }

  serviceServoPulses();
  update_turn_angle();

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

  if (startup_cal_state != StartupCalState::Ready) {
    stopMotors();

    if (startup_cal_state == StartupCalState::CalibratingIR) {
      update_calibration();

      if (now_ms - calibration_start_ms >= CALIBRATION_TIME_MS) {
        print_calibration();

        startup_cal_state = StartupCalState::WaitingForStillIMU;
        imu_still_start_ms = millis();

        Serial.println("IR calibration complete.");
        Serial.println("YELLOW LED: place robot flat and keep it still for IMU calibration.");
      }

      return;
    }

    if (startup_cal_state == StartupCalState::WaitingForStillIMU) {
      Yellow();

      if (now_ms - imu_still_start_ms >= IMU_STILL_WAIT_MS) {
        startup_cal_state = StartupCalState::CalibratingIMU;
        Serial.println("Starting IMU gyro calibration. Keep robot still.");
      }

      return;
    }

    if (startup_cal_state == StartupCalState::CalibratingIMU) {
      Yellow();

      calibrate_gyro_z_bias();

      startup_cal_state = StartupCalState::Ready;
      calibration_done = true;

      Serial.println("All calibration complete. Robot ready.");

      return;
    }
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

  if (ENABLE_OPEN_FIELD_TEST4) {
    Red();

    if (!open_field_test4_done && !open_field_test4_running) {
      runOpenFieldTest4();
    } else {
      stopMotors();
    }

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
    printEncoders();
  }
}
