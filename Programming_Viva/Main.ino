#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <math.h>
#include <MFRC522_I2C.h>
#include "ArenaTypes.h"

// =====================================================
// More Global Variables and Constants
// =====================================================

namespace robot_config {

// Network used by Messages, robot should keep working if unavailable
constexpr const char* WIFI_SSID = "PhaseSpaceNetwork_2.4G";
constexpr const char* WIFI_PASSWORD = "8igMacNet";
constexpr uint16_t UDP_PORT = 4210;

// Sensor/Feature switches
constexpr bool ENABLE_TOP_RGB = true;
constexpr bool ENABLE_MECHANICAL_KILL_BUTTON = true;
constexpr bool ENABLE_REVIVE_BUTTON = true;
constexpr bool ENABLE_QTR_STATUS = true;
constexpr bool ENABLE_ULTRASONIC_STATUS = true;
constexpr bool ENABLE_RFID_STATUS = true;
constexpr bool ENABLE_SEED_DISPENSER = true;

// LED & button pins
constexpr int TOP_RGB_RED_PIN = 39;
constexpr int TOP_RGB_BLUE_PIN = 37;
constexpr int TOP_RGB_GREEN_PIN = 35;
constexpr bool TOP_RGB_COMMON_ANODE = false;
constexpr int MECHANICAL_KILL_BUTTON_PIN = 33;
constexpr int REVIVE_BUTTON_PIN = 13;

// Motor and encoder constants
constexpr float MOTOR_GEAR_RATIO = 100.0f;
constexpr float MOTOR_RAW_CPR = 24.0f;

// Reflectance sensor tuning
constexpr uint16_t QTR_RC_TIMEOUT_US = 1000;
constexpr bool QTR_LINE_IS_HIGH_RAW = true;
constexpr bool QTR_FOLLOW_BLACK_LINE = true;
constexpr uint16_t QTR_LINE_DETECT_ON_THRESHOLD = 650;
constexpr uint16_t QTR_LINE_DETECT_OFF_THRESHOLD = 450;

// Servo timing
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

// RFID wiring
constexpr uint8_t RFID_I2C_ADDRESS = 0x28;
constexpr int RFID_RESET_PIN = -1;

}

// =====================================================
// State machine(s)
// =====================================================

enum class FollowState { // For line following
  Idle,
  FollowLine,
  CrossIntersection,
  LineGap,
  LostLine,
};

enum class JunctionType { // For junctionse encountered during line following
  None,
  Straight,
  LeftTurn,
  RightTurn,
  TIntersection,
  WideIntersection,
  Lost
};

enum class StartupCalState { // For calibrating IR sensor & IMU
  CalibratingIR,
  WaitingForStillIMU,
  CalibratingIMU,
  Ready
};

enum class MissionPhase { // For knowing where it is
  BaseToGate,
  Tunnel,
  Arena
};

enum class TunnelState { // For tunnel following
  WaitAtBaseGate,
  DrivingIntoTunnel,
  WallFollow,
  WaitAtExitGate,
  PassingExitGate
};

// Setting default settings
StartupCalState startup_cal_state = StartupCalState::CalibratingIR;
MissionPhase mission_phase = MissionPhase::BaseToGate;
FollowState follow_state = FollowState::Idle;

JunctionType last_junction_type = JunctionType::None;
unsigned long last_junction_detect_ms = 0;

// =====================================================
// More global timing constants
// =====================================================

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 5000;
constexpr unsigned long CALIBRATION_TIME_MS = 5000;
constexpr unsigned long CONTROL_INTERVAL_MS = 30;
constexpr unsigned long LINE_STATUS_INTERVAL_MS = 250;
constexpr unsigned long INTERSECTION_CROSS_MS = 500;
constexpr unsigned long RFID_SCAN_COOLDOWN_MS = 1500;
constexpr unsigned long LINE_GAP_FORWARD_MS = 500;
constexpr int16_t LINE_GAP_SPEED = 220;

constexpr unsigned long IMU_STILL_WAIT_MS = 3000;
unsigned long imu_still_start_ms = 0;

// =====================================================
// IMU and turning
// =====================================================

constexpr unsigned long TURN_TIMEOUT_MS = 5000;
constexpr uint16_t GYRO_BIAS_SAMPLE_COUNT = 200;
constexpr unsigned long GYRO_BIAS_SAMPLE_DELAY_MS = 5;

constexpr float ITG320X_LSB_PER_DPS = 14.375f;
constexpr float GYRO_Z_DEADBAND_DPS = 0.4f;

constexpr float TURN_90_TARGET_DEG = 100.0f; // Max turn before second guessing
constexpr float TURN_180_TARGET_DEG = 190.0f;
constexpr float TURN_90_IGNORE_LINE_UNTIL_DEG = 25.0f; // Minimum turn before reading scanner
constexpr float TURN_180_IGNORE_LINE_UNTIL_DEG = 120.0f;

constexpr uint16_t TURN_LINE_DETECT_THRESHOLD = 600;
constexpr uint16_t TURN_CENTER_DETECT_THRESHOLD = 550;

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

struct Axis3 {
  int16_t x;
  int16_t y;
  int16_t z;
};

// ===================================================================================
// IR/Line sensor for LineSensors, Turning, RobotBehaviour, and general pathfinding
// ===================================================================================

constexpr uint8_t SENSOR_COUNT = 9;
constexpr uint16_t LINE_POSITION_SCALE = 1000;
constexpr int32_t LINE_CENTER = ((SENSOR_COUNT - 1) * LINE_POSITION_SCALE) / 2;

constexpr uint16_t INTERSECTION_THRESHOLD = 800;
constexpr uint8_t INTERSECTION_SENSOR_COUNT = 7;

constexpr unsigned long LOST_REVERSE_MS = 250;
constexpr unsigned long LOST_GENTLE_SEARCH_MS = 900;

constexpr int16_t LOST_REVERSE_SPEED = 160;
constexpr int16_t LOST_GENTLE_TURN_SPEED = 180;
constexpr int16_t LOST_HARD_TURN_SPEED = 260;

const uint8_t sensor_pins[SENSOR_COUNT] = {2, 3, 4, 5, 8, 9, 10, 11, 12};

uint16_t raw_values[SENSOR_COUNT] = {};
uint16_t min_values[SENSOR_COUNT] = {};
uint16_t max_values[SENSOR_COUNT] = {};
uint16_t calibrated_values[SENSOR_COUNT] = {};

unsigned long calibration_start_ms = 0;
unsigned long state_start_ms = 0;
unsigned long last_control_ms = 0;
unsigned long last_line_status_ms = 0;

int32_t last_error = 0;
int8_t last_line_side = 0;

bool calibration_done = false;
bool line_detected = false;
bool run_enabled = true;

// =====================================================
// Motors for MotorEncoders and general driving logic
// =====================================================

MotoronI2C mc(0x10);

const uint8_t MOTOR_LEFT  = 1;
const uint8_t MOTOR_RIGHT = 2;

const double MOTOR_RATIO = 1.4403 / 1.4681;

constexpr int16_t BASE_SPEED = 250;
constexpr int16_t TURN_SPEED = 230;

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

// ===================================================================
// Wheel encoders for distance estimates, tunnel entry, and debugging 
// ===================================================================

const uint8_t ENC_LEFT_A  = 28;
const uint8_t ENC_LEFT_B  = 26;
const uint8_t ENC_RIGHT_A = 22;
const uint8_t ENC_RIGHT_B = 24;

const double wheel_diameter = 38.35;

volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

// =====================================================
// LEDs and buttons for LEDs, stopping, and reviving
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

// ===========================================================
// RFID reader for RFID, RobotBehaviour, and ArenaPathfinding
// ===========================================================

constexpr uint8_t RFID_ADDR = robot_config::RFID_I2C_ADDRESS;
MFRC522_I2C rfid(RFID_ADDR, robot_config::RFID_RESET_PIN, &Wire1);

// ====================================================
// Seed dispenser servos for Servos and RobotBehaviour
// ====================================================

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

// ===================================================================================================================
// Tunnel entry ultrasonic control for Tunnel_Entry wall following and door detection, and general obstacle avoidance
// ===================================================================================================================

constexpr uint8_t TUNNEL_TRIG_SIDE    = 47;
constexpr uint8_t TUNNEL_ECHO_SIDE    = 46;
constexpr uint8_t TUNNEL_TRIG_FORWARD = 52;
constexpr uint8_t TUNNEL_ECHO_FORWARD = 53;

constexpr float BASE_DOOR_OPEN_MM      = 250.0f;
constexpr float TARGET_SIDE_MM         = 60.0f;
constexpr float SIDE_WALL_DETECTED_MM  = 120.0f;
constexpr float TUNNEL_DOOR_CLOSED_MM  = 50.0f;
constexpr float TUNNEL_DOOR_OPEN_MM    = 250.0f;
constexpr float TUNNEL_DONE_MM         = 500.0f;
constexpr float US_MAX_VALID_MM        = 2000.0f;
constexpr float US_MIN_VALID_MM        = 20.0f;

constexpr float    WALL_KP         = 4.0f;
constexpr int16_t  BASE_RAMP_SPEED = 400;
constexpr int16_t  WALL_MAX_SPEED  = 600;

constexpr long BASE_EXIT_MIN_DISTANCE_COUNTS = 2000;
constexpr unsigned long BASE_EXIT_DRIVE_TIMEOUT_MS = 3000;

// =====================================================
// setup
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
  Serial.println("=== GIGA R1 line-following + IMU turning + RFID seed robot ===");

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

  setupRFID();
  setupServos();
  setup_tunnel();
  setupMessages();

  reset_calibration();

  startup_cal_state = StartupCalState::CalibratingIR;
  calibration_start_ms = millis();
  last_control_ms = millis();
  last_line_status_ms = millis();

  Serial.println("IR calibration: move sensors over floor and line for 5 seconds.");
  Serial.println("Then place the robot flat and still when the LED turns yellow.");
}

// =====================================================
// loop()
// =====================================================

void loop()
{
  serviceServoPulses(); // Make sure servos are functioning
  update_turn_angle(); // Update angle
  messagesLoop(); // Listen for messages

  if (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());

    if (command == '0') { // If 0 stop
      running = !running;

      if (!running) {
        stopMotors();
        Serial.println("running=0");
      } else {
        Serial.println("running=1");
      }

      return;
    }

    else if (command == '3') { // If 3 run test 3
      RunTest3();
      return;
    }

    else if (command == 't' || command == 'T') { // If t/T tunnel entry
      startTunnelEntry();
      return;
    }

    else if (command == 'e' || command == 'E') { // If e/E go to exit
      arenaGoToExit();
      return;
    }

    else if (command == 'b' || command == 'B') { // If b/B go to entrance
      arenaReturnToEntrance();
      return;
    }
  }

  int OffButtonPressed = digitalRead(OffButtonPin); // Read buttons
  int RevButtonPressed = digitalRead(RevButtonPin);

  Reviving = false;

  if (OffButtonPressed == LOW && previousOffButtonPressed != LOW) { // To gaurd against holding it down
    running = !running;
    delay(50);
  } else if (RevButtonPressed == LOW) {
    Reviving = true;
    delay(50);
  }

  previousOffButtonPressed = OffButtonPressed;

  read_rc_discharge_times(); // IR sensor

  const unsigned long now_ms = millis();

  if (startup_cal_state != StartupCalState::Ready) {
    stopMotors();

    if (startup_cal_state == StartupCalState::CalibratingIR) {
      Blue();
      update_calibration(); // Initial calibration of IR

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
      calibrate_gyro_z_bias(); // Calibration of IMU
      startup_cal_state = StartupCalState::Ready;
      calibration_done = true;
      Serial.println("All calibration complete. Robot ready.");
      return;
    }
  }

  if (!messagesRobotAllowedToMove()) {
    run_enabled = false; // If turned off remotely
    stopMotors();
    Red();
    delay(25);
    return;
  }

  if (!running) {
    run_enabled = false; // If turned off localy
    Flash(previousStateRed);
    previousStateRed++;
    stopMotors();
    delay(25);
    return;
  }

  run_enabled = true;

  if (Reviving) { // For reviving
    Green();
    stopMotors();
    delay(500);
    setMotors(-BASE_SPEED, -BASE_SPEED);
    return;
  }

  if (mission_phase == MissionPhase::Tunnel) { // If in tunnel logic
    Red();

    if (now_ms - last_control_ms >= CONTROL_INTERVAL_MS) {
      last_control_ms = now_ms;
      run_tunnel_step();
    }

    return;
  }

  Red();
  stopMotors();

  if (now_ms - last_line_status_ms >= LINE_STATUS_INTERVAL_MS) { // Else just wait for command
    last_line_status_ms = now_ms;
    Serial.println("idle_waiting_for_command");
  }
}