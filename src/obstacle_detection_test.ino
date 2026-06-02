// =====================================================
// Obstacle swerve test sketch (standalone)
//
// Behaviour:
//   1. Line follow normally.
//   2. Forward ultrasonic sees obstacle -> ApproachObstacle.
//   3. Continue line following until the next RFID junction
//      (the node 1 tile before the obstacle), then stop.
//   4. Turn right 90° (uses turnRight90WithLines from Turning.ino).
//   5. Line follow until the obstacle is no longer ahead,
//      THEN until the next RFID junction. Stop.
//   6. Turn left 90°.
//   7. Same as 5.
//   8. Turn left 90°.
//   9. Line follow to next RFID junction (back on original line). Stop.
// =====================================================

#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>

// -----------------------------------------------------
// Hardware pins
// -----------------------------------------------------
constexpr uint8_t TRIG_FORWARD = 52;
constexpr uint8_t ECHO_FORWARD = 53;

// QTR sensors
constexpr uint8_t SENSOR_COUNT = 9;
const uint8_t sensor_pins[SENSOR_COUNT] = {2, 3, 4, 5, 8, 9, 10, 11, 12};

// -----------------------------------------------------
// Motors
// -----------------------------------------------------
MotoronI2C mc;
constexpr uint8_t MOTOR_LEFT  = 1;
constexpr uint8_t MOTOR_RIGHT = 2;
constexpr int16_t MAX_SPEED   = 600;
constexpr int16_t BASE_SPEED  = 250;
constexpr float   LINE_KP     = 0.055f;

// -----------------------------------------------------
// RFID
// -----------------------------------------------------
constexpr uint8_t RFID_ADDR = 0x28;
MFRC522_I2C rfid(RFID_ADDR, -1, &Wire1);

constexpr unsigned long RFID_COOLDOWN_MS = 1500;
unsigned long last_rfid_ms = 0;

// -----------------------------------------------------
// Encoders (used by Turning.ino's turn functions)
// -----------------------------------------------------
constexpr uint8_t ENC_LEFT_A  = 28;
constexpr uint8_t ENC_LEFT_B  = 26;
constexpr uint8_t ENC_RIGHT_A = 22;
constexpr uint8_t ENC_RIGHT_B = 24;

volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;

// -----------------------------------------------------
// Thresholds
// -----------------------------------------------------
constexpr float OBSTACLE_DETECT_MM = 200.0f;
constexpr float MIN_VALID_MM       = 20.0f;
constexpr float MAX_VALID_MM       = 2000.0f;

// -----------------------------------------------------
// QTR
// -----------------------------------------------------
constexpr uint16_t QTR_TIMEOUT_US      = 1000;
constexpr uint16_t LINE_ON_THRESHOLD   = 650;
constexpr uint16_t LINE_OFF_THRESHOLD  = 450;
constexpr unsigned long CALIBRATION_MS = 5000;
constexpr uint16_t LINE_POSITION_SCALE = 1000;
constexpr int32_t  LINE_CENTER = ((SENSOR_COUNT - 1) * LINE_POSITION_SCALE) / 2;

uint16_t raw_values[SENSOR_COUNT];
uint16_t min_values[SENSOR_COUNT];
uint16_t max_values[SENSOR_COUNT];
uint16_t cal_values[SENSOR_COUNT];

bool line_detected = false;
int32_t last_error = 0;
float last_valid_forward_mm = 1000.0f;

// -----------------------------------------------------
// State machine
// -----------------------------------------------------
enum class SwerveState {
  LineFollow,
  ApproachObstacle,
  TurnRight,
  GoStraightUntilJunc1,
  TurnLeft1,
  GoStraightUntilJunc2,
  TurnLeft2,
  GoStraightUntilJunc3,
  Done
};

SwerveState state = SwerveState::LineFollow;
bool obstacle_cleared = false;

// =====================================================
// Encoder ISRs
// =====================================================
void readLeftEncoder() {
  if (digitalRead(ENC_LEFT_A) == digitalRead(ENC_LEFT_B)) encoder_left_count--;
  else                                                    encoder_left_count++;
}

void readRightEncoder() {
  if (digitalRead(ENC_RIGHT_A) == digitalRead(ENC_RIGHT_B)) encoder_right_count++;
  else                                                      encoder_right_count--;
}

long getLeftEncoder() {
  noInterrupts(); long v = encoder_left_count; interrupts(); return v;
}

long getRightEncoder() {
  noInterrupts(); long v = encoder_right_count; interrupts(); return v;
}

// =====================================================
// Motor control
// =====================================================
void setMotors(int16_t left, int16_t right) {
  left  = constrain(left,  -MAX_SPEED, MAX_SPEED);
  right = constrain(right, -MAX_SPEED, MAX_SPEED);
  mc.setSpeed(MOTOR_LEFT,  left);
  mc.setSpeed(MOTOR_RIGHT, right);
}

void stopMotors() { setMotors(0, 0); }
void driveForward() { setMotors(BASE_SPEED, BASE_SPEED); }

// =====================================================
// Ultrasonic
// =====================================================
float readForwardMM() {
  digitalWrite(TRIG_FORWARD, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_FORWARD, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_FORWARD, LOW);
  unsigned long dur = pulseIn(ECHO_FORWARD, HIGH, 30000);
  if (dur == 0) return -1.0f;
  return (dur * 0.343f) / 2.0f;
}

float forwardDistanceMM() {
  float d = readForwardMM();
  if (d > MIN_VALID_MM && d < MAX_VALID_MM) {
    last_valid_forward_mm = d;
    return d;
  }
  return last_valid_forward_mm;
}

bool obstacleAhead() {
  return forwardDistanceMM() < OBSTACLE_DETECT_MM;
}

// =====================================================
// QTR
// =====================================================
void readQTR() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(sensor_pins[i], OUTPUT);
    digitalWrite(sensor_pins[i], HIGH);
  }
  delayMicroseconds(15);
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(sensor_pins[i], INPUT);
    raw_values[i] = QTR_TIMEOUT_US;
  }
  unsigned long start_us = micros();
  while (micros() - start_us < QTR_TIMEOUT_US) {
    uint16_t elapsed = (uint16_t)(micros() - start_us);
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      if (raw_values[i] == QTR_TIMEOUT_US && digitalRead(sensor_pins[i]) == LOW) {
        raw_values[i] = elapsed;
      }
    }
  }
}

void updateCalibratedValues() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    uint16_t range = max_values[i] - min_values[i];
    uint16_t v = 0;
    if (range > 0 && raw_values[i] > min_values[i]) {
      v = (uint16_t)min<uint32_t>(1000,
        ((uint32_t)(raw_values[i] - min_values[i]) * 1000) / range);
    }
    cal_values[i] = v;
  }
}

bool updateLineFound() {
  uint16_t peak = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (cal_values[i] > peak) peak = cal_values[i];
  }
  if (line_detected) {
    line_detected = peak >= LINE_OFF_THRESHOLD;
  } else {
    line_detected = peak >= LINE_ON_THRESHOLD;
  }
  return line_detected;
}

int32_t estimateLinePosition() {
  uint32_t sum = 0, weighted = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    weighted += (uint32_t)cal_values[i] * i * LINE_POSITION_SCALE;
    sum += cal_values[i];
  }
  if (sum == 0) return -1;
  return (int32_t)(weighted / sum);
}

void followLineStep() {
  readQTR();
  updateCalibratedValues();
  bool found = updateLineFound();
  int32_t pos = found ? estimateLinePosition() : -1;
  int32_t err = pos >= 0 ? pos - LINE_CENTER : last_error;

  if (!found) {
    driveForward();
    return;
  }

  last_error = err;
  int16_t correction = (int16_t)(LINE_KP * (float)err);
  setMotors(BASE_SPEED + correction, BASE_SPEED - correction);
}

// =====================================================
// RFID — junction detection
// =====================================================
bool junctionReached() {
  if (millis() - last_rfid_ms < RFID_COOLDOWN_MS) return false;
  if (!rfid.PICC_IsNewCardPresent())              return false;
  if (!rfid.PICC_ReadCardSerial())                return false;

  Serial.print("RFID UID: ");
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) Serial.print('0');
    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  rfid.PICC_HaltA();
  last_rfid_ms = millis();
  return true;
}

// =====================================================
// State machine
// =====================================================
void setState(SwerveState next) {
  if (state == next) return;
  Serial.print("State -> ");
  state = next;
  Serial.println((int)next);
}

void runStateMachine() {
  switch (state) {

    case SwerveState::LineFollow:
      followLineStep();
      if (obstacleAhead()) {
        Serial.println("Obstacle ahead — approaching to next junction.");
        setState(SwerveState::ApproachObstacle);
      }
      break;

    case SwerveState::ApproachObstacle:
      followLineStep();
      if (junctionReached()) {
        Serial.println("At node 1 tile before obstacle — stopping.");
        stopMotors();
        setState(SwerveState::TurnRight);
      }
      break;

    case SwerveState::TurnRight:
      turnRight90WithLines();
      obstacle_cleared = false;
      setState(SwerveState::GoStraightUntilJunc1);
      break;

    case SwerveState::GoStraightUntilJunc1:
      followLineStep();
      if (!obstacle_cleared) {
        if (!obstacleAhead()) {
          Serial.println("Obstacle passed (leg 1).");
          obstacle_cleared = true;
        }
      } else if (junctionReached()) {
        stopMotors();
        setState(SwerveState::TurnLeft1);
      }
      break;

    case SwerveState::TurnLeft1:
      turnLeft90WithLines();
      obstacle_cleared = false;
      setState(SwerveState::GoStraightUntilJunc2);
      break;

    case SwerveState::GoStraightUntilJunc2:
      followLineStep();
      if (!obstacle_cleared) {
        if (!obstacleAhead()) {
          Serial.println("Obstacle passed (leg 2).");
          obstacle_cleared = true;
        }
      } else if (junctionReached()) {
        stopMotors();
        setState(SwerveState::TurnLeft2);
      }
      break;

    case SwerveState::TurnLeft2:
      turnLeft90WithLines();
      setState(SwerveState::GoStraightUntilJunc3);
      break;

    case SwerveState::GoStraightUntilJunc3:
      followLineStep();
      if (junctionReached()) {
        stopMotors();
        Serial.println("Back on original trajectory. Done.");
        setState(SwerveState::Done);
      }
      break;

    case SwerveState::Done:
      stopMotors();
      break;
  }
}

// =====================================================
// Setup helpers
// =====================================================
void beginMotoron() {
  Wire1.begin();
  Wire1.setClock(100000);
  mc.setBus(&Wire1);
  mc.reinitialize();
  delay(10);
  mc.disableCrc();
  mc.clearResetFlag();
  mc.clearMotorFaultUnconditional();
  mc.setCommandTimeoutMilliseconds(2000);
  mc.setMaxAcceleration(MOTOR_LEFT,  800);
  mc.setMaxDeceleration(MOTOR_LEFT,  800);
  mc.setMaxAcceleration(MOTOR_RIGHT, 800);
  mc.setMaxDeceleration(MOTOR_RIGHT, 800);
}

void beginRFID() {
  rfid.PCD_WriteRegister(rfid.TModeReg,      0x80);
  rfid.PCD_WriteRegister(rfid.TPrescalerReg, 0xA9);
  rfid.PCD_WriteRegister(rfid.TReloadRegH,   0x03);
  rfid.PCD_WriteRegister(rfid.TReloadRegL,   0xE8);
  rfid.PCD_WriteRegister(rfid.TxASKReg,      0x40);
  rfid.PCD_WriteRegister(rfid.ModeReg,       0x3D);
  rfid.PCD_AntennaOn();
}

void calibrateQTR() {
  Serial.println("Calibrating QTR — sweep over line for 5s.");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    min_values[i] = QTR_TIMEOUT_US;
    max_values[i] = 0;
  }
  unsigned long start = millis();
  while (millis() - start < CALIBRATION_MS) {
    readQTR();
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      if (raw_values[i] < min_values[i]) min_values[i] = raw_values[i];
      if (raw_values[i] > max_values[i]) max_values[i] = raw_values[i];
    }
  }
  Serial.println("Calibration done.");
}

// =====================================================
// setup / loop
// =====================================================
void setup() {
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 3000) delay(10);

  Serial.println("=== Obstacle swerve test ===");

  pinMode(TRIG_FORWARD, OUTPUT);
  pinMode(ECHO_FORWARD, INPUT);
  digitalWrite(TRIG_FORWARD, LOW);

  pinMode(ENC_LEFT_A,  INPUT_PULLUP);
  pinMode(ENC_LEFT_B,  INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  readLeftEncoder,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), readRightEncoder, CHANGE);

  beginMotoron();
  beginRFID();

  calibrateQTR();

  Serial.println("Starting line follow.");
}

void loop() {
  runStateMachine();
}
