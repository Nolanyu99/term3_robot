// =====================================================
// ObstacleAvoid.ino
// Standalone obstacle-swerve state machine.
// Uses functions defined in other tabs:
//   - update_line_following()   (LineSensors.ino)
//   - turnLeft90WithLines()     (Turning.ino)
//   - turnRight90WithLines()    (Turning.ino)
//   - stopMotors()              (MotorEncoders.ino)
//   - handleRFID() / RFID globals from RFID.ino
// =====================================================

#include <Arduino.h>

// Forward sensor pins (matching tunnel sketch)
constexpr uint8_t TRIG_FORWARD = 52;
constexpr uint8_t ECHO_FORWARD = 53;

constexpr float OBSTACLE_DETECT_MM = 200.0f;
constexpr float US_MIN_VALID_MM    = 20.0f;
constexpr float US_MAX_VALID_MM    = 2000.0f;

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

SwerveState swerve_state = SwerveState::LineFollow;
bool obstacle_cleared = false;

// Track RFID detection by watching for new tags
unsigned long last_seen_rfid_ms = 0;
constexpr unsigned long RFID_DEBOUNCE_MS = 1500;

// -----------------------------------------------------
// Forward ultrasonic
// -----------------------------------------------------
float read_forward_mm() {
  digitalWrite(TRIG_FORWARD, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_FORWARD, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_FORWARD, LOW);
  unsigned long dur = pulseIn(ECHO_FORWARD, HIGH, 30000);
  if (dur == 0) return -1.0f;
  return (dur * 0.343f) / 2.0f;
}

float forward_distance_mm_obs() {
  float d = read_forward_mm();
  if (d > US_MIN_VALID_MM && d < US_MAX_VALID_MM) {
    last_valid_forward_mm = d;
    return d;
  }
  return last_valid_forward_mm;
}

bool obstacle_ahead() {
  return forward_distance_mm_obs() < OBSTACLE_DETECT_MM;
}

// -----------------------------------------------------
// Junction detection via RFID (uses rfid object from RFID.ino)
// -----------------------------------------------------
bool junction_reached() {
  if (millis() - last_seen_rfid_ms < RFID_DEBOUNCE_MS) return false;
  if (!rfid.PICC_IsNewCardPresent())                   return false;
  if (!rfid.PICC_ReadCardSerial())                     return false;

  Serial.print("Junction RFID: ");
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) Serial.print('0');
    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  rfid.PICC_HaltA();
  last_seen_rfid_ms = millis();
  return true;
}

// -----------------------------------------------------
// Setup — call once from main setup()
// -----------------------------------------------------
void setup_obstacle_avoid() {
  pinMode(TRIG_FORWARD, OUTPUT);
  pinMode(ECHO_FORWARD, INPUT);
  digitalWrite(TRIG_FORWARD, LOW);

  swerve_state = SwerveState::LineFollow;
  obstacle_cleared = false;
}

void set_swerve_state(SwerveState next) {
  if (swerve_state == next) return;
  Serial.print("Swerve state -> ");
  swerve_state = next;
  Serial.println((int)next);
}

// -----------------------------------------------------
// Step — call once per main loop iteration
// -----------------------------------------------------
void run_obstacle_avoid_step() {
  switch (swerve_state) {

    case SwerveState::LineFollow:
      update_line_following();
      if (obstacle_ahead()) {
        Serial.println("Obstacle detected — approaching to junction.");
        set_swerve_state(SwerveState::ApproachObstacle);
      }
      break;

    case SwerveState::ApproachObstacle:
      update_line_following();
      if (junction_reached()) {
        Serial.println("At node 1 tile before obstacle — stopping.");
        stopMotors();
        set_swerve_state(SwerveState::TurnRight);
      }
      break;

    case SwerveState::TurnRight:
      turnRight90WithLines();
      obstacle_cleared = false;
      set_swerve_state(SwerveState::GoStraightUntilJunc1);
      break;

    case SwerveState::GoStraightUntilJunc1:
      update_line_following();
      if (!obstacle_cleared) {
        if (!obstacle_ahead()) {
          Serial.println("Obstacle passed (leg 1).");
          obstacle_cleared = true;
        }
      } else if (junction_reached()) {
        stopMotors();
        set_swerve_state(SwerveState::TurnLeft1);
      }
      break;

    case SwerveState::TurnLeft1:
      turnLeft90WithLines();
      obstacle_cleared = false;
      set_swerve_state(SwerveState::GoStraightUntilJunc2);
      break;

    case SwerveState::GoStraightUntilJunc2:
      update_line_following();
      if (!obstacle_cleared) {
        if (!obstacle_ahead()) {
          Serial.println("Obstacle passed (leg 2).");
          obstacle_cleared = true;
        }
      } else if (junction_reached()) {
        stopMotors();
        set_swerve_state(SwerveState::TurnLeft2);
      }
      break;

    case SwerveState::TurnLeft2:
      turnLeft90WithLines();
      set_swerve_state(SwerveState::GoStraightUntilJunc3);
      break;

    case SwerveState::GoStraightUntilJunc3:
      update_line_following();
      if (junction_reached()) {
        stopMotors();
        Serial.println("Back on original trajectory. Done.");
        set_swerve_state(SwerveState::Done);
      }
      break;

    case SwerveState::Done:
      stopMotors();
      break;
  }
}
