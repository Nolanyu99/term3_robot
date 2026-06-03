// =====================================================
// ObstacleAvoid.ino
// Obstacle-swerve state machine integrated with the main robot.
//
// Uses functions/globals defined in other tabs:
//   - update_line_following_no_junction_turns() (LineSensors.ino)
//   - turnLeft90WithLines() / turnRight90WithLines() (Turning.ino)
//   - stopMotors() (MotorEncoders.ino)
//   - rfid object from RFID.ino
//   - forward_distance_mm() and ultrasonic pins from Main.ino
// =====================================================

constexpr float OBSTACLE_DETECT_MM = 200.0f;

SwerveState swerve_state = SwerveState::LineFollow;
bool obstacle_cleared = false;

// Track RFID detection by watching for new tags.
unsigned long last_seen_rfid_ms = 0;
constexpr unsigned long RFID_DEBOUNCE_MS = 1500;

bool obstacle_ahead()
{
  return forward_distance_mm() < OBSTACLE_DETECT_MM;
}

bool junction_reached()
{
  if (millis() - last_seen_rfid_ms < RFID_DEBOUNCE_MS) {
    return false;
  }

  if (!rfid.PICC_IsNewCardPresent()) {
    return false;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    return false;
  }

  Serial.print("Junction RFID: ");

  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) {
      Serial.print('0');
    }

    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(' ');
  }

  Serial.println();

  rfid.PICC_HaltA();
  last_seen_rfid_ms = millis();
  return true;
}

void setup_obstacle_avoid()
{
  // The forward ultrasonic pins are already configured by setup_tunnel().
  swerve_state = SwerveState::LineFollow;
  obstacle_cleared = false;
  last_seen_rfid_ms = 0;
}

void set_swerve_state(SwerveState next)
{
  if (swerve_state == next) {
    return;
  }

  Serial.print("Swerve state -> ");
  swerve_state = next;
  Serial.println(static_cast<int>(next));
}

void obstacle_line_follow_step()
{
  read_rc_discharge_times();
  update_line_following_no_junction_turns();
}

void prepare_after_swerve_turn(int8_t expected_line_side)
{
  read_rc_discharge_times();
  update_calibrated_values();
  last_error = 0;
  last_line_side = expected_line_side;
  set_follow_state(FollowState::FollowLine);
  last_control_ms = millis();
  delay(100);
}

void run_obstacle_avoid_step()
{
  switch (swerve_state) {
    case SwerveState::LineFollow:
      obstacle_line_follow_step();

      if (obstacle_ahead()) {
        Serial.println("Obstacle detected - approaching to junction.");
        set_swerve_state(SwerveState::ApproachObstacle);
      }
      break;

    case SwerveState::ApproachObstacle:
      obstacle_line_follow_step();

      if (junction_reached()) {
        Serial.println("At node 1 tile before obstacle - stopping.");
        stopMotors();
        set_swerve_state(SwerveState::TurnRight);
      }
      break;

    case SwerveState::TurnRight:
      turnRight90WithLines();
      prepare_after_swerve_turn(1);
      obstacle_cleared = false;
      set_swerve_state(SwerveState::GoStraightUntilJunc1);
      break;

    case SwerveState::GoStraightUntilJunc1:
      obstacle_line_follow_step();

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
      prepare_after_swerve_turn(-1);
      obstacle_cleared = false;
      set_swerve_state(SwerveState::GoStraightUntilJunc2);
      break;

    case SwerveState::GoStraightUntilJunc2:
      obstacle_line_follow_step();

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
      prepare_after_swerve_turn(-1);
      set_swerve_state(SwerveState::GoStraightUntilJunc3);
      break;

    case SwerveState::GoStraightUntilJunc3:
      obstacle_line_follow_step();

      if (junction_reached()) {
        stopMotors();
        Serial.println("Back on original trajectory. Done.");
        set_swerve_state(SwerveState::Done);
      }
      break;

    case SwerveState::Done:
      stopMotors();
      suppress_junction_turns = false;
      swerve_state = SwerveState::LineFollow;
      break;
  }
}
