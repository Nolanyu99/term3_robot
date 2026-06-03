// =====================================================
// IMU-assisted turns
// Used by RobotBehaviour and ArenaPathfinding
// =====================================================

// Turns until line reacquisition or max angle is reached
bool turnUntilLineOrAngle(float max_degrees, int8_t direction, float ignore_line_until_deg)
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

  // Remember which way this turn was trying to go so, if the line is lost
  // immediately after the turn, lost-line recovery searches that same way.
  // recover_lost_line() uses -1 for left and +1 for right.
  last_turn_recovery_side = direction > 0 ? -1 : 1;

  reset_turn_angle();

  const unsigned long start_ms = millis();

  Serial.print("turn_scan_start direction=");
  Serial.print(direction > 0 ? "left" : "right");
  Serial.print(" max_deg=");
  Serial.print(max_degrees, 1);
  Serial.print(" ignore_until=");
  Serial.println(ignore_line_until_deg, 1);

  while (fabsf(turn_angle_deg) < max_degrees) {
    serviceServoPulses();
    update_turn_angle();

    if (direction > 0) {
      setMotors(-LEFT_FORWARD_SIGN * TURN_SPEED,
                 RIGHT_FORWARD_SIGN * TURN_SPEED);
    } else {
      setMotors( LEFT_FORWARD_SIGN * TURN_SPEED,
                -RIGHT_FORWARD_SIGN * TURN_SPEED);
    }

    const float abs_angle = fabsf(turn_angle_deg);

    if (abs_angle >= ignore_line_until_deg && blackLineDetectedDuringTurn()) {
      stopMotors();

      read_rc_discharge_times();
      update_calibrated_values();
      line_detected = true;
      last_error = 0;

      set_follow_state(FollowState::FollowLine);

      last_turn_end_ms = millis();

      Serial.print("turn_line_found angle=");
      Serial.println(turn_angle_deg, 1);

      delay(80);
      return true;
    }

    if (millis() - start_ms >= TURN_TIMEOUT_MS) {
      Serial.print("turn_timeout angle=");
      Serial.println(turn_angle_deg, 1);
      stopMotors();
      last_turn_end_ms = millis();
      set_follow_state(FollowState::LostLine);
      return false;
    }
  }

  stopMotors();

  Serial.print("turn_no_line angle=");
  Serial.println(turn_angle_deg, 1);

  last_turn_end_ms = millis();
  set_follow_state(FollowState::LostLine);
  return false;
}

// Turns left about 90 degrees and finds the line
bool turnLeft90WithLines()
{
  return turnUntilLineOrAngle(TURN_90_TARGET_DEG, 1, TURN_90_IGNORE_LINE_UNTIL_DEG);
}

// Turns right about 90 degrees and finds the line
bool turnRight90WithLines()
{
  return turnUntilLineOrAngle(TURN_90_TARGET_DEG, -1, TURN_90_IGNORE_LINE_UNTIL_DEG);
}

// Turns left about 180 degrees
bool turnLeft180WithLines()
{
  return turnUntilLineOrAngle(TURN_180_TARGET_DEG, 1, TURN_180_IGNORE_LINE_UNTIL_DEG);
}

// Turns right about 180 degrees
bool turnRight180WithLines()
{
  return turnUntilLineOrAngle(TURN_180_TARGET_DEG, -1, TURN_180_IGNORE_LINE_UNTIL_DEG);
}

// Performs left 90-degree line turn (more intuitive to call)
void turnLeft90()
{
  turnLeft90WithLines();
}

// Performs right 90-degree line turn (more intuitive to call)
void turnRight90()
{
  turnRight90WithLines();
}

// Performs left 180-degree line turn (more intuitive to call)
void turnLeft180()
{
  turnLeft180WithLines();
}

// Performs right 180-degree line turn (more intuitive to call)
void turnRight180()
{
  turnRight180WithLines();
}

// For deadckoning
bool turnByGyroOnly(float target_degrees, int8_t direction)
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

  // Keep lost-line recovery consistent after gyro-only turns too.
  last_turn_recovery_side = direction > 0 ? -1 : 1;

  reset_turn_angle();

  const unsigned long start_ms = millis();

  Serial.print("gyro_turn_start direction=");
  Serial.print(direction > 0 ? "left" : "right");
  Serial.print(" target=");
  Serial.println(target_degrees, 1);

  while (fabsf(turn_angle_deg) < target_degrees) {
    serviceServoPulses();
    update_turn_angle();

    if (direction > 0) {
      setMotors(-LEFT_FORWARD_SIGN * TURN_SPEED,
                 RIGHT_FORWARD_SIGN * TURN_SPEED);
    } else {
      setMotors( LEFT_FORWARD_SIGN * TURN_SPEED,
                -RIGHT_FORWARD_SIGN * TURN_SPEED);
    }

    if (millis() - start_ms >= TURN_TIMEOUT_MS) {
      Serial.print("gyro_turn_timeout angle=");
      Serial.println(turn_angle_deg, 1);
      stopMotors();
      last_turn_end_ms = millis();
      return false;
    }
  }

  stopMotors();
  last_turn_end_ms = millis();

  Serial.print("gyro_turn_done angle=");
  Serial.println(turn_angle_deg, 1);

  delay(100);
  return true;
}

bool turnLeft90GyroOnly()
{
  return turnByGyroOnly(90.0f, 1);
}

bool turnRight90GyroOnly()
{
  return turnByGyroOnly(90.0f, -1);
}