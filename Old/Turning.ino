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

      Serial.print("turn_line_found angle=");
      Serial.println(turn_angle_deg, 1);

      delay(80);
      return true;
    }

    if (millis() - start_ms >= TURN_TIMEOUT_MS) {
      Serial.print("turn_timeout angle=");
      Serial.println(turn_angle_deg, 1);
      stopMotors();
      set_follow_state(FollowState::LostLine);
      return false;
    }
  }

  stopMotors();

  Serial.print("turn_no_line angle=");
  Serial.println(turn_angle_deg, 1);

  set_follow_state(FollowState::LostLine);
  return false;
}

bool turnLeft90WithLines()
{
  return turnUntilLineOrAngle(TURN_90_TARGET_DEG, 1, TURN_90_IGNORE_LINE_UNTIL_DEG);
}

bool turnRight90WithLines()
{
  return turnUntilLineOrAngle(TURN_90_TARGET_DEG, -1, TURN_90_IGNORE_LINE_UNTIL_DEG);
}

bool turnLeft180WithLines()
{
  return turnUntilLineOrAngle(TURN_180_TARGET_DEG, 1, TURN_180_IGNORE_LINE_UNTIL_DEG);
}

bool turnRight180WithLines()
{
  return turnUntilLineOrAngle(TURN_180_TARGET_DEG, -1, TURN_180_IGNORE_LINE_UNTIL_DEG);
}

void turnLeft90()
{
  turnLeft90WithLines();
}

void turnRight90()
{
  turnRight90WithLines();
}

void turnLeft180()
{
  turnLeft180WithLines();
}

void turnRight180()
{
  turnRight180WithLines();
}