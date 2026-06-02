// ============================================================
// Reflectance-sensor calibration and line detection/following
// Used by Main, Turning, RobotBehaviour, and ArenaPathfinding
// ============================================================

// Reads reflectance sensors by RC discharge time
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

// Clears stored sensor min/max calibration values
void reset_calibration()
{
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    min_values[i] = robot_config::QTR_RC_TIMEOUT_US;
    max_values[i] = 0;
  }
}

// Expands min/max values from live readings
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

// Converts raw sensor values to calibrated 0 to 1000 values
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

// Determines whether line is W or B
uint16_t target_value_at(uint8_t sensor_index)
{
  return robot_config::QTR_FOLLOW_BLACK_LINE
           ? calibrated_values[sensor_index]
           : 1000 - calibrated_values[sensor_index];
}

// Returns strongest reading position
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

// Applies hysteresis to avoid line flicker
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

// Remembers which side last saw the line
void update_last_line_side()
{
  const uint16_t left_strength =
    target_value_at(0) + target_value_at(1) + target_value_at(2);

  const uint16_t center_strength =
    target_value_at(3) + target_value_at(4) + target_value_at(5);

  const uint16_t right_strength =
    target_value_at(6) + target_value_at(7) + target_value_at(8);

  if (left_strength > center_strength && left_strength > right_strength) {
    last_line_side = -1;
  } else if (right_strength > center_strength && right_strength > left_strength) {
    last_line_side = 1;
  } else if (center_strength > 0) {
    last_line_side = 0;
  }
}

// Detects when a turn has found the line
bool blackLineDetectedDuringTurn()
{
  read_rc_discharge_times();
  update_calibrated_values();

  const uint16_t peak = line_peak();

  const uint16_t center_peak =
    max(target_value_at(3), max(target_value_at(4), target_value_at(5)));

  return peak >= TURN_LINE_DETECT_THRESHOLD ||
         center_peak >= TURN_CENTER_DETECT_THRESHOLD;
}

// Detects intersections
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

// Checks one calibrated sensor
bool sensor_active(uint8_t index)
{
  return target_value_at(index) >= INTERSECTION_THRESHOLD;
}

// Counts active sensors in a range
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

// Classifies the current intersection shape
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

  if (!left_active && center_active && !right_active) {
    return JunctionType::Straight;
  }

  return JunctionType::None;
}

// Converts junction enum to text
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

// Computes weighted line position across the sensor array
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

// Changes state and records transition time
void set_follow_state(FollowState next_state)
{
  if (follow_state == next_state) {
    return;
  }

  follow_state = next_state;
  state_start_ms = millis();
}

// Runs one line-following control step
void update_line_following()
{
  update_calibrated_values();

  const bool found = update_line_found();

  if (found) {
    
    update_last_line_side();

    if (follow_state == FollowState::LineGap) {
      Serial.println("line_gap=recovered");
      set_follow_state(FollowState::FollowLine);
    }
  }

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
    if (follow_state != FollowState::LineGap &&
        follow_state != FollowState::LostLine) {
      set_follow_state(FollowState::LineGap);
      Serial.println("line_gap=start");
    }

    if (follow_state == FollowState::LineGap) {
      if (millis() - state_start_ms < LINE_GAP_FORWARD_MS) {
        drive_forward_gap();
        return;
      }

      Serial.println("line_gap=timeout_lost");
      set_follow_state(FollowState::LostLine);
    }

    recover_lost_line();
    return;
  }

  if (follow_state == FollowState::CrossIntersection) {
    if (millis() - state_start_ms < INTERSECTION_CROSS_MS) {
      drive_forward();
      return;
    }

    set_follow_state(FollowState::FollowLine);
  }

  if (special_junction && millis() - last_junction_detect_ms > 700) {
    last_junction_type = junction;
    last_junction_detect_ms = millis();

    Serial.print("junction=");
    Serial.println(junctionTypeName(junction));

    set_follow_state(FollowState::CrossIntersection);

    if (junction == JunctionType::LeftTurn) {
      Serial.println("Detected left turn.");
      centreAfterIR();
      turnLeft90WithLines();
      return;
    }

    if (junction == JunctionType::RightTurn) {
      Serial.println("Detected right turn.");
      centreAfterIR();
      turnRight90WithLines();
      return;
    }

    if (junction == JunctionType::TIntersection) {
      Serial.println("Detected T intersection.");
      centreAfterIR();
      turnRight90WithLines();
      return;
    }

    if (junction == JunctionType::WideIntersection) {
      Serial.println("Detected wide intersection.");
      centreAfterIR();
      turnRight90WithLines();
      return;
    }
  }

  last_error = error;

  set_follow_state(FollowState::FollowLine);
  follow_line(error);
}