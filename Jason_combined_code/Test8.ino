// =====================================================
// Test 8 - straight-line revive using the forward distance sensor
// =====================================================
// Provides:
//   - test8AddOrUpdateReviveTarget / test8MarkReviveConfirmed
//     (called by Messages.ino when revive broadcasts arrive)
//   - RunTest8(): scripted revive flow
//
// Server integration:
// - messagesLoop() stores revive target broadcasts by calling
//   test8AddOrUpdateReviveTarget(...).
// - A target is removed only by test8MarkReviveConfirmed(...), which is called
//   from a server revive confirmation message.
// - Drives straight toward the object detected by the forward ultrasonic
//   sensor. If a line is visible, it uses the line sensors to steer.
// - At 10 cm it slows down, starts reviving when contact/close range is reached,
//   waits 5 seconds, then reverses until the nearest RFID tag is scanned.

constexpr uint8_t TEST8_MAX_REVIVE_TARGETS = 10;
constexpr int16_t TEST8_APPROACH_SPEED = 500;
constexpr int16_t TEST8_SLOW_APPROACH_SPEED = 300;
constexpr int16_t TEST8_REVERSE_SPEED = 130;
constexpr float TEST8_SLOW_DISTANCE_MM = 100.0f;      // 10 cm
constexpr unsigned long TEST8_POSITION_SCAN_TIMEOUT_MS = 10000UL;
constexpr unsigned long TEST8_CONFIRM_TIMEOUT_MS = 10000UL;
constexpr unsigned long TEST8_REVIVE_HOLD_MS = 5000UL;
constexpr unsigned long TEST8_OBJECT_TIMEOUT_MS = 25000UL;
constexpr unsigned long TEST8_REVERSE_RFID_TIMEOUT_MS = 12000UL;
constexpr unsigned long TEST8_REVERSE_AFTER_FAILED_REVIVE_MS = 700UL;
constexpr unsigned long TEST8_DISTANCE_PRINT_INTERVAL_MS = 250UL;


Test8ReviveTarget test8_revive_targets[TEST8_MAX_REVIVE_TARGETS];
char test8_last_confirmed_robot_id[16] = {};
unsigned long test8_last_confirmed_ms = 0;

bool test8RobotIdMatches(const char* a, const char* b)
{
  return a != nullptr && b != nullptr && strcmp(a, b) == 0;
}

void test8AddOrUpdateReviveTarget(const char* robot_id, int8_t x, int8_t y)
{
  if (robot_id == nullptr || robot_id[0] == '\0') {
    Serial.println("test8_warning=empty_revive_robot_id");
    return;
  }

  if (!arenaInside(x, y) || arenaIsWallCell(x, y)) {
    Serial.println("test8_warning=invalid_revive_target_cell");
    return;
  }

  for (uint8_t i = 0; i < TEST8_MAX_REVIVE_TARGETS; ++i) {
    if (test8_revive_targets[i].active &&
        test8RobotIdMatches(test8_revive_targets[i].robot_id, robot_id)) {
      test8_revive_targets[i].x = x;
      test8_revive_targets[i].y = y;

      Serial.print("test8_revive_target_updated id=");
      Serial.print(robot_id);
      Serial.print(" x=");
      Serial.print(x);
      Serial.print(" y=");
      Serial.println(y);
      return;
    }
  }

  for (uint8_t i = 0; i < TEST8_MAX_REVIVE_TARGETS; ++i) {
    if (!test8_revive_targets[i].active) {
      strncpy(test8_revive_targets[i].robot_id, robot_id, sizeof(test8_revive_targets[i].robot_id) - 1);
      test8_revive_targets[i].robot_id[sizeof(test8_revive_targets[i].robot_id) - 1] = '\0';
      test8_revive_targets[i].x = x;
      test8_revive_targets[i].y = y;
      test8_revive_targets[i].active = true;

      Serial.print("test8_revive_target_added id=");
      Serial.print(test8_revive_targets[i].robot_id);
      Serial.print(" x=");
      Serial.print(x);
      Serial.print(" y=");
      Serial.println(y);
      return;
    }
  }

  Serial.println("test8_warning=revive_target_list_full");
}

void test8MarkReviveConfirmed(const char* robot_id)
{
  if (robot_id == nullptr || robot_id[0] == '\0') {
    Serial.println("test8_warning=empty_revive_confirmation_id");
    return;
  }

  strncpy(test8_last_confirmed_robot_id, robot_id, sizeof(test8_last_confirmed_robot_id) - 1);
  test8_last_confirmed_robot_id[sizeof(test8_last_confirmed_robot_id) - 1] = '\0';
  test8_last_confirmed_ms = millis();

  for (uint8_t i = 0; i < TEST8_MAX_REVIVE_TARGETS; ++i) {
    if (test8_revive_targets[i].active &&
        test8RobotIdMatches(test8_revive_targets[i].robot_id, robot_id)) {
      test8_revive_targets[i].active = false;

      Serial.print("test8_revive_confirmed id=");
      Serial.println(robot_id);
      return;
    }
  }

  Serial.print("test8_revive_confirmed_unknown_target id=");
  Serial.println(robot_id);
}

bool test8ReviveConfirmedRecently(const char* robot_id)
{
  return robot_id != nullptr &&
         test8RobotIdMatches(test8_last_confirmed_robot_id, robot_id) &&
         millis() - test8_last_confirmed_ms <= TEST8_CONFIRM_TIMEOUT_MS;
}

bool test8AnyReviveTargets()
{
  for (uint8_t i = 0; i < TEST8_MAX_REVIVE_TARGETS; ++i) {
    if (test8_revive_targets[i].active) {
      return true;
    }
  }

  return false;
}

int8_t test8FindNearestReviveTargetIndex()
{
  if (!arena_have_pose) {
    return -1;
  }

  int8_t best_index = -1;
  uint16_t best_cost = ARENA_INF_COST;

  for (uint8_t i = 0; i < TEST8_MAX_REVIVE_TARGETS; ++i) {
    if (!test8_revive_targets[i].active) {
      continue;
    }

    const int16_t manhattan = abs(test8_revive_targets[i].x - arena_pose.x) +
                              abs(test8_revive_targets[i].y - arena_pose.y);

    if (manhattan < best_cost) {
      best_cost = manhattan;
      best_index = static_cast<int8_t>(i);
    }
  }

  return best_index;
}

const char* test8BestKnownTargetId()
{
  if (arena_have_pose) {
    const int8_t target_index = test8FindNearestReviveTargetIndex();

    if (target_index >= 0) {
      return test8_revive_targets[target_index].robot_id;
    }
  }

  for (uint8_t i = 0; i < TEST8_MAX_REVIVE_TARGETS; ++i) {
    if (test8_revive_targets[i].active) {
      return test8_revive_targets[i].robot_id;
    }
  }

  return nullptr;
}

bool test8StopRequested()
{
  if (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());

    if (command == '0' || command == 'x' || command == 'X') {
      stopMotors();
      running = false;
      run_enabled = false;
      Serial.println("test8_stop=serial");
      return true;
    }
  }

  const int OffButtonPressed = digitalRead(OffButtonPin);

  if (OffButtonPressed == LOW && previousOffButtonPressed != LOW) {
    previousOffButtonPressed = OffButtonPressed;
    stopMotors();
    running = false;
    run_enabled = false;
    Serial.println("test8_stop=button");
    return true;
  }

  previousOffButtonPressed = OffButtonPressed;

  if (!messagesRobotAllowedToMove()) {
    stopMotors();
    Serial.println("test8_stop=remote_disabled");
    return true;
  }

  return false;
}

void test8DriveStraightOrFollowLine(int16_t base_speed)
{
  read_rc_discharge_times();
  update_calibrated_values();

  const bool found = update_line_found();

  if (found) {
    update_last_line_side();
    const int32_t position = estimate_line_position();

    if (position >= 0) {
      const int32_t error = position - LINE_CENTER;
      const int16_t correction = static_cast<int16_t>(LINE_KP * static_cast<float>(error));
      const int16_t left_speed = LEFT_FORWARD_SIGN * constrain(base_speed + correction, -500, 500);
      const int16_t right_speed = RIGHT_FORWARD_SIGN * constrain(base_speed - correction, -500, 500);
      last_error = error;
      set_follow_state(FollowState::FollowLine);
      setMotors(left_speed, right_speed);
      return;
    }
  }

  setMotors(
    LEFT_FORWARD_SIGN * base_speed,
    RIGHT_FORWARD_SIGN * base_speed
  );
}

void test8HoldReviveForFiveSeconds(const char* robot_id)
{
  stopMotors();
  Green();
  Reviving = true;

  Serial.println("test8_stage=revive_hold_5_seconds");

  if (robot_id != nullptr && robot_id[0] != '\0') {
    Serial.print("test8_revive_started id=");
    Serial.println(robot_id);
    messagesSendReviveRequest(MESSAGE_TEAM_ID, robot_id);
  } else {
    Serial.println("test8_revive_started id=unknown_no_server_target");
  }

  const unsigned long start_ms = millis();

  while (millis() - start_ms < TEST8_REVIVE_HOLD_MS) {
    serviceServoPulses();
    update_turn_angle();
    messagesLoop();

    if (test8StopRequested()) {
      break;
    }

    delay(10);
  }

  Reviving = false;
  Serial.println("test8_stage=revive_hold_done");
}

bool test8ReverseUntilRFID()
{
  Serial.println("test8_stage=reverse_until_closest_rfid");

  const unsigned long start_ms = millis();

  while (millis() - start_ms < TEST8_REVERSE_RFID_TIMEOUT_MS) {
    serviceServoPulses();
    update_turn_angle();
    messagesLoop();

    if (test8StopRequested()) {
      return false;
    }

    setMotors(
      -LEFT_FORWARD_SIGN * TEST8_REVERSE_SPEED,
      -RIGHT_FORWARD_SIGN * TEST8_REVERSE_SPEED
    );

    if (arenaUpdatePoseFromRFID() || scanRFIDForTest()) {
      stopMotors();
      Serial.println("test8_result=rfid_found_after_reverse");
      return true;
    }

    delay(5);
  }

  stopMotors();
  Serial.println("test8_warning=reverse_rfid_timeout");
  return false;
}

bool test8ApproachObjectAndRevive(const char* robot_id)
{
  Serial.println("test8_stage=straight_line_object_approach");

  bool reached_10cm = false;
  unsigned long last_print_ms = 0;
  const unsigned long start_ms = millis();

  while (millis() - start_ms < TEST8_OBJECT_TIMEOUT_MS) {
    serviceServoPulses();
    update_turn_angle();
    messagesLoop();

    if (test8StopRequested()) {
      return false;
    }

    const float distance_mm = forward_distance_mm();
    const bool distance_valid = distance_mm > 0.0f && distance_mm < 2000.0f;

    if (millis() - last_print_ms >= TEST8_DISTANCE_PRINT_INTERVAL_MS) {
      last_print_ms = millis();
      Serial.print("test8_distance_mm=");
      Serial.print(distance_mm);

      if (!distance_valid) {
        Serial.print(" invalid");
      }

      Serial.println();
    }

    if (!reached_10cm && distance_valid && distance_mm <= TEST8_SLOW_DISTANCE_MM) {
      reached_10cm = true;
      Serial.println("test8_stage=within_10cm_slowing_down");
    }

    if (reached_10cm) {
      if (digitalRead(RevButtonPin) == LOW) {
        Serial.println("test8_stage=revive_button_pressed_contact_reached");
        test8HoldReviveForFiveSeconds(robot_id);
        return true;
      }

      test8DriveStraightOrFollowLine(TEST8_SLOW_APPROACH_SPEED);
    } else {
      test8DriveStraightOrFollowLine(TEST8_APPROACH_SPEED);
    }

    delay(5);
  }

  stopMotors();
  Serial.println("test8_error=object_approach_timeout");
  return false;
}

void RunTest8()
{
  if (startup_cal_state != StartupCalState::Ready) {
    Serial.println("test8_error=not_calibrated");
    return;
  }

  if (!running) {
    Serial.println("test8_error=not_running");
    return;
  }

  if (!itg320x_ready) {
    Serial.println("test8_error=gyro_not_ready");
    stopMotors();
    return;
  }

  Serial.println("test8=start_straight_object_rescue");

  run_enabled = true;
  Reviving = false;
  stopMotors();
  set_follow_state(FollowState::FollowLine);

  for (uint8_t i = 0; i < 25; ++i) {
    messagesLoop();
    delay(5);
  }

  arenaUpdatePoseFromRFID();

  const char* robot_id = test8BestKnownTargetId();

  if (robot_id != nullptr) {
    Serial.print("test8_selected_target id=");
    Serial.println(robot_id);
  } else {
    Serial.println("test8_selected_target=none_using_distance_sensor_only");
  }

  const bool revived = test8ApproachObjectAndRevive(robot_id);

  stopMotors();

  if (!revived) {
    Yellow();
    Serial.println("test8=stopped_before_revive");
    return;
  }

  test8ReverseUntilRFID();
  stopMotors();
  Green();
  Serial.println("test8=complete");
}
