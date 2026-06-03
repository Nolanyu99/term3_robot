// =====================================================
// Higher-level test behaviour for Control primarily
// Called in Main
// =====================================================

// Dispenses one seed and reports it to the server if possible
void plant()
{
  closeBothGates();

  centreAfterRFID();

  stopMotors();
  delay(50);

  dispenseOne();

  delay(50);
}

// Keeps buttons and remote safety responsive
bool checkStopInputsDuringTest()
{
  serviceServoPulses();
  update_turn_angle();

  if (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());

    if (command == '0' || command == 'x' || command == 'X') {
      running = false;
      run_enabled = false;
      stopMotors();
      Serial.println("test_stop=serial");
      return true;
    }
  }

  int OffButtonPressed = digitalRead(OffButtonPin);
  int RevButtonPressed = digitalRead(RevButtonPin);

  if (OffButtonPressed == LOW && previousOffButtonPressed != LOW) {
    running = !running;
    previousOffButtonPressed = OffButtonPressed;
    delay(50);

    if (!running) {
      run_enabled = false;
      stopMotors();
      Serial.println("test_stop=button");
      return true;
    }
  }

  previousOffButtonPressed = OffButtonPressed;

  if (RevButtonPressed == LOW) {
    Reviving = true;
    Green();
    stopMotors();
    Serial.println("test_pause=revive_button");

    while (digitalRead(RevButtonPin) == LOW) {
      serviceServoPulses();
      update_turn_angle();
      delay(5);
    }

    Reviving = false;
    delay(50);
  }

  return false;
}

//Scans RFID, recentres, and plants
bool scanRFIDForTest()
{
  if (rfid.PICC_IsNewCardPresent() &&
      rfid.PICC_ReadCardSerial() &&
      (lastScanTime == 0 || millis() - lastScanTime >= RFID_SCAN_COOLDOWN_MS)) {

    lastScanTime = millis();

    Serial.print("test_rfid_uid=");

    for (byte i = 0; i < rfid.uid.size; i++) {
      print_hex2(rfid.uid.uidByte[i]);
      Serial.print(' ');
    }

    Serial.println();

    rfid.PICC_HaltA();
    return true;
  }

  return false;
}

// Drives until a target number of RFID scans are completed
bool driveUntilRFIDAndPlant(uint8_t target_scans)
{
  uint8_t scan_count = 0;

  Serial.print("drive_until_rfid target=");
  Serial.println(target_scans);

  while (scan_count < target_scans) {
    if (checkStopInputsDuringTest()) {
      return false;
    }

    Red();

    if (scanRFIDForTest()) {
      stopMotors();
      delay(100);

      Serial.print("test_scan_number=");
      Serial.println(scan_count + 1);

      centreAfterRFID();

      if (seeds > 0) {
        dispenseOne();
        seeds--;
      } else {
        Serial.println("seed_warning=no_seeds_left");
      }

      scan_count++;

      stopMotors();
      delay(300);
      continue;
    }

    update_test3_line_following();

    delay(5);
  }

  stopMotors();
  return true;
}

// Line following for Test3 only.
// It follows the line but does NOT auto-turn at intersections or holes.
void update_test3_line_following()
{
  read_rc_discharge_times();
  update_calibrated_values();

  const bool found = update_line_found();

  if (!run_enabled) {
    set_follow_state(FollowState::Idle);
    stopMotors();
    return;
  }

  if (!found) {
    if (follow_state != FollowState::LineGap &&
        follow_state != FollowState::LostLine) {
      set_follow_state(FollowState::LineGap);
      Serial.println("test3_line_gap=start");
    }

    // First treat the missing line as the RFID hole/gap and continue forward.
    // If the line is still missing after that short grace period, use the
    // normal hesitant lost-line recovery, which now remembers the last turn.
    if (follow_state == FollowState::LineGap) {
      if (millis() - state_start_ms < LINE_GAP_FORWARD_MS) {
        drive_forward_gap();
        return;
      }

      Serial.println("test3_line_gap=timeout_lost");
      set_follow_state(FollowState::LostLine);
    }

    recover_lost_line();
    return;
  }

  update_last_line_side();

  if (follow_state == FollowState::LineGap ||
      follow_state == FollowState::LostLine) {
    Serial.println("test3_line_gap=recovered");
  }

  const int32_t position = estimate_line_position();

  if (position >= 0) {
    const int32_t error = position - LINE_CENTER;
    last_error = error;
    set_follow_state(FollowState::FollowLine);
    follow_line(error);
    return;
  }

  set_follow_state(FollowState::LostLine);
  recover_lost_line();
}

// Runs the scripted RFID-turn-plant route
void RunTest3()
{
  if (startup_cal_state != StartupCalState::Ready) {
    Serial.println("test3_error=not_calibrated");
    return;
  }

  if (!running) {
    Serial.println("test3_error=not_running");
    return;
  }

  if (!itg320x_ready) {
    Serial.println("test3_error=gyro_not_ready");
    stopMotors();
    return;
  }

  Serial.println("test3=start");

  run_enabled = true;
  Reviving = false;

  // Stage 1: drive forward until 2 RFID scans, planting each time.
  Serial.println("test3_stage=forward_until_2_scans");

  if (!driveUntilRFIDAndPlant(2)) {
    Serial.println("test3=aborted_stage1");
    return;
  }

  // Turn right 90 degrees.
  Serial.println("test3_stage=right_90");
  turnRight90();
  read_rc_discharge_times();
  update_calibrated_values();
  last_error = 0;
  last_line_side = 1;
  set_follow_state(FollowState::FollowLine);
  delay(100);

  if (checkStopInputsDuringTest()) {
    Serial.println("test3=aborted_after_right_turn");
    return;
  }

  // Stage 2: drive until 1 RFID scan, planting.
  Serial.println("test3_stage=forward_until_1_scan");

  if (!driveUntilRFIDAndPlant(1)) {
    Serial.println("test3=aborted_stage2");
    return;
  }

  // Turn left 90 degrees.
  Serial.println("test3_stage=left_90");
  turnLeft90();
  read_rc_discharge_times();
  update_calibrated_values();
  last_error = 0;
  last_line_side = -1;
  set_follow_state(FollowState::FollowLine);
  delay(100);

  if (checkStopInputsDuringTest()) {
    Serial.println("test3=aborted_after_left_turn");
    return;
  }

  // Stage 3: drive until 2 more RFID scans, planting each time.
  Serial.println("test3_stage=forward_until_2_more_scans");

  if (!driveUntilRFIDAndPlant(2)) {
    Serial.println("test3=aborted_stage3");
    return;
  }

  stopMotors();
  Green();

  Serial.println("test3=complete");
}

// =====================================================
// Test 8 - straight-line revive using the forward distance sensor
// =====================================================
// Server integration:
// - messagesLoop() stores revive target broadcasts by calling
//   test8AddOrUpdateReviveTarget(...).
// - A target is removed only by test8MarkReviveConfirmed(...), which is called
//   from a server revive confirmation message.
// - This version drives straight toward the object detected by the forward
//   ultrasonic sensor. If a line is visible, it uses the line sensors to steer.
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

bool test8UpdateCurrentPositionFromRFID()
{
  Serial.println("test8_stage=scan_rfid_for_current_position");

  const unsigned long start_ms = millis();

  while (millis() - start_ms < TEST8_POSITION_SCAN_TIMEOUT_MS) {
    if (checkStopInputsDuringTest() || !messagesRobotAllowedToMove()) {
      stopMotors();
      return false;
    }

    serviceServoPulses();
    update_turn_angle();
    messagesLoop();
    arenaRequestAndRefreshMap();

    if (arenaUpdatePoseFromRFID()) {
      Serial.print("test8_current_position x=");
      Serial.print(arena_pose.x);
      Serial.print(" y=");
      Serial.println(arena_pose.y);
      return true;
    }

    delay(5);
  }

  Serial.println("test8_error=current_position_rfid_timeout");
  return false;
}

int8_t test8FindNearestReviveTargetIndex()
{
  if (!arena_have_pose) {
    return -1;
  }

  int8_t best_index = -1;
  uint16_t best_cost = ARENA_INF_COST;
  uint8_t best_len = 255;

  for (uint8_t i = 0; i < TEST8_MAX_REVIVE_TARGETS; ++i) {
    if (!test8_revive_targets[i].active) {
      continue;
    }

    const int16_t manhattan = abs(test8_revive_targets[i].x - arena_pose.x) +
                              abs(test8_revive_targets[i].y - arena_pose.y);

    if (manhattan < best_cost) {
      best_cost = manhattan;
      best_len = static_cast<uint8_t>(min(manhattan, 254));
      best_index = static_cast<int8_t>(i);
    }
  }

  (void)best_len;
  return best_index;
}

bool test8NeighbourForTarget(const Test8ReviveTarget& target,
                             ArenaPoint* neighbour,
                             ArenaHeading* final_heading)
{
  if (neighbour == nullptr || final_heading == nullptr || !arena_have_pose) {
    return false;
  }

  const ArenaPoint candidates[4] = {
    {static_cast<int8_t>(target.x - 1), target.y},
    {static_cast<int8_t>(target.x + 1), target.y},
    {target.x, static_cast<int8_t>(target.y - 1)},
    {target.x, static_cast<int8_t>(target.y + 1)}
  };

  const ArenaHeading headings[4] = {
    ArenaHeading::East,
    ArenaHeading::West,
    ArenaHeading::South,
    ArenaHeading::North
  };

  int8_t best = -1;
  uint8_t best_path_len = 255;
  uint16_t best_manhattan = 65535;

  for (uint8_t i = 0; i < 4; ++i) {
    if (!arenaInside(candidates[i].x, candidates[i].y) ||
        arenaIsWallCell(candidates[i].x, candidates[i].y) ||
        arenaBlocked(candidates[i].x, candidates[i].y)) {
      continue;
    }

    ArenaPoint path[ARENA_CELL_COUNT];
    uint8_t path_len = 0;
    bool have_path = arenaFindPath(arena_pose, candidates[i], path, &path_len);

    const uint16_t manhattan = abs(candidates[i].x - arena_pose.x) +
                               abs(candidates[i].y - arena_pose.y);

    if (have_path && path_len > 0) {
      if (path_len < best_path_len ||
          (path_len == best_path_len && manhattan < best_manhattan)) {
        best = static_cast<int8_t>(i);
        best_path_len = path_len;
        best_manhattan = manhattan;
      }
    }
  }

  if (best < 0) {
    Serial.println("test8_error=no_reachable_neighbour_cell");
    return false;
  }

  *neighbour = candidates[best];
  *final_heading = headings[best];

  Serial.print("test8_neighbour_cell x=");
  Serial.print(neighbour->x);
  Serial.print(" y=");
  Serial.print(neighbour->y);
  Serial.print(" final_heading=");
  Serial.println(static_cast<uint8_t>(*final_heading));
  return true;
}

bool test8WaitForReviveConfirmation(const char* robot_id)
{
  Serial.println("test8_stage=wait_for_revive_confirmation");

  const unsigned long start_ms = millis();

  while (millis() - start_ms < TEST8_CONFIRM_TIMEOUT_MS) {
    if (checkStopInputsDuringTest() || !messagesRobotAllowedToMove()) {
      stopMotors();
      return false;
    }

    serviceServoPulses();
    update_turn_angle();
    messagesLoop();

    if (test8ReviveConfirmedRecently(robot_id)) {
      Serial.print("test8_result=revive_confirmed id=");
      Serial.println(robot_id);
      return true;
    }

    delay(10);
  }

  return false;
}

void test8ReverseAfterFailedRevive()
{
  Serial.println("test8_result=no_confirmation_reverse_and_stop");

  setMotors(-TEST8_REVERSE_SPEED, -TEST8_REVERSE_SPEED);

  const unsigned long start_ms = millis();
  while (millis() - start_ms < TEST8_REVERSE_AFTER_FAILED_REVIVE_MS) {
    serviceServoPulses();
    update_turn_angle();
    messagesLoop();
    delay(5);
  }

  stopMotors();
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
      // Once within 10 cm, keep moving slowly until the front revive button
      // is physically pressed by contact with the target.
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

  // Best effort: update our pose if we are already on a tag, but do not abort
  // if no tag is found. This version of Test 8 drives straight toward the
  // ultrasonic target rather than pathfinding to a known arena cell.
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
