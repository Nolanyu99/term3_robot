void plant()
{
  closeBothGates();

  centreAfterRFID();

  stopMotors();
  delay(50);

  dispenseOne();

  delay(50);
}

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

bool scanRFIDForTest()
{
  if (rfid.PICC_IsNewCardPresent() &&
      rfid.PICC_ReadCardSerial() &&
      millis() - lastScanTime >= RFID_SCAN_COOLDOWN_MS) {

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

bool driveUntilRFIDAndPlant(uint8_t target_scans)
{
  uint8_t scan_count = 0;

  Serial.print("line_follow_until_rfid target=");
  Serial.println(target_scans);

  run_enabled = true;
  set_follow_state(FollowState::FollowLine);

  while (scan_count < target_scans) {
    if (checkStopInputsDuringTest()) {
      return false;
    }

    Red();

    const unsigned long now_ms = millis();
    if (now_ms - last_control_ms >= CONTROL_INTERVAL_MS) {
      last_control_ms = now_ms;

      read_rc_discharge_times();
      update_line_following_no_junction_turns();
    }

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

      // After planting, return to line-following for the next RFID scan.
      set_follow_state(FollowState::FollowLine);
      last_control_ms = millis();
    }

    delay(2);
  }

  stopMotors();
  return true;
}

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
