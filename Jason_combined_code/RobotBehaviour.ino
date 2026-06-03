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

// RunTest3 and driveUntilRFIDAndPlant were removed — the scripted
// 2-scan / turn-right / 1-scan / turn-left / 2-scan course no longer
// serves a purpose now that the arena pathfinder + isFertile server
// query handle planting properly. plant(), checkStopInputsDuringTest(),
// and scanRFIDForTest() are still used elsewhere (RFID.ino, Test8.ino).
