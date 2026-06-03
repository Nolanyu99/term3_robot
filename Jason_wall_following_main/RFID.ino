bool rfid_firmware_version_valid(byte version)
{
  return version == 0x15 ||
         version == 0x90 ||
         version == 0x91 ||
         version == 0x92 ||
         version == 0xB2;
}

void print_hex2(byte value)
{
  if (value < 0x10) {
    Serial.print('0');
  }

  Serial.print(value, HEX);
}

void init_reader_without_soft_reset()
{
  rfid.PCD_WriteRegister(rfid.TModeReg, 0x80);
  rfid.PCD_WriteRegister(rfid.TPrescalerReg, 0xA9);
  rfid.PCD_WriteRegister(rfid.TReloadRegH, 0x03);
  rfid.PCD_WriteRegister(rfid.TReloadRegL, 0xE8);
  rfid.PCD_WriteRegister(rfid.TxASKReg, 0x40);
  rfid.PCD_WriteRegister(rfid.ModeReg, 0x3D);
  rfid.PCD_AntennaOn();
}

bool i2c_device_present(uint8_t address)
{
  Wire1.beginTransmission(address);
  return Wire1.endTransmission() == 0;
}

void setupRFID()
{
  if (i2c_device_present(RFID_ADDR)) {
    Serial.println("RFID found at 0x28 on SDA1/SCL1");
  } else {
    Serial.println("No I2C device found at 0x28 on SDA1/SCL1");
    Serial.println("Check VCC, GND, SDA1, and SCL1 wiring");
    return;
  }

  byte version = rfid.PCD_ReadRegister(rfid.VersionReg);

  if (rfid_firmware_version_valid(version)) {
    Serial.print("RFID firmware version OK: 0x");
    print_hex2(version);
    Serial.println();
  } else {
    Serial.print("Device responded, but firmware version is not recognised: 0x");
    print_hex2(version);
    Serial.println();
    return;
  }

  init_reader_without_soft_reset();
}

bool handleRFID()
{
  if (rfid.PICC_IsNewCardPresent() &&
      rfid.PICC_ReadCardSerial() &&
      millis() - lastScanTime >= RFID_SCAN_COOLDOWN_MS) {

    Serial.print("UID: ");

    for (byte i = 0; i < rfid.uid.size; i++) {
      print_hex2(rfid.uid.uidByte[i]);
      Serial.print(' ');
    }

    Serial.println();

    stopMotors();
    delay(100);

    rfid.PICC_HaltA();

    if (seeds > 0) {
      plant();
      lastScanTime = millis();
      seeds--;
    }

    return true;
  }

  return false;
}
