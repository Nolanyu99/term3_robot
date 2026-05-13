//Rfid test
#include <Wire.h>
#include <MFRC522_I2C.h>

// WS1850S default I2C address is 0x28 on the M5Stack RFID2 unit.
// If nothing is detected, try 0x3C — some clones use that instead.
#define RFID_ADDR 0x28

MFRC522_I2C mfrc522(RFID_ADDR, -1);  // -1 = no reset pin wired

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // wait briefly for USB serial

  Wire.begin();          // start I2C as master
  mfrc522.PCD_Init();    // init the reader

  Serial.println(F("WS1850S RFID test"));
  Serial.print(F("Firmware version: 0x"));
  byte v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  Serial.println(v, HEX);   // expect 0x90 / 0x91 / 0x92, or 0xB2 for clones
  Serial.println(F("Place a tag near the reader..."));
}

void loop() {
  // Look for a new card
  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial())   return;

  // Print UID
  Serial.print(F("UID: "));
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) Serial.print('0');
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  mfrc522.PICC_HaltA();   // stop reading this card so we can detect a new tap
  delay(300);
}
