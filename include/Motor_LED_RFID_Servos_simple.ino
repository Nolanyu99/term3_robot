#include <Wire.h>
#include <Motoron.h>
#include <math.h>
#include <MFRC522_I2C.h>
#include <Arduino.h>


// Motor Section

MotoronI2C mc;

// Motoron motor channels
const uint8_t MOTOR_LEFT  = 1;
const uint8_t MOTOR_RIGHT = 2;

const double MOTOR_RATIO = 1.4264/1.4681;

// Max Motoron speed is usually -800 to +800
const int16_t MAX_SPEED_R = 800 * MOTOR_RATIO;
const int16_t MAX_SPEED_L = 800;

// Same acceleration/deceleration for both motors
const uint16_t ACCEL = 800;
const uint16_t DECEL = 800;

unsigned long lastPrintTime = 0;

const uint8_t ENC_LEFT_A  = 28;
const uint8_t ENC_LEFT_B  = 26;
const uint8_t ENC_RIGHT_A = 22;
const uint8_t ENC_RIGHT_B = 24;

const long wheel_diameter = 38.35;

volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;


// LED and Button section

const int redPin = 39;
const int greenPin = 35;
const int bluePin = 37;
const int OffButtonPin = 33;
const int RevButtonPin = 13;

int previousStateRed = 0;
bool running = true;
bool Reviving = false;
int previousOffButtonPressed = 1;


// RFID section

constexpr uint8_t RFID_ADDR = 0x28;

// RFID reader on SDA1/SCL1 using Wire1
MFRC522_I2C rfid(RFID_ADDR, -1, &Wire1);

//String scannedChips[81];


// Servos section

constexpr int SERVO1_PIN = 36;  // upper gate: isolates one seed
constexpr int SERVO2_PIN = 38;  // lower gate: releases into tube

constexpr int UPPER_CLOSED = 90;
constexpr int UPPER_OPEN = 0;
constexpr int LOWER_CLOSED = 0;
constexpr int LOWER_OPEN = 90;

constexpr int SERVO_MIN_PULSE_US = 500;
constexpr int SERVO_MAX_PULSE_US = 2500;
constexpr unsigned long SERVO_PERIOD_US = 20000;

int upper_pulse_us = 1500;
int lower_pulse_us = 1500;
int seeds = 5;
unsigned long last_servo_frame_us = 0;
unsigned long last_status_ms = 0;


// Helper Functions

void checkpoint(const char *message)
{
  //Serial.println(message);
  //Serial.flush();   // Makes sure the message is sent before continuing
}

// Motor Functions

void readLeftEncoder()
{
  if (digitalRead(ENC_LEFT_A) == digitalRead(ENC_LEFT_B)) {
    encoderLeftCount--;
  } else {
    encoderLeftCount++;
  }
}

void readRightEncoder()
{
  if (digitalRead(ENC_RIGHT_A) == digitalRead(ENC_RIGHT_B)) {
    encoderRightCount++;
  } else {
    encoderRightCount--;
  }
}

long getLeftEncoder()
{
  noInterrupts();
  long value = encoderLeftCount;
  interrupts();
  return value;
}

long getRightEncoder()
{
  noInterrupts();
  long value = encoderRightCount;
  interrupts();
  return value;
}

void printEncoders()
{
  Serial.print("Left encoder: ");
  Serial.print(getLeftEncoder());
  Serial.print(", ");
  Serial.print(" | Right encoder: ");
  Serial.println(getRightEncoder());
  Serial.print("Error: ");
  Serial.println(getLeftEncoder() - getRightEncoder());
  Serial.print("Estimated distance traveled (L): ");
  Serial.println((getLeftEncoder() * M_PI * wheel_diameter) / (12 * 100));
  Serial.print("Estimated distance traveled (R): ");
  Serial.println((getRightEncoder() * M_PI * wheel_diameter) / (12 * 100));
}


// LED Functions

void Flash(int previousStateRed) {
  if (previousStateRed % 20 <= 9) {
    Serial.print("Red\n");
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
  }
  else {
    Serial.print("Off\n");
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
  }
}


// RFID Functions

bool rfid_firmware_version_valid(byte version) {
  return version == 0x15 ||
         version == 0x90 ||
         version == 0x91 ||
         version == 0x92 ||
         version == 0xB2;
}

void print_hex2(byte value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void init_reader_without_soft_reset() {
  // Avoids PCD_Init(), which may freeze on some WS1850S / MFRC522-compatible modules
  rfid.PCD_WriteRegister(rfid.TModeReg, 0x80);
  rfid.PCD_WriteRegister(rfid.TPrescalerReg, 0xA9);
  rfid.PCD_WriteRegister(rfid.TReloadRegH, 0x03);
  rfid.PCD_WriteRegister(rfid.TReloadRegL, 0xE8);
  rfid.PCD_WriteRegister(rfid.TxASKReg, 0x40);
  rfid.PCD_WriteRegister(rfid.ModeReg, 0x3D);
  rfid.PCD_AntennaOn();
}

bool i2c_device_present(uint8_t address) {
  Wire1.beginTransmission(address);
  return Wire1.endTransmission() == 0;
}

// Servo functions

int angleToPulseUs(int angle) {
    angle = constrain(angle, 0, 180);
    return map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
}

void serviceServoPulses() {
    const unsigned long now = micros();
    if (now - last_servo_frame_us < SERVO_PERIOD_US) {
        return;
    }
    last_servo_frame_us = now;

    digitalWrite(SERVO1_PIN, HIGH);
    delayMicroseconds(upper_pulse_us);
    digitalWrite(SERVO1_PIN, LOW);

    digitalWrite(SERVO2_PIN, HIGH);
    delayMicroseconds(lower_pulse_us);
    digitalWrite(SERVO2_PIN, LOW);
}

void waitWithServo(unsigned long ms) {
    const unsigned long start = millis();
    while (millis() - start < ms) {
        serviceServoPulses();
        delay(1);
    }
}

void writeUpperServo(int angle) {
    upper_pulse_us = angleToPulseUs(angle);
}

void writeLowerServo(int angle) {
    lower_pulse_us = angleToPulseUs(angle);
}

void closeBothGates() {
    writeUpperServo(UPPER_CLOSED);
    writeLowerServo(LOWER_CLOSED);
}

void dispenseOne() {
    Serial.println(F("Dispense cycle..."));

    writeUpperServo(UPPER_OPEN);
    waitWithServo(700);

    writeUpperServo(UPPER_CLOSED);
    waitWithServo(1000);

    writeLowerServo(LOWER_OPEN);
    waitWithServo(700);

    writeLowerServo(LOWER_CLOSED);
    waitWithServo(500);

    Serial.println(F("Done."));
}

// Alorithm functions

void plant() {
  closeBothGates();
  while ((getLeftEncoder() < 1200) || (getRightEncoder()  < 1200)) {
    mc.setSpeed(MOTOR_LEFT, 400);
    mc.setSpeed(MOTOR_RIGHT, 400 * MOTOR_RATIO);
  }
  mc.setSpeed(MOTOR_LEFT, 0);
  mc.setSpeed(MOTOR_RIGHT, 0);
  delay(50);
  dispenseOne();  
}

void setup()
{
  Serial.begin(115200);

  // Useful for boards with native USB, like Arduino GIGA
  unsigned long startTime = millis();
  while (!Serial && millis() - startTime < 5000) {
    // Wait up to 5 seconds for Serial Monitor
  }

  delay(1000);

  checkpoint("1. Starting setup");

  checkpoint("2. Starting Wire");
  Wire1.begin();
  checkpoint("3. Wire.begin done");

  checkpoint("4. Setting I2C clock");
  Wire1.setClock(100000);
  checkpoint("5. I2C clock set");

  checkpoint("6. Reinitializing Motoron");
  mc.setBus(&Wire1);
  mc.reinitialize();
  checkpoint("7. Motoron reinitialize done");

  delay(10);

  checkpoint("8. Disabling CRC");
  mc.disableCrc();
  checkpoint("9. CRC disabled");

  delay(10);

  checkpoint("10. Clearing reset flag");
  mc.clearResetFlag();
  checkpoint("11. Reset flag cleared");

  checkpoint("12. Clearing motor fault");
  mc.clearMotorFaultUnconditional();
  checkpoint("13. Motor fault cleared");

  checkpoint("14. Setting command timeout");
  mc.setCommandTimeoutMilliseconds(2000);
  checkpoint("15. Command timeout set");

 // Motors
  checkpoint("16. Setting left acceleration");
  mc.setMaxAcceleration(MOTOR_LEFT, ACCEL);
  checkpoint("17. Left acceleration set");

  checkpoint("18. Setting left deceleration");
  mc.setMaxDeceleration(MOTOR_LEFT, DECEL);
  checkpoint("19. Left deceleration set");

  checkpoint("20. Setting right acceleration");
  mc.setMaxAcceleration(MOTOR_RIGHT, ACCEL);
  checkpoint("21. Right acceleration set");

  checkpoint("22. Setting right deceleration");
  mc.setMaxDeceleration(MOTOR_RIGHT, DECEL);
  checkpoint("23. Right deceleration set");

  checkpoint("24. Setting left motor speed");
  mc.setSpeed(MOTOR_LEFT, MAX_SPEED_L);
  checkpoint("25. Left motor speed set");

  checkpoint("26. Setting right motor speed");
  mc.setSpeed(MOTOR_RIGHT, MAX_SPEED_R);
  checkpoint("27. Right motor speed set");

 // Encoders
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  checkpoint("28. Pinmode Left A");
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  checkpoint("29. Pinmode Left B");
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  checkpoint("30. Pinmode Right A");
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  checkpoint("31. Pinmode Right B");

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), readLeftEncoder, CHANGE);
  checkpoint("32. Attach interrupt Left");
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), readRightEncoder, CHANGE);
  checkpoint("33. Attach interrupt Right");
  Serial.println("");
  Serial.print("Done");
  Serial.println("");
  Serial.println("");

 // LED
  pinMode(redPin, OUTPUT);
  checkpoint("34. Red pinmode");
  pinMode(greenPin, OUTPUT);
  checkpoint("35. Green pinmode");
  pinMode(bluePin, OUTPUT);
  checkpoint("36. Blue pinmode");

 // Buttons 
  pinMode(OffButtonPin, INPUT_PULLUP);
  checkpoint("37. Off button pinmode");
  pinMode(RevButtonPin, INPUT_PULLUP);
  checkpoint("38. Revive button pinmode");
  
 // RFID
   if (i2c_device_present(RFID_ADDR)) {
    checkpoint("39. RDIF found at at 0x28");
  }
  else {
    Serial.println("No I2C device found at 0x28 on SDA1/SCL1");
    Serial.println("Check VCC, GND, SDA1, and SCL1 wiring");
    return;
  }

  byte version = rfid.PCD_ReadRegister(rfid.VersionReg);
  if (rfid_firmware_version_valid(version)) {
    checkpoint("40. RFID version compatible");
  }
  else {
    Serial.println("Device responded, but firmware version is not recognised as MFRC522-compatible");
    return;
  }

  init_reader_without_soft_reset();
  checkpoint("41. initiating reader without soft reset");

  // Servos
  pinMode(SERVO1_PIN, OUTPUT);
  checkpoint("42. pinmode servo 1");
  pinMode(SERVO2_PIN, OUTPUT);
  checkpoint("43. pinmode servo 2");
  digitalWrite(SERVO1_PIN, LOW);
  checkpoint("44. lower servo 2 signal");
  digitalWrite(SERVO2_PIN, LOW);
  checkpoint("45. lower servo 1 signal");

  closeBothGates();
  checkpoint("46. closed both servos");
  waitWithServo(500);

  Serial.println(F("SG90 Seed Dispenser test"));
  Serial.println(F("Commands:"));
  Serial.println(F("  d           run a full dispense cycle"));
  Serial.println(F("  c           close both gates"));

  delay(2000);
  Serial.println("starting...");
}

void loop()
{

  int OffButtonPressed = digitalRead(OffButtonPin);
  /*Serial.print("OffButtonPressed ");
  Serial.println(OffButtonPressed);
  Serial.print("previousOffButtonPressed: ");
  Serial.println(previousOffButtonPressed);*/

  int RevButtonPressed = digitalRead(RevButtonPin); 
  /*Serial.print("RevButtonPressed ");
  Serial.println(RevButtonPressed);

  Serial.println("------------");*/

  Reviving = false;

  if (OffButtonPressed == 0 && previousOffButtonPressed != 0) {
    running = !running;
    delay(50);
  }
  else if (RevButtonPressed == 0) {
    Reviving = true;
    delay(50);
  }

  if (!running) {
    Flash(previousStateRed);
    previousStateRed++;
    mc.setSpeed(MOTOR_LEFT, 0);
    mc.setSpeed(MOTOR_RIGHT, 0);
    delay(25);
  }

  else {
    if (Reviving) {
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, HIGH);
      digitalWrite(bluePin, LOW);

      mc.setSpeed(MOTOR_LEFT, 0);
      mc.setSpeed(MOTOR_RIGHT, 0);

      delay(500);
    }

    else if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      Serial.print("UID: ");

      for (byte i = 0; i < rfid.uid.size; i++) {
        print_hex2(rfid.uid.uidByte[i]);
        Serial.print(' ');
      }
      mc.setSpeed(MOTOR_LEFT, 0);
      mc.setSpeed(MOTOR_RIGHT, 0);
      delay(100);

      rfid.PICC_HaltA();

      if (seeds > 0) {
        plant();
        seeds--;
      }
    }
      
    else {
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
      //delay(50);

      if (millis() - lastPrintTime >= 1000) {
        lastPrintTime = millis();

        /*Serial.print("Loop running. millis = ");
        Serial.print(millis());
        Serial.print(" ,");
        printEncoders();

        Serial.flush();*/
      }

      mc.setSpeed(MOTOR_LEFT, 300);
      mc.setSpeed(MOTOR_RIGHT, 300 * MOTOR_RATIO);
    }    
  }

  previousOffButtonPressed = OffButtonPressed;
}