#include <Wire.h>
#include <Motoron.h>
#include <math.h>

MotoronI2C mc;

// Motoron motor channels
const uint8_t MOTOR_LEFT  = 1;
const uint8_t MOTOR_RIGHT = 2;

// Max Motoron speed is usually -800 to +800
const int16_t MAX_SPEED_R = 800 * (1.4264/1.4681);
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

void checkpoint(const char *message)
{
  //Serial.println(message);
  //Serial.flush();   // Makes sure the message is sent before continuing
}

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

  checkpoint("28. Setup complete, motors should be running");

  delay(100);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  checkpoint("29. Pinmode Left A");
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  checkpoint("30. Pinmode Left B");
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  checkpoint("31. Pinmode Right A");
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  checkpoint("32. Pinmode Right B");

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), readLeftEncoder, CHANGE);
  checkpoint("33. Attach interrupt Left");
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), readRightEncoder, CHANGE);
  checkpoint("34. Attach interrupt Right");
  Serial.println("");
  Serial.print("Done");
  Serial.println("");
  Serial.println("");
}

void loop()
{

    if (millis() - lastPrintTime >= 1000) {
    lastPrintTime = millis();

    //Serial.print("Loop running. millis = ");
    Serial.print(millis());
    Serial.print(" ,");
    printEncoders();

    Serial.flush();
  }

  mc.setSpeed(MOTOR_LEFT, MAX_SPEED_L);
  mc.setSpeed(MOTOR_RIGHT, MAX_SPEED_R);
  delay(1000);
  mc.setSpeed(MOTOR_LEFT, 0);
  mc.setSpeed(MOTOR_RIGHT, 0);

  delay(1000);
}
