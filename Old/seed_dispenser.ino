//servo test code
#include <Servo.h>

// ---- Pin assignments ----
const int SERVO1_PIN = 9;   // upper gate (isolates one seed)
const int SERVO2_PIN = 10;  // lower gate (releases into tube)

Servo gateUpper;
Servo gateLower;

// ---- Tune these once you find the right angles ----
int upperClosed = 90;
int upperOpen   = 0;
int lowerClosed = 0;
int lowerOpen   = 90;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  gateUpper.attach(SERVO1_PIN);
  gateLower.attach(SERVO2_PIN);

  // Start with both gates closed
  gateUpper.write(upperClosed);
  gateLower.write(lowerClosed);

  Serial.println(F("SG90 Seed Dispenser test"));
  Serial.println(F("Commands:"));
  Serial.println(F("  d           run a full dispense cycle"));
  Serial.println(F("  c           close both gates"));
}

// ---- Full dispense cycle: trap one seed, then drop it ----
void dispenseOne() {
  Serial.println(F("Dispense cycle..."));

  // 1. Open upper gate -> seed falls into the chamber
  gateUpper.write(upperOpen);
  delay(700);

  // 2. Close upper gate -> seed is trapped between gates
  gateUpper.write(upperClosed);
  delay(1000);

  // 3. Open lower gate -> seed drops out
  gateLower.write(lowerOpen);
  delay(700);

  // 4. Close lower gate -> ready for next seed
  gateLower.write(lowerClosed);
  delay(500);

  Serial.println(F("Done."));
}

void loop() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  if (cmd == 'd'){
    dispenseOne();
  } else if (cmd == 'c'){
    gateUpper.write(upperClosed);
    gateLower.write(lowerClosed);
    Serial.println(F("Both gates closed."));
  }
}
