const uint8_t SensorCount = 9;
const uint8_t sensorPins[SensorCount] = {2, 3, 4, 5, 6, 7, 8, 22, 23};

const uint16_t TIMEOUT_US = 2500;   // max time to wait for discharge

uint16_t sensorValues[SensorCount];

void readQTR() {
  // 1. Charge all sensor capacitors by driving pins HIGH
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }
  delayMicroseconds(10);   // charge time

  // 2. Switch all pins to INPUT (releases them to discharge)
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // 3. Time how long each pin takes to go LOW
  uint32_t startTime = micros();
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorValues[i] = TIMEOUT_US;   // assume max if it never discharges
  }

  while ((micros() - startTime) < TIMEOUT_US) {
    uint32_t elapsed = micros() - startTime;
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] == TIMEOUT_US && digitalRead(sensorPins[i]) == LOW) {
        sensorValues[i] = elapsed;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println(F("Custom QTR RC reader"));
  Serial.println(F("Values: low ~50 = bright, high ~2500 = dark"));
}

void loop() {
  readQTR();

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(100);
}
