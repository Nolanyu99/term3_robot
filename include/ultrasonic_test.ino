//Ultrasonic sensor

const int TRIG_PIN = 22;
const int ECHO_PIN = 23;

// Maximum sensible distance to wait for an echo (in cm)
// Beyond this, treat it as "out of range"
const float MAX_DISTANCE_CM = 400.0;
const unsigned long ECHO_TIMEOUT_US = (MAX_DISTANCE_CM * 2 * 29.1);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  Serial.println(F("HC-SR04 distance test"));
  delay(100);
}

float readDistanceCm() {
  // Send a clean 10 us HIGH pulse on Trig
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure how long Echo stays HIGH (in microseconds)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

  if (duration == 0) {
    return -1.0;   // timed out / out of range
  }

  // Convert time to distance: speed of sound = 0.0343 cm/us, /2 for round trip
  float distanceCm = (duration * 0.0343) / 2.0;
  return distanceCm;
}

void loop() {
  float d = readDistanceCm();

  if (d < 0) {
    Serial.println(F("Out of range"));
  } else if (d < 20){
    Serial.print(F("Distance: "));
    Serial.print(d, 1);
    Serial.println(F(" cm"));
    Serial.println(F(" Object: Wall"));
  } else {
    Serial.print(F("Distance: "));
    Serial.print(d, 1);
    Serial.println(F(" cm"));
  }

  delay(100);   // 10 readings per second
}
