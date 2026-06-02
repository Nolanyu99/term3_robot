// =====================================================
// Software-timed seed-dispenser servo control
// Used by Main and RobotBehavior
// =====================================================

//Converts angle to pulse width
int angleToPulseUs(int angle)
{
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
}

// Generates software servo pulses without Servo library timers
void serviceServoPulses()
{
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

// Waits while continuing servo pulses and messaging
void waitWithServo(unsigned long ms)
{
  const unsigned long start = millis();

  while (millis() - start < ms) {
    serviceServoPulses();
    delay(1);
  }
}

// Sets upper gate target angle
void writeUpperServo(int angle)
{
  upper_pulse_us = angleToPulseUs(angle);
}

// Sets lower gate target angle
void writeLowerServo(int angle)
{
  lower_pulse_us = angleToPulseUs(angle);
}

// Closes both seed gates
void closeBothGates()
{
  writeUpperServo(UPPER_CLOSED);
  writeLowerServo(LOWER_CLOSED);
}

// Runs one upper/lower gate cycle
void dispenseOne()
{
  Serial.println(F("Dispense cycle..."));

  writeUpperServo(UPPER_OPEN);
  waitWithServo(robot_config::SEED_UPPER_OPEN_MS);

  writeUpperServo(UPPER_CLOSED);
  waitWithServo(robot_config::SEED_UPPER_CLOSE_MS);

  writeLowerServo(LOWER_OPEN);
  waitWithServo(robot_config::SEED_LOWER_OPEN_MS);

  writeLowerServo(LOWER_CLOSED);
  waitWithServo(robot_config::SEED_LOWER_CLOSE_MS);

  Serial.println(F("Done."));
}

// Prepares seed servos and closes gates
void setupServos()
{
  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);

  digitalWrite(SERVO1_PIN, LOW);
  digitalWrite(SERVO2_PIN, LOW);

  closeBothGates();
  waitWithServo(500);
}