// ==========================================================================
// Encoder interrupts, Motoron setup, and basic drive
// Used by Main, Turning, Tunnel_Entry, RobotBehaviour, and ArenaPathfinding
// ==========================================================================

// Counts left encoder changes
void readLeftEncoder()
{
  if (digitalRead(ENC_LEFT_A) == digitalRead(ENC_LEFT_B)) {
    encoderLeftCount--;
  } else {
    encoderLeftCount++;
  }
}

// Counts right encoder changes
void readRightEncoder()
{
  if (digitalRead(ENC_RIGHT_A) == digitalRead(ENC_RIGHT_B)) {
    encoderRightCount++;
  } else {
    encoderRightCount--;
  }
}

// Reads left count safely
long getLeftEncoder()
{
  noInterrupts();
  long value = encoderLeftCount;
  interrupts();
  return value;
}

// Reads right count safely
long getRightEncoder()
{
  noInterrupts();
  long value = encoderRightCount;
  interrupts();
  return value;
}

//Prints both encoder counts
void printEncoders()
{
  const long left = getLeftEncoder();
  const long right = getRightEncoder();

  Serial.print(" Left encoder=");
  Serial.print(left);
  Serial.print(" Right encoder=");
  Serial.print(right);
}

// Sends signed left/right speeds to the Motoron
void setMotors(int16_t leftSpeed, int16_t rightSpeed)
{
  leftSpeed  = constrain(leftSpeed,  -MAX_SPEED_L, MAX_SPEED_L);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED_R, MAX_SPEED_R);

  last_left_command = leftSpeed;
  last_right_command = rightSpeed;

  if (!motoron_ready) {
    return;
  }

  mc.setSpeed(MOTOR_LEFT, leftSpeed);
  mc.setSpeed(MOTOR_RIGHT, static_cast<int16_t>(rightSpeed * MOTOR_RATIO));

  if (mc.getLastError() != 0) {
    motoron_ready = false;
    Serial.print("motoron_error=");
    Serial.println(mc.getLastError());
  }
}

// Stops both drive motors
void stopMotors()
{
  setMotors(0, 0);
}

// Applies acceleration/deceleration limits
void configureMotor(uint8_t motor)
{
  mc.setMaxAcceleration(motor, ACCEL);
  mc.setMaxDeceleration(motor, DECEL);
}

//Initialises the Motoron controller
void beginMotoron()
{
  Wire1.begin();
  Wire1.setClock(100000);

  mc.setBus(&Wire1);

  mc.reinitialize();
  delay(10);

  mc.disableCrc();
  delay(10);

  mc.clearResetFlag();
  mc.clearMotorFaultUnconditional();
  mc.setCommandTimeoutMilliseconds(2000);

  configureMotor(MOTOR_LEFT);
  configureMotor(MOTOR_RIGHT);

  motoron_ready = mc.getLastError() == 0;

  Serial.print("motoron_ready=");
  Serial.print(motoron_ready ? 1 : 0);
  Serial.print(" error=");
  Serial.println(mc.getLastError());

  stopMotors();
}

// Drives both tracks forward at base speed
void drive_forward()
{
  setMotors(
    LEFT_FORWARD_SIGN * BASE_SPEED,
    RIGHT_FORWARD_SIGN * BASE_SPEED
  );
}

// Applies proportional steering
void follow_line(int32_t error)
{
  const int16_t correction = static_cast<int16_t>(LINE_KP * static_cast<float>(error));

  const int16_t left_speed = LEFT_FORWARD_SIGN * (BASE_SPEED + correction);
  const int16_t right_speed = RIGHT_FORWARD_SIGN * (BASE_SPEED - correction);

  setMotors(left_speed, right_speed);
}

// Reverses/searches according to last line side, or recent turn direction
void recover_lost_line()
{
  const unsigned long lost_time_ms = millis() - state_start_ms;

  int8_t search_direction = last_error < 0 ? -1 : 1;

  if (last_line_side != 0) {
    search_direction = last_line_side;
  } else if (last_turn_recovery_side != 0 &&
             millis() - last_turn_end_ms < TURN_RECOVERY_MEMORY_MS) {
    search_direction = last_turn_recovery_side;
  }

  if (lost_time_ms < LOST_REVERSE_MS) {
    setMotors(
      -LEFT_FORWARD_SIGN * LOST_REVERSE_SPEED,
      -RIGHT_FORWARD_SIGN * LOST_REVERSE_SPEED
    );
    return;
  }

  if (lost_time_ms < LOST_REVERSE_MS + LOST_GENTLE_SEARCH_MS) {
    if (search_direction < 0) {
      setMotors(-LOST_GENTLE_TURN_SPEED, LOST_GENTLE_TURN_SPEED);
    } else {
      setMotors(LOST_GENTLE_TURN_SPEED, -LOST_GENTLE_TURN_SPEED);
    }
    return;
  }

  if (search_direction < 0) {
    setMotors(-LOST_HARD_TURN_SPEED, LOST_HARD_TURN_SPEED);
  } else {
    setMotors(LOST_HARD_TURN_SPEED, -LOST_HARD_TURN_SPEED);
  }
}

// Moves to centre the robot over the tag
void centreAfterRFID()
{
  int L_base = getLeftEncoder();
  int R_base = getRightEncoder();

  while ((getLeftEncoder() - L_base < 250) || (getRightEncoder() - R_base < 250)) {
    setMotors(BASE_SPEED, BASE_SPEED);
    serviceServoPulses();
    update_turn_angle();
  }

  stopMotors();
}

// Centres around current IR position
void centreAfterIR()
{
  int L_base = getLeftEncoder();
  int R_base = getRightEncoder();

  while ((getLeftEncoder() - L_base < 300) || (getRightEncoder() - R_base < 300)) {
    setMotors(BASE_SPEED, BASE_SPEED);
    serviceServoPulses();
    update_turn_angle();
  }

  stopMotors();
}

// Drives straight through a short missing-line section (hole)
void drive_forward_gap()
{
  setMotors(
    LEFT_FORWARD_SIGN * LINE_GAP_SPEED,
    RIGHT_FORWARD_SIGN * LINE_GAP_SPEED
  );
}