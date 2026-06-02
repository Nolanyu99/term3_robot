// =====================================================================
// Ultrasonic door detection and side-wall following through the tunnel
// Used in Main
// =====================================================================

TunnelState tunnel_state = TunnelState::WaitAtBaseGate;

unsigned long tunnel_state_start_ms = 0;
long base_exit_start_counts = 0;

float last_valid_side_mm = TARGET_SIDE_MM;
float last_valid_forward_mm = 1000.0f;

// Returns distance in millimetres or -1 if invalid
float read_ultrasonic_mm(uint8_t trig_pin, uint8_t echo_pin)
{
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  const unsigned long duration = pulseIn(echo_pin, HIGH, 30000);

  if (duration == 0) {
    return -1.0f;
  }

  return (duration * 0.343f) / 2.0f;
}

// Reads side ultrasonic with last-valid fallback
float side_distance_mm()
{
  const float d = read_ultrasonic_mm(TUNNEL_TRIG_SIDE, TUNNEL_ECHO_SIDE);

  if (d > US_MIN_VALID_MM && d < US_MAX_VALID_MM) {
    last_valid_side_mm = d;
    return d;
  }

  return last_valid_side_mm;
}

// Reads forward ultrasonic with last-valid fallback
float forward_distance_mm()
{
  const float d = read_ultrasonic_mm(TUNNEL_TRIG_FORWARD, TUNNEL_ECHO_FORWARD);

  if (d > US_MIN_VALID_MM && d < US_MAX_VALID_MM) {
    last_valid_forward_mm = d;
    return d;
  }

  return last_valid_forward_mm;
}

// Holds a target side distance from the wall
void wall_follow_step()
{
  const float d_side = side_distance_mm();
  const float error = d_side - TARGET_SIDE_MM;
  float correction = WALL_KP * error;

  correction = constrain(correction, -300.0f, 300.0f);

  int16_t left_speed  = BASE_RAMP_SPEED - (int16_t)correction;
  int16_t right_speed = BASE_RAMP_SPEED + (int16_t)correction;

  left_speed  = constrain(left_speed,  -WALL_MAX_SPEED, WALL_MAX_SPEED);
  right_speed = constrain(right_speed, -WALL_MAX_SPEED, WALL_MAX_SPEED);

  setMotors(left_speed, right_speed);
}

// Drives straight through the base doorway
void drive_into_tunnel()
{
  setMotors(BASE_RAMP_SPEED / 2, BASE_RAMP_SPEED / 2);
}

// Configures ultrasonic pins
void setup_tunnel()
{
  pinMode(TUNNEL_TRIG_SIDE,    OUTPUT);
  pinMode(TUNNEL_ECHO_SIDE,    INPUT);
  pinMode(TUNNEL_TRIG_FORWARD, OUTPUT);
  pinMode(TUNNEL_ECHO_FORWARD, INPUT);

  digitalWrite(TUNNEL_TRIG_SIDE,    LOW);
  digitalWrite(TUNNEL_TRIG_FORWARD, LOW);

  tunnel_state = TunnelState::WaitAtBaseGate;
  tunnel_state_start_ms = millis();
}

// Changes tunnel state and records entry time
void set_tunnel_state(TunnelState next)
{
  if (tunnel_state == next) {
    return;
  }

  tunnel_state = next;
  tunnel_state_start_ms = millis();

  if (next == TunnelState::DrivingIntoTunnel) {
    base_exit_start_counts = getLeftEncoder();
  }
}

// Starts the tunnel-entry mission phase
void startTunnelEntry()
{
  if (startup_cal_state != StartupCalState::Ready) {
    Serial.println("tunnel_error=not_calibrated");
    return;
  }

  if (!running) {
    Serial.println("tunnel_error=not_running");
    return;
  }

  run_enabled = false;
  mission_phase = MissionPhase::Tunnel;
  set_tunnel_state(TunnelState::WaitAtBaseGate);
  last_control_ms = millis();

  Serial.println("tunnel=start");
}

// Advances the tunnel state machine
void run_tunnel_step()
{
  if (checkStopInputsDuringTest()) {
    mission_phase = MissionPhase::BaseToGate;
    Serial.println("tunnel=aborted");
    return;
  }

  const float fwd = forward_distance_mm();
  const float side = side_distance_mm();

  switch (tunnel_state) {
    case TunnelState::WaitAtBaseGate:
      stopMotors();

      if (fwd > BASE_DOOR_OPEN_MM) {
        Serial.println("Base gate opened.");
        set_tunnel_state(TunnelState::DrivingIntoTunnel);
      }
      break;

    case TunnelState::DrivingIntoTunnel:
      drive_into_tunnel();
      {
        const long traveled = getLeftEncoder() - base_exit_start_counts;

        if (traveled < BASE_EXIT_MIN_DISTANCE_COUNTS) {
          break;
        }
      }

      if (side < SIDE_WALL_DETECTED_MM) {
        Serial.println("Side wall found - wall following.");
        set_tunnel_state(TunnelState::WallFollow);
      } else if (millis() - tunnel_state_start_ms > BASE_EXIT_DRIVE_TIMEOUT_MS) {
        Serial.println("No wall after timeout - wall following anyway.");
        set_tunnel_state(TunnelState::WallFollow);
      }
      break;

    case TunnelState::WallFollow:
      wall_follow_step();

      if (fwd < TUNNEL_DOOR_CLOSED_MM) {
        Serial.println("Reached exit gate.");
        stopMotors();
        set_tunnel_state(TunnelState::WaitAtExitGate);
      }
      break;

    case TunnelState::WaitAtExitGate:
      stopMotors();

      if (fwd > TUNNEL_DOOR_OPEN_MM) {
        Serial.println("Exit gate opened.");
        set_tunnel_state(TunnelState::PassingExitGate);
      }
      break;

    case TunnelState::PassingExitGate:
      wall_follow_step();

      if (fwd > TUNNEL_DONE_MM) {
        Serial.println("Through exit gate - entering arena.");
        stopMotors();
        mission_phase = MissionPhase::Arena;
      }
      break;
  }
}