// =====================================================
// Tunnel_Entry.ino
// Base-exit gate + ramp wall-following + tunnel-exit gate
// Runs after line-following reaches the base exit gate.
// =====================================================

#include <Arduino.h>

// -----------------------------------------------------
// Mission phase (shared with Main.ino and LineSensors.ino)
// -----------------------------------------------------
enum class MissionPhase {
  BaseToGate,   // line following toward the base exit gate
  Tunnel,       // gate + ramp wall-follow + exit gate sequence
  Arena         // out in the arena, normal mission
};

MissionPhase mission_phase = MissionPhase::BaseToGate;

// -----------------------------------------------------
// Ultrasonic pins
// -----------------------------------------------------
constexpr uint8_t TUNNEL_TRIG_SIDE    = 47;
constexpr uint8_t TUNNEL_ECHO_SIDE    = 46;
constexpr uint8_t TUNNEL_TRIG_FORWARD = 52;
constexpr uint8_t TUNNEL_ECHO_FORWARD = 53;

// -----------------------------------------------------
// Distance thresholds (mm)
// -----------------------------------------------------
constexpr float GATE_STOP_MM           = 100.0f;  // base exit gate detected ahead
constexpr float BASE_DOOR_OPEN_MM      = 250.0f;  // base gate has opened
constexpr float TARGET_SIDE_MM         = 60.0f;   // wall-follow target distance
constexpr float SIDE_WALL_DETECTED_MM  = 120.0f;  // side wall is "found"
constexpr float TUNNEL_DOOR_CLOSED_MM  = 50.0f;   // outer door detected ahead
constexpr float TUNNEL_DOOR_OPEN_MM    = 250.0f;  // outer door has opened
constexpr float TUNNEL_DONE_MM         = 500.0f;  // fully through outer door
constexpr float US_MAX_VALID_MM        = 2000.0f;
constexpr float US_MIN_VALID_MM        = 20.0f;

// -----------------------------------------------------
// Wall-follow control
// -----------------------------------------------------
constexpr float    WALL_KP          = 4.0f;
constexpr int16_t  BASE_RAMP_SPEED  = 400;
constexpr int16_t  WALL_MAX_SPEED   = 600;

// -----------------------------------------------------
// Distance gate before checking for the side wall
// (so the robot drives fully into the tunnel first)
// -----------------------------------------------------
constexpr long BASE_EXIT_MIN_DISTANCE_COUNTS = 2000;
constexpr unsigned long BASE_EXIT_DRIVE_TIMEOUT_MS = 3000;

// -----------------------------------------------------
// Internal tunnel sub-states
// -----------------------------------------------------
enum class TunnelState {
  WaitAtBaseGate,   // stopped at base gate, waiting for it to open
  DrivingIntoTunnel,// driving forward into the tunnel
  WallFollow,       // following the wall up the ramp
  WaitAtExitGate,   // stopped at outer door, waiting
  PassingExitGate   // driving through the outer door
};

TunnelState tunnel_state = TunnelState::WaitAtBaseGate;

unsigned long tunnel_state_start_ms = 0;
long base_exit_start_counts = 0;

float last_valid_side_mm = TARGET_SIDE_MM;
float last_valid_forward_mm = 1000.0f;

// -----------------------------------------------------
// Ultrasonic reading
// -----------------------------------------------------
float read_ultrasonic_mm(uint8_t trig_pin, uint8_t echo_pin)
{
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  unsigned long duration = pulseIn(echo_pin, HIGH, 30000);
  if (duration == 0) {
    return -1.0f;
  }

  return (duration * 0.343f) / 2.0f;
}

float side_distance_mm()
{
  float d = read_ultrasonic_mm(TUNNEL_TRIG_SIDE, TUNNEL_ECHO_SIDE);
  if (d > US_MIN_VALID_MM && d < US_MAX_VALID_MM) {
    last_valid_side_mm = d;
    return d;
  }
  return last_valid_side_mm;
}

float forward_distance_mm()
{
  float d = read_ultrasonic_mm(TUNNEL_TRIG_FORWARD, TUNNEL_ECHO_FORWARD);
  if (d > US_MIN_VALID_MM && d < US_MAX_VALID_MM) {
    last_valid_forward_mm = d;
    return d;
  }
  return last_valid_forward_mm;
}

// -----------------------------------------------------
// Wall-follow step (side sensor on LEFT wall)
//   error > 0 -> too far from wall -> steer toward it
//   error < 0 -> too close -> steer away
// -----------------------------------------------------
void wall_follow_step()
{
  float d_side = side_distance_mm();
  float error = d_side - TARGET_SIDE_MM;
  float correction = WALL_KP * error;

  correction = constrain(correction, -300.0f, 300.0f);

  int16_t left_speed  = BASE_RAMP_SPEED - (int16_t)correction;
  int16_t right_speed = BASE_RAMP_SPEED + (int16_t)correction;

  left_speed  = constrain(left_speed,  -WALL_MAX_SPEED, WALL_MAX_SPEED);
  right_speed = constrain(right_speed, -WALL_MAX_SPEED, WALL_MAX_SPEED);

  setMotors(left_speed, right_speed);
}

void drive_into_tunnel()
{
  setMotors(BASE_RAMP_SPEED / 2, BASE_RAMP_SPEED / 2);
}

// -----------------------------------------------------
// Tunnel setup — call once from Main.ino setup()
// -----------------------------------------------------
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

// -----------------------------------------------------
// State transition helper
// -----------------------------------------------------
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

// -----------------------------------------------------
// Main tunnel step — call each control cycle while
// mission_phase == MissionPhase::Tunnel
// -----------------------------------------------------
void run_tunnel_step()
{
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
          break;  // keep driving in before looking for the wall
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
