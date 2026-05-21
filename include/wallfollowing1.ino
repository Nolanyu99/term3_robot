#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

// =====================================================
// Motor configuration
// =====================================================
const uint8_t MOTOR_LEFT  = 1;
const uint8_t MOTOR_RIGHT = 2;
const int16_t MAX_SPEED_R = 800 * (1.4264/1.4681);
const int16_t MAX_SPEED_L = 800;

// =====================================================
// Ultrasonic sensor pins  (UPDATE THESE TO YOUR ACTUAL PINS)
// =====================================================
const uint8_t TRIG_SIDE    = 47;
const uint8_t ECHO_SIDE    = 46;
const uint8_t TRIG_FORWARD = 52;
const uint8_t ECHO_FORWARD = 53;

// =====================================================
// Wall-following parameters
// =====================================================
const float TARGET_SIDE_MM     = 50.0;    // 5 cm from left wall — slightly above sensor's reliable minimum
const float STOP_FORWARD_MM    = 50.0;   // stop when door/wall ahead is closer than 10 cm
const float DOOR_OPEN_MM       = 250.0;   // resume when forward distance exceeds 25 cm
const float Kp_WALL            = 4.0;     // tune empirically (more aggressive than full-tunnel since narrow space)
const int16_t BASE_RAMP_SPEED  = 550;     // higher than flat-ground wall-follow because of the ramp incline

const float MAX_VALID_DIST     = 2000.0;
const float MIN_VALID_DIST     = 20.0;

// =====================================================
// Mode state
// =====================================================
enum WallFollowMode {
    WF_DRIVING,         // wall-following up the ramp
    WF_WAITING_AT_DOOR, // stopped, waiting for door to open
    WF_PASSING_THROUGH, // door open, drive through
    WF_DONE             // exited tunnel
};

WallFollowMode mode = WF_DRIVING;

// Last valid readings to handle dropouts
float lastValidSide = TARGET_SIDE_MM;
float lastValidForward = 1000.0;

// =====================================================
// Ultrasonic reading
// =====================================================
float readUltrasonicMM(uint8_t trigPin, uint8_t echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return -1.0;

    return (duration * 0.343) / 2.0;
}

float readSideDistance() {
    float d = readUltrasonicMM(TRIG_SIDE, ECHO_SIDE);
    if (d > MIN_VALID_DIST && d < MAX_VALID_DIST) {
        lastValidSide = d;
        return d;
    }
    return lastValidSide;
}

float readForwardDistance() {
    float d = readUltrasonicMM(TRIG_FORWARD, ECHO_FORWARD);
    if (d > MIN_VALID_DIST && d < MAX_VALID_DIST) {
        lastValidForward = d;
        return d;
    }
    return lastValidForward;
}

// =====================================================
// Motor control
// =====================================================
void setMotors(int16_t leftSpeed, int16_t rightSpeed) {
    leftSpeed  = constrain(leftSpeed,  -MAX_SPEED_L, MAX_SPEED_L);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED_R, MAX_SPEED_R);
    mc.setSpeed(MOTOR_LEFT,  leftSpeed);
    mc.setSpeed(MOTOR_RIGHT, rightSpeed);
}

void stopMotors() {
    setMotors(0, 0);
}

// =====================================================
// Wall-following step
// =====================================================
// Side sensor on LEFT side:
//   error = d_side - target
//   error > 0 → too far from wall → steer LEFT (toward wall) → left motor slower
//   error < 0 → too close to wall → steer RIGHT (away)      → right motor slower
void wallFollowStep() {
    float d_side = readSideDistance();
    float error = d_side - TARGET_SIDE_MM;
    float correction = Kp_WALL * error;

    int16_t leftSpeed  = BASE_RAMP_SPEED - (int16_t)correction;
    int16_t rightSpeed = BASE_RAMP_SPEED + (int16_t)correction;

    setMotors(leftSpeed, rightSpeed);
}

// =====================================================
// Setup
// =====================================================
void setup() {
    Serial.begin(115200);
    unsigned long startTime = millis();
    while (!Serial && millis() - startTime < 3000) {}
    delay(500);

    Wire1.begin();
    Wire1.setClock(100000);
    mc.setBus(&Wire1);
    mc.reinitialize();
    delay(10);
    mc.disableCrc();
    mc.clearResetFlag();
    mc.clearMotorFaultUnconditional();
    mc.setCommandTimeoutMilliseconds(2000);
    mc.setMaxAcceleration(MOTOR_LEFT,  800);
    mc.setMaxDeceleration(MOTOR_LEFT,  800);
    mc.setMaxAcceleration(MOTOR_RIGHT, 800);
    mc.setMaxDeceleration(MOTOR_RIGHT, 800);

    pinMode(TRIG_SIDE,    OUTPUT);
    pinMode(ECHO_SIDE,    INPUT);
    pinMode(TRIG_FORWARD, OUTPUT);
    pinMode(ECHO_FORWARD, INPUT);
    digitalWrite(TRIG_SIDE,    LOW);
    digitalWrite(TRIG_FORWARD, LOW);

    Serial.println("Wall-follow module ready. Starting in WF_DRIVING mode.");
}

// =====================================================
// Loop
// =====================================================
unsigned long lastLogMs = 0;
unsigned long doorWaitStartMs = 0;

void loop() {
    float fwd = readForwardDistance();

    switch (mode) {
        case WF_DRIVING:
            wallFollowStep();
            if (fwd < STOP_FORWARD_MM) {
                Serial.println("Reached door — stopping.");
                stopMotors();
                mode = WF_WAITING_AT_DOOR;
                doorWaitStartMs = millis();
            }
            break;

        case WF_WAITING_AT_DOOR:
            stopMotors();
            if (fwd > DOOR_OPEN_MM) {
                Serial.println("Door opened — driving through.");
                mode = WF_PASSING_THROUGH;
            }
            break;

        case WF_PASSING_THROUGH:
            wallFollowStep();
            // Once we've driven past the door, declare done
            // (you may want a timer here or to wait for QTR line detection in final integration)
            if (fwd > 500.0) {
                Serial.println("Through the door — exiting wall-follow.");
                stopMotors();
                mode = WF_DONE;
            }
            break;

        case WF_DONE:
            stopMotors();
            break;
    }

    // Periodic logging
    if (millis() - lastLogMs >= 200) {
        lastLogMs = millis();
        Serial.print("mode=");
        switch (mode) {
            case WF_DRIVING:         Serial.print("DRIVING"); break;
            case WF_WAITING_AT_DOOR: Serial.print("WAITING"); break;
            case WF_PASSING_THROUGH: Serial.print("PASSING"); break;
            case WF_DONE:            Serial.print("DONE");    break;
        }
        Serial.print(" side=");
        Serial.print(lastValidSide, 0);
        Serial.print("mm fwd=");
        Serial.print(lastValidForward, 0);
        Serial.println("mm");
    }

    delay(20);
}
