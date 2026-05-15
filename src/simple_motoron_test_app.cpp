#include <Arduino.h>
#include <Motoron.h>
#include <Wire.h>

namespace {

MotoronI2C mc;

constexpr uint8_t MOTOR_LEFT = 1;
constexpr uint8_t MOTOR_RIGHT = 2;
constexpr int16_t MAX_SPEED = 600;
constexpr int16_t TURN_SPEED = 600;
constexpr uint8_t RIGHT_MOTOR_TRIM_PERCENT = 95;
constexpr uint16_t ACCEL = 800;
constexpr uint16_t DECEL = 800;
constexpr unsigned long STATUS_INTERVAL_MS = 1000;
constexpr unsigned long STOP_DURATION_MS = 700;
constexpr unsigned long FORWARD_DURATION_MS = 2000;
constexpr unsigned long TURN_90_DURATION_MS = 850;
constexpr unsigned long U_TURN_DURATION_MS = TURN_90_DURATION_MS * 2;

struct DemoStep {
    const char* name;
    int16_t left_speed;
    int16_t right_speed;
    unsigned long duration_ms;
};

constexpr int16_t trim_right_speed(int16_t speed) {
    return static_cast<int16_t>(
        (static_cast<int32_t>(speed) * RIGHT_MOTOR_TRIM_PERCENT) / 100);
}

const DemoStep demo_steps[] = {
    {"forward", -MAX_SPEED, trim_right_speed(MAX_SPEED), FORWARD_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
    {"left turn", TURN_SPEED, trim_right_speed(TURN_SPEED), TURN_90_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
    {"right turn", -TURN_SPEED, trim_right_speed(-TURN_SPEED), TURN_90_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
    {"u-turn", TURN_SPEED, trim_right_speed(TURN_SPEED), U_TURN_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
};

constexpr size_t STEP_COUNT = sizeof(demo_steps) / sizeof(demo_steps[0]);

size_t step_index = 0;
unsigned long step_start_time = 0;
unsigned long last_print_time = 0;

void checkpoint(const char* message) {
    Serial.println(message);
    Serial.flush();
}

void set_motor_speeds(int16_t left_speed, int16_t right_speed) {
    mc.setSpeed(MOTOR_LEFT, left_speed);
    mc.setSpeed(MOTOR_RIGHT, right_speed);
}

void apply_step() {
    const DemoStep& step = demo_steps[step_index];
    set_motor_speeds(step.left_speed, step.right_speed);
    step_start_time = millis();

    Serial.print("Step ");
    Serial.print(step_index + 1);
    Serial.print('/');
    Serial.print(STEP_COUNT);
    Serial.print(": ");
    Serial.print(step.name);
    Serial.print(" left=");
    Serial.print(step.left_speed);
    Serial.print(" right=");
    Serial.print(step.right_speed);
    Serial.print(" duration_ms=");
    Serial.println(step.duration_ms);
    Serial.flush();
}

}  // namespace

void simple_motoron_test_app_setup() {
    Serial.begin(115200);

    const unsigned long start_time = millis();
    while (!Serial && millis() - start_time < 5000) {
    }

    delay(1000);

    checkpoint("1. Starting setup");

    checkpoint("2. Starting Wire1");
    Wire1.begin();
    checkpoint("3. Wire1.begin done");

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

    checkpoint("24. Setup complete, starting movement sequence");
    apply_step();
}

void simple_motoron_test_app_loop() {
    unsigned long now = millis();

    if (now - step_start_time >= demo_steps[step_index].duration_ms) {
        step_index = (step_index + 1) % STEP_COUNT;
        apply_step();
        now = millis();
    }

    const DemoStep& step = demo_steps[step_index];
    set_motor_speeds(step.left_speed, step.right_speed);

    if (now - last_print_time >= STATUS_INTERVAL_MS) {
        last_print_time = now;

        const unsigned long elapsed_ms = now - step_start_time;
        const unsigned long remaining_ms =
            elapsed_ms >= step.duration_ms ? 0 : step.duration_ms - elapsed_ms;

        Serial.print("Loop running. millis=");
        Serial.print(now);
        Serial.print(" step=");
        Serial.print(step.name);
        Serial.print(" remaining_ms=");
        Serial.println(remaining_ms);
        Serial.flush();
    }

    delay(100);
}
