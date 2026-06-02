#include <Arduino.h>
#include <Motoron.h>
#include <Wire.h>

namespace {

MotoronI2C mc;

constexpr uint8_t MOTOR_LEFT = 1;
constexpr uint8_t MOTOR_RIGHT = 2;
constexpr uint8_t ENC_LEFT_A = 28;
constexpr uint8_t ENC_LEFT_B = 26;
constexpr uint8_t ENC_RIGHT_A = 22;
constexpr uint8_t ENC_RIGHT_B = 24;
constexpr int16_t TEST_SPEED = 300;
constexpr uint16_t ACCEL = 800;
constexpr uint16_t DECEL = 800;
constexpr unsigned long MOVE_DURATION_MS = 2000;
constexpr unsigned long STOP_DURATION_MS = 700;
constexpr unsigned long STATUS_INTERVAL_MS = 1000;

struct TestStep {
    const char* name;
    int16_t left_speed;
    int16_t right_speed;
    unsigned long duration_ms;
};

const TestStep test_steps[] = {
    {"left forward", TEST_SPEED, 0, MOVE_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
    {"left reverse", -TEST_SPEED, 0, MOVE_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
    {"right forward", 0, TEST_SPEED, MOVE_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
    {"right reverse", 0, -TEST_SPEED, MOVE_DURATION_MS},
    {"stop", 0, 0, STOP_DURATION_MS},
};

constexpr size_t STEP_COUNT = sizeof(test_steps) / sizeof(test_steps[0]);

size_t step_index = 0;
unsigned long step_start_ms = 0;
unsigned long last_status_ms = 0;
volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;
long step_start_left_count = 0;
long step_start_right_count = 0;

void read_left_encoder() {
    if (digitalRead(ENC_LEFT_A) == digitalRead(ENC_LEFT_B)) {
        encoder_left_count--;
    } else {
        encoder_left_count++;
    }
}

void read_right_encoder() {
    if (digitalRead(ENC_RIGHT_A) == digitalRead(ENC_RIGHT_B)) {
        encoder_right_count++;
    } else {
        encoder_right_count--;
    }
}

long get_left_encoder() {
    noInterrupts();
    const long value = encoder_left_count;
    interrupts();
    return value;
}

long get_right_encoder() {
    noInterrupts();
    const long value = encoder_right_count;
    interrupts();
    return value;
}

void set_motor_speeds(int16_t left_speed, int16_t right_speed) {
    mc.setSpeed(MOTOR_LEFT, left_speed);
    mc.setSpeed(MOTOR_RIGHT, right_speed);
}

void apply_step() {
    const TestStep& step = test_steps[step_index];
    set_motor_speeds(step.left_speed, step.right_speed);
    step_start_ms = millis();
    step_start_left_count = get_left_encoder();
    step_start_right_count = get_right_encoder();

    Serial.print("step=");
    Serial.print(step_index + 1);
    Serial.print('/');
    Serial.print(STEP_COUNT);
    Serial.print(" name=");
    Serial.print(step.name);
    Serial.print(" left=");
    Serial.print(step.left_speed);
    Serial.print(" right=");
    Serial.print(step.right_speed);
    Serial.print(" duration_ms=");
    Serial.println(step.duration_ms);
    Serial.flush();
}

void print_encoder_status(const TestStep& step, unsigned long elapsed_ms) {
    const long left_count = get_left_encoder();
    const long right_count = get_right_encoder();

    Serial.print("running name=");
    Serial.print(step.name);
    Serial.print(" elapsed_ms=");
    Serial.print(elapsed_ms);
    Serial.print(" encoder_total=");
    Serial.print(left_count);
    Serial.print(',');
    Serial.print(right_count);
    Serial.print(" encoder_step_delta=");
    Serial.print(left_count - step_start_left_count);
    Serial.print(',');
    Serial.println(right_count - step_start_right_count);
    Serial.flush();
}

void configure_motor(uint8_t motor) {
    mc.setMaxAcceleration(motor, ACCEL);
    mc.setMaxDeceleration(motor, DECEL);
}

}  // namespace

void individual_wheel_test_app_setup() {
    Serial.begin(115200);

    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < 5000) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== Individual wheel back-and-forth test ===");
    Serial.println("Lift the wheels before running this test.");
    Serial.println("Sequence: left forward/reverse, then right forward/reverse.");

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
    configure_motor(MOTOR_LEFT);
    configure_motor(MOTOR_RIGHT);

    pinMode(ENC_LEFT_A, INPUT_PULLUP);
    pinMode(ENC_LEFT_B, INPUT_PULLUP);
    pinMode(ENC_RIGHT_A, INPUT_PULLUP);
    pinMode(ENC_RIGHT_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), read_left_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), read_right_encoder, CHANGE);

    Serial.println("Encoders enabled: left A/B=28/26, right A/B=22/24.");

    apply_step();
}

void individual_wheel_test_app_loop() {
    unsigned long now_ms = millis();

    if (now_ms - step_start_ms >= test_steps[step_index].duration_ms) {
        step_index = (step_index + 1) % STEP_COUNT;
        apply_step();
        now_ms = millis();
    }

    const TestStep& step = test_steps[step_index];
    set_motor_speeds(step.left_speed, step.right_speed);

    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;
        print_encoder_status(step, now_ms - step_start_ms);
    }

    delay(100);
}
