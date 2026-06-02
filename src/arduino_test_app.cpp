#include <Arduino.h>

namespace {

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 1000;
constexpr unsigned long FAST_BLINK_INTERVAL_MS = 200;
constexpr unsigned long SLOW_BLINK_INTERVAL_MS = 1000;

unsigned long blink_interval_ms = FAST_BLINK_INTERVAL_MS;
unsigned long last_blink_ms = 0;
unsigned long last_heartbeat_ms = 0;
bool led_enabled = true;
bool led_on = false;

void print_help() {
    Serial.println(F("Commands:"));
    Serial.println(F("  f  fast blink"));
    Serial.println(F("  s  slow blink"));
    Serial.println(F("  0  LED off"));
    Serial.println(F("  1  LED on"));
    Serial.println(F("  a  read A0"));
    Serial.println(F("  h  help"));
}

void set_led(bool on) {
    led_on = on;
    digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
}

void handle_command(char command) {
    if (command == '\r' || command == '\n') {
        return;
    }

    Serial.print(F("rx="));
    Serial.println(command);

    switch (command) {
        case 'f':
            led_enabled = true;
            blink_interval_ms = FAST_BLINK_INTERVAL_MS;
            Serial.println(F("Blink mode: fast"));
            break;
        case 's':
            led_enabled = true;
            blink_interval_ms = SLOW_BLINK_INTERVAL_MS;
            Serial.println(F("Blink mode: slow"));
            break;
        case '0':
            led_enabled = false;
            set_led(false);
            Serial.println(F("LED forced off"));
            break;
        case '1':
            led_enabled = false;
            set_led(true);
            Serial.println(F("LED forced on"));
            break;
        case 'a':
            Serial.print(F("A0="));
            Serial.println(analogRead(A0));
            break;
        case 'h':
        case '?':
            print_help();
            break;
        default:
            Serial.println(F("Echo OK. Send h for commands."));
            break;
    }
}

void update_led() {
    if (!led_enabled) {
        return;
    }

    const unsigned long now_ms = millis();
    if (now_ms - last_blink_ms < blink_interval_ms) {
        return;
    }

    last_blink_ms = now_ms;
    set_led(!led_on);
}

void print_heartbeat() {
    const unsigned long now_ms = millis();
    if (now_ms - last_heartbeat_ms < HEARTBEAT_INTERVAL_MS) {
        return;
    }

    last_heartbeat_ms = now_ms;
    Serial.print(F("arduino_test alive ms="));
    Serial.print(now_ms);
    Serial.print(F(" led="));
    Serial.println(led_on ? F("on") : F("off"));
}

}  // namespace

void arduino_test_app_setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    set_led(false);

    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    Serial.println();
    Serial.println(F("=== Arduino smoke test ==="));
    Serial.println(F("Serial, millis(), analogRead(), and LED_BUILTIN are available."));
    print_help();

    last_blink_ms = millis();
    last_heartbeat_ms = millis();
}

void arduino_test_app_loop() {
    update_led();
    print_heartbeat();

    while (Serial.available() > 0) {
        handle_command(static_cast<char>(Serial.read()));
    }
}
