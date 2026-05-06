#include <Arduino.h>

namespace {

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long BLINK_INTERVAL_MS = 500;
constexpr unsigned long PRINT_INTERVAL_MS = 1000;

unsigned long last_blink_ms = 0;
unsigned long last_print_ms = 0;
bool led_on = false;

}  // namespace

void upload_test_app_setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== Upload test ===");
    Serial.println("Firmware is running. No motors or sensors are used.");
}

void upload_test_app_loop() {
    const unsigned long now_ms = millis();

    if (now_ms - last_blink_ms >= BLINK_INTERVAL_MS) {
        last_blink_ms = now_ms;
        led_on = !led_on;
        digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
    }

    if (now_ms - last_print_ms >= PRINT_INTERVAL_MS) {
        last_print_ms = now_ms;
        Serial.print("upload_test alive ms=");
        Serial.println(now_ms);
    }
}
