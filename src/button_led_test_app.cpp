#include <Arduino.h>

#include "robot_config.hpp"

namespace {

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long BLINK_INTERVAL_MS = 250;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 50;
constexpr unsigned long STATUS_INTERVAL_MS = 1000;

bool stopped = true;
bool red_on = false;
bool revive_pressed = false;
int last_kill_button_reading = HIGH;
int stable_kill_button_state = HIGH;
int last_revive_button_reading = HIGH;
int stable_revive_button_state = HIGH;
unsigned long last_kill_button_change_ms = 0;
unsigned long last_revive_button_change_ms = 0;
unsigned long last_blink_ms = 0;
unsigned long last_status_ms = 0;

uint8_t rgb_level(bool on) {
    if (robot_config::TOP_RGB_COMMON_ANODE) {
        return on ? 0 : 255;
    }
    return on ? 255 : 0;
}

void write_top_rgb(bool red, bool green, bool blue) {
    if (robot_config::TOP_RGB_RED_PIN >= 0) {
        analogWrite(robot_config::TOP_RGB_RED_PIN, rgb_level(red));
    }
    if (robot_config::TOP_RGB_GREEN_PIN >= 0) {
        analogWrite(robot_config::TOP_RGB_GREEN_PIN, rgb_level(green));
    }
    if (robot_config::TOP_RGB_BLUE_PIN >= 0) {
        analogWrite(robot_config::TOP_RGB_BLUE_PIN, rgb_level(blue));
    }
}

void render_led() {
    if (revive_pressed) {
        write_top_rgb(false, true, false);
        return;
    }

    if (!stopped) {
        red_on = true;
        write_top_rgb(true, false, false);
        return;
    }

    write_top_rgb(red_on, false, false);
}

void set_stopped(bool value) {
    stopped = value;
    red_on = stopped;
    if (stopped) {
        Serial.println("state=STOPPED red blink");
    } else {
        Serial.println("state=RUNNING red solid");
    }
    render_led();
}

void update_kill_button() {
    if (robot_config::MECHANICAL_KILL_BUTTON_PIN < 0) {
        return;
    }

    const int reading = digitalRead(robot_config::MECHANICAL_KILL_BUTTON_PIN);
    const unsigned long now_ms = millis();

    if (reading != last_kill_button_reading) {
        last_kill_button_change_ms = now_ms;
        last_kill_button_reading = reading;
    }

    if (now_ms - last_kill_button_change_ms < BUTTON_DEBOUNCE_MS) {
        return;
    }

    if (reading == stable_kill_button_state) {
        return;
    }

    stable_kill_button_state = reading;
    if (stable_kill_button_state == LOW) {
        set_stopped(!stopped);
    }
}

void update_revive_button() {
    if (robot_config::REVIVE_BUTTON_PIN < 0) {
        return;
    }

    const int reading = digitalRead(robot_config::REVIVE_BUTTON_PIN);
    const unsigned long now_ms = millis();

    if (reading != last_revive_button_reading) {
        last_revive_button_change_ms = now_ms;
        last_revive_button_reading = reading;
    }

    if (now_ms - last_revive_button_change_ms < BUTTON_DEBOUNCE_MS) {
        return;
    }

    if (reading == stable_revive_button_state) {
        return;
    }

    stable_revive_button_state = reading;
    revive_pressed = stable_revive_button_state == LOW;
    Serial.print("revive=");
    Serial.println(revive_pressed ? "pressed green" : "released");
    render_led();
}

void update_led() {
    if (revive_pressed || !stopped) {
        render_led();
        return;
    }

    const unsigned long now_ms = millis();
    if (now_ms - last_blink_ms < BLINK_INTERVAL_MS) {
        return;
    }

    last_blink_ms = now_ms;
    red_on = !red_on;
    render_led();
}

void print_status() {
    const unsigned long now_ms = millis();
    if (now_ms - last_status_ms < STATUS_INTERVAL_MS) {
        return;
    }
    last_status_ms = now_ms;

    Serial.print("kill_button=");
    const int raw_kill_button = digitalRead(robot_config::MECHANICAL_KILL_BUTTON_PIN);
    Serial.print(raw_kill_button == LOW ? "pressed" : "released");
    Serial.print(" raw=");
    Serial.print(raw_kill_button);
    Serial.print(" stable=");
    Serial.print(stable_kill_button_state);
    Serial.print(" revive_button=");
    const int raw_revive_button = digitalRead(robot_config::REVIVE_BUTTON_PIN);
    Serial.print(raw_revive_button == LOW ? "pressed" : "released");
    Serial.print(" state=");
    Serial.print(stopped ? "stopped" : "running");
    Serial.print(" led=");
    if (revive_pressed) {
        Serial.println("green");
    } else {
        Serial.println(stopped ? "blinking red" : "solid red");
    }
}

}  // namespace

void button_led_test_app_setup() {
    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== Button + RGB LED test ===");
    Serial.println("Kill button toggles stopped/running.");
    Serial.println("Stopped = blinking red. Running = solid red. Revive held = green.");
    Serial.print("RGB pins R/B/G = D");
    Serial.print(robot_config::TOP_RGB_RED_PIN);
    Serial.print("/D");
    Serial.print(robot_config::TOP_RGB_BLUE_PIN);
    Serial.print("/D");
    Serial.println(robot_config::TOP_RGB_GREEN_PIN);
    Serial.print("Kill button pin = D");
    Serial.println(robot_config::MECHANICAL_KILL_BUTTON_PIN);
    Serial.print("Revive button pin = D");
    Serial.println(robot_config::REVIVE_BUTTON_PIN);

    if (robot_config::TOP_RGB_RED_PIN >= 0) {
        pinMode(robot_config::TOP_RGB_RED_PIN, OUTPUT);
    }
    if (robot_config::TOP_RGB_GREEN_PIN >= 0) {
        pinMode(robot_config::TOP_RGB_GREEN_PIN, OUTPUT);
    }
    if (robot_config::TOP_RGB_BLUE_PIN >= 0) {
        pinMode(robot_config::TOP_RGB_BLUE_PIN, OUTPUT);
    }

    if (robot_config::MECHANICAL_KILL_BUTTON_PIN >= 0) {
        pinMode(robot_config::MECHANICAL_KILL_BUTTON_PIN, INPUT_PULLUP);
        last_kill_button_reading = digitalRead(robot_config::MECHANICAL_KILL_BUTTON_PIN);
        stable_kill_button_state = last_kill_button_reading;
    }

    if (robot_config::REVIVE_BUTTON_PIN >= 0) {
        pinMode(robot_config::REVIVE_BUTTON_PIN, INPUT_PULLUP);
        last_revive_button_reading = digitalRead(robot_config::REVIVE_BUTTON_PIN);
        stable_revive_button_state = last_revive_button_reading;
        revive_pressed = stable_revive_button_state == LOW;
    }

    last_kill_button_change_ms = millis();
    last_revive_button_change_ms = millis();
    last_blink_ms = millis();
    last_status_ms = millis();
    set_stopped(true);
}

void button_led_test_app_loop() {
    update_kill_button();
    update_revive_button();
    update_led();
    print_status();
}
