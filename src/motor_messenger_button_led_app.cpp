#include <Arduino.h>
#include <MiniMessenger.h>
#include <Motoron.h>
#include <WiFi.h>
#include <Wire.h>

#include "robot_config.hpp"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif

#ifndef BOARD_ID
#define BOARD_ID "1"
#endif

namespace {

constexpr uint8_t MOTOR_LEFT = 1;
constexpr uint8_t MOTOR_RIGHT = 2;
constexpr uint8_t MOTOR_AUX = 3;
constexpr int16_t RUN_SPEED = 300;
constexpr uint8_t RIGHT_MOTOR_TRIM_PERCENT = 95;
constexpr uint16_t MOTOR_ACCEL = 800;
constexpr uint16_t MOTOR_DECEL = 800;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 50;
constexpr unsigned long REGISTER_INTERVAL_MS = 10000;
constexpr unsigned long STATUS_INTERVAL_MS = 1000;
constexpr unsigned long LED_BLINK_INTERVAL_MS = 250;

MiniMessenger messenger;
MotoronI2C motoron;

bool motoron_ready = false;
bool stopped_by_button = true;
bool stopped_by_server = false;
bool top_red_on = false;
int last_kill_button_reading = HIGH;
int stable_kill_button_state = HIGH;
unsigned long last_kill_button_change_ms = 0;
unsigned long last_register_ms = 0;
unsigned long last_status_ms = 0;
unsigned long last_led_blink_ms = 0;

constexpr int16_t trim_right_speed(int16_t speed) {
    return static_cast<int16_t>(
        (static_cast<int32_t>(speed) * RIGHT_MOTOR_TRIM_PERCENT) / 100);
}

bool robot_stopped() {
    return stopped_by_button || stopped_by_server || !motoron_ready;
}

uint8_t rgb_level(bool on) {
    if (robot_config::TOP_RGB_COMMON_ANODE) {
        return on ? 0 : 255;
    }
    return on ? 255 : 0;
}

void write_top_rgb(bool red, bool green, bool blue) {
    if (!robot_config::ENABLE_TOP_RGB) {
        return;
    }

    if (robot_config::TOP_RGB_RED_PIN >= 0) {
        digitalWrite(robot_config::TOP_RGB_RED_PIN, rgb_level(red));
    }
    if (robot_config::TOP_RGB_GREEN_PIN >= 0) {
        digitalWrite(robot_config::TOP_RGB_GREEN_PIN, rgb_level(green));
    }
    if (robot_config::TOP_RGB_BLUE_PIN >= 0) {
        digitalWrite(robot_config::TOP_RGB_BLUE_PIN, rgb_level(blue));
    }
}

void render_top_led() {
    if (!robot_config::ENABLE_TOP_RGB) {
        return;
    }

    if (robot_stopped()) {
        write_top_rgb(top_red_on, false, false);
        return;
    }

    top_red_on = true;
    write_top_rgb(true, false, false);
}

void update_top_led() {
    if (!robot_config::ENABLE_TOP_RGB) {
        return;
    }

    if (!robot_stopped()) {
        render_top_led();
        return;
    }

    const unsigned long now_ms = millis();
    if (now_ms - last_led_blink_ms < LED_BLINK_INTERVAL_MS) {
        return;
    }

    last_led_blink_ms = now_ms;
    top_red_on = !top_red_on;
    render_top_led();
}

void begin_top_led() {
    if (!robot_config::ENABLE_TOP_RGB) {
        return;
    }

    if (robot_config::TOP_RGB_RED_PIN >= 0) {
        pinMode(robot_config::TOP_RGB_RED_PIN, OUTPUT);
    }
    if (robot_config::TOP_RGB_GREEN_PIN >= 0) {
        pinMode(robot_config::TOP_RGB_GREEN_PIN, OUTPUT);
    }
    if (robot_config::TOP_RGB_BLUE_PIN >= 0) {
        pinMode(robot_config::TOP_RGB_BLUE_PIN, OUTPUT);
    }

    top_red_on = true;
    last_led_blink_ms = millis();
    render_top_led();
}

const char* wifi_status_name(int status) {
    switch (status) {
        case WL_IDLE_STATUS:
            return "idle";
        case WL_NO_SSID_AVAIL:
            return "no_ssid";
        case WL_SCAN_COMPLETED:
            return "scan_done";
        case WL_CONNECTED:
            return "connected";
        case WL_CONNECT_FAILED:
            return "connect_failed";
        case WL_CONNECTION_LOST:
            return "connection_lost";
        case WL_DISCONNECTED:
            return "disconnected";
        case WL_NO_MODULE:
            return "no_module";
        default:
            return "unknown";
    }
}

void stop_motors() {
    if (!motoron_ready) {
        return;
    }

    motoron.setSpeed(MOTOR_LEFT, 0);
    motoron.setSpeed(MOTOR_RIGHT, 0);
    motoron.setSpeed(MOTOR_AUX, 0);
}

void set_motor_speeds(int16_t left_speed, int16_t right_speed) {
    if (!motoron_ready) {
        return;
    }

    motoron.setSpeed(MOTOR_LEFT, left_speed);
    motoron.setSpeed(MOTOR_RIGHT, right_speed);
    motoron.setSpeed(MOTOR_AUX, 0);

    if (motoron.getLastError() != 0) {
        motoron_ready = false;
        Serial.print("motoron_error=");
        Serial.println(motoron.getLastError());
    }
}

void apply_motor_state() {
    if (robot_stopped()) {
        stop_motors();
    } else {
        set_motor_speeds(RUN_SPEED, trim_right_speed(RUN_SPEED));
    }
    render_top_led();
}

void configure_motor(uint8_t motor) {
    motoron.setMaxAcceleration(motor, MOTOR_ACCEL);
    motoron.setMaxDeceleration(motor, MOTOR_DECEL);
}

void begin_motoron() {
    Serial.println("Motoron: starting Wire1");
    Wire1.begin();
    Wire1.setClock(100000);

    motoron.setBus(&Wire1);
    motoron.reinitialize();
    delay(10);
    motoron.disableCrc();
    delay(10);
    motoron.clearResetFlag();
    motoron.clearMotorFaultUnconditional();
    motoron.setCommandTimeoutMilliseconds(2000);
    configure_motor(MOTOR_LEFT);
    configure_motor(MOTOR_RIGHT);
    configure_motor(MOTOR_AUX);

    motoron_ready = motoron.getLastError() == 0;
    Serial.print("motoron_ready=");
    Serial.print(motoron_ready ? 1 : 0);
    Serial.print(" error=");
    Serial.println(motoron.getLastError());
    stop_motors();
}

void print_stop_state(const char* reason) {
    Serial.print("state=");
    Serial.print(robot_stopped() ? "STOPPED" : "RUNNING");
    Serial.print(" button_stop=");
    Serial.print(stopped_by_button ? 1 : 0);
    Serial.print(" mqtt_stop=");
    Serial.print(stopped_by_server ? 1 : 0);
    Serial.print(" reason=");
    Serial.println(reason);
}

void set_button_stopped(bool stopped, const char* reason) {
    if (stopped_by_button == stopped) {
        return;
    }

    stopped_by_button = stopped;
    apply_motor_state();
    print_stop_state(reason);
}

void set_server_stopped(bool stopped, const char* reason) {
    if (stopped_by_server == stopped) {
        return;
    }

    stopped_by_server = stopped;
    apply_motor_state();
    print_stop_state(reason);
}

void update_kill_button() {
    if (!robot_config::ENABLE_MECHANICAL_KILL_BUTTON ||
        robot_config::MECHANICAL_KILL_BUTTON_PIN < 0) {
        return;
    }

    const int reading = digitalRead(robot_config::MECHANICAL_KILL_BUTTON_PIN);
    const unsigned long now_ms = millis();

    if (reading != last_kill_button_reading) {
        last_kill_button_change_ms = now_ms;
        last_kill_button_reading = reading;
    }

    if (now_ms - last_kill_button_change_ms < BUTTON_DEBOUNCE_MS ||
        reading == stable_kill_button_state) {
        return;
    }

    stable_kill_button_state = reading;
    if (stable_kill_button_state == LOW) {
        set_button_stopped(!stopped_by_button, "kill_button_press");
    }
}

void begin_kill_button() {
    if (!robot_config::ENABLE_MECHANICAL_KILL_BUTTON ||
        robot_config::MECHANICAL_KILL_BUTTON_PIN < 0) {
        return;
    }

    pinMode(robot_config::MECHANICAL_KILL_BUTTON_PIN, INPUT_PULLUP);
    last_kill_button_reading = digitalRead(robot_config::MECHANICAL_KILL_BUTTON_PIN);
    stable_kill_button_state = last_kill_button_reading;
    last_kill_button_change_ms = millis();
}

bool message_contains_stop_word(const char* message) {
    return strstr(message, "Stop") != nullptr ||
           strstr(message, "stop") != nullptr ||
           strstr(message, "STOP") != nullptr;
}

void on_message(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
    char message[160];
    const size_t copy_length = min(length, sizeof(message) - 1);
    memcpy(message, payload, copy_length);
    message[copy_length] = '\0';

    Serial.print("mqtt from=");
    Serial.print(metadata.fromBoardId);
    Serial.print(" msg=\"");
    Serial.print(message);
    Serial.println('"');

    if (strstr(message, "type=heartbeat enable=1")) {
        set_server_stopped(false, "heartbeat_enable");
        return;
    }

    if (strstr(message, "type=heartbeat enable=0")) {
        set_server_stopped(true, "heartbeat_disable");
        return;
    }

    if (strstr(message, "type=emergency enabled=true")) {
        set_server_stopped(true, "emergency");
        return;
    }

    if (strstr(message, "type=disable enabled=false")) {
        set_server_stopped(true, "disable");
        return;
    }

    if (message_contains_stop_word(message)) {
        set_server_stopped(true, "mqtt_stop");
    }
}

void register_with_server() {
    char registration[80];
    snprintf(
        registration,
        sizeof(registration),
        "type=register team_id=%s board_id=%s",
        GROUP_ID,
        BOARD_ID);

    if (messenger.sendToBoard("server", registration)) {
        Serial.print("mqtt_registered=1 ");
        Serial.println(registration);
    }
}

void print_status() {
    Serial.print("wifi=");
    Serial.print(wifi_status_name(WiFi.status()));
    Serial.print(" mqtt=");
    Serial.print(messenger.isConnected() ? "connected" : "disconnected");
    Serial.print(" motoron=");
    Serial.print(motoron_ready ? 1 : 0);
    Serial.print(" state=");
    Serial.print(robot_stopped() ? "stopped" : "running");
    Serial.print(" button_stop=");
    Serial.print(stopped_by_button ? 1 : 0);
    Serial.print(" mqtt_stop=");
    Serial.print(stopped_by_server ? 1 : 0);
    Serial.print(" kill_button=");
    if (robot_config::ENABLE_MECHANICAL_KILL_BUTTON &&
        robot_config::MECHANICAL_KILL_BUTTON_PIN >= 0) {
        Serial.print(digitalRead(robot_config::MECHANICAL_KILL_BUTTON_PIN) == LOW ?
            "pressed" : "released");
    } else {
        Serial.print("disabled");
    }
    Serial.print(" led=");
    Serial.print(robot_stopped() ? "blinking_red" : "solid_red");

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print(" ip=");
        Serial.print(WiFi.localIP());
        Serial.print(" rssi=");
        Serial.print(WiFi.RSSI());
    }

    Serial.println();
}

}  // namespace

void motor_messenger_button_led_app_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== GIGA R1 Motor + MiniMessenger + Button/LED test ===");
    Serial.println("Kill button toggles stopped/running. MQTT stop keeps the robot stopped.");
    Serial.println("Stopped = blinking red. Running = solid red.");
    Serial.print("group=");
    Serial.print(GROUP_ID);
    Serial.print(" board=");
    Serial.println(BOARD_ID);
    Serial.print("kill_button_pin=D");
    Serial.println(robot_config::MECHANICAL_KILL_BUTTON_PIN);
    Serial.print("rgb_pins R/G/B=D");
    Serial.print(robot_config::TOP_RGB_RED_PIN);
    Serial.print("/D");
    Serial.print(robot_config::TOP_RGB_GREEN_PIN);
    Serial.print("/D");
    Serial.println(robot_config::TOP_RGB_BLUE_PIN);

    begin_top_led();
    begin_kill_button();
    begin_motoron();
    apply_motor_state();
    print_stop_state("startup");

    messenger.onMessage(on_message);
    messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BOARD_ID);
    print_status();
}

void motor_messenger_button_led_app_loop() {
    messenger.loop();
    update_kill_button();
    apply_motor_state();
    update_top_led();

    const unsigned long now_ms = millis();
    if (messenger.isConnected() &&
        (last_register_ms == 0 || now_ms - last_register_ms >= REGISTER_INTERVAL_MS)) {
        last_register_ms = now_ms;
        register_with_server();
    }

    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;
        print_status();
    }
}
