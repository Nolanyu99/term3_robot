#include <Arduino.h>
#include <MiniMessenger.h>
#include <Motoron.h>
#include <WiFi.h>
#include <Wire.h>

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
constexpr int16_t TEST_SPEED = 300;
constexpr uint8_t RIGHT_MOTOR_TRIM_PERCENT = 95;
constexpr uint16_t MOTOR_ACCEL = 800;
constexpr uint16_t MOTOR_DECEL = 800;
constexpr unsigned long REGISTER_INTERVAL_MS = 10000;
constexpr unsigned long STATUS_INTERVAL_MS = 1000;

MiniMessenger messenger;
MotoronI2C motoron;

bool motoron_ready = false;
bool server_enabled = false;
unsigned long last_register_ms = 0;
unsigned long last_status_ms = 0;

constexpr int16_t trim_right_speed(int16_t speed) {
    return static_cast<int16_t>(
        (static_cast<int32_t>(speed) * RIGHT_MOTOR_TRIM_PERCENT) / 100);
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
    if (server_enabled) {
        set_motor_speeds(TEST_SPEED, trim_right_speed(TEST_SPEED));
    } else {
        stop_motors();
    }
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

void set_server_enabled(bool enabled, const char* reason) {
    if (server_enabled == enabled) {
        return;
    }

    server_enabled = enabled;
    apply_motor_state();

    Serial.print("server_enabled=");
    Serial.print(server_enabled ? 1 : 0);
    Serial.print(" reason=");
    Serial.println(reason);
}

void on_message(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
    char message[160];
    const size_t copy_length = min(length, sizeof(message) - 1);
    memcpy(message, payload, copy_length);
    message[copy_length] = '\0';

    Serial.print("Message from Board ");
    Serial.print(metadata.fromBoardId);
    Serial.print(": ");
    Serial.println(message);

    if (strstr(message, "type=heartbeat enable=1")) {
        set_server_enabled(true, "heartbeat_enable");
        return;
    }

    if (strstr(message, "type=heartbeat enable=0")) {
        set_server_enabled(false, "heartbeat_disable");
        return;
    }

    if (strstr(message, "type=emergency enabled=true")) {
        set_server_enabled(false, "emergency");
        return;
    }

    if (strstr(message, "type=disable enabled=false")) {
        set_server_enabled(false, "disable");
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
        Serial.print("Registered with server: ");
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
    Serial.print(" server_enabled=");
    Serial.print(server_enabled ? 1 : 0);

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print(" ip=");
        Serial.print(WiFi.localIP());
        Serial.print(" rssi=");
        Serial.print(WiFi.RSSI());
    }

    Serial.println();
}

}  // namespace

void motor_messenger_test_app_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== GIGA R1 Motor + MiniMessenger test ===");
    Serial.println("Lift the wheels before enabling from the dashboard.");
    Serial.print("group=");
    Serial.print(GROUP_ID);
    Serial.print(" board=");
    Serial.println(BOARD_ID);

    begin_motoron();

    messenger.onMessage(on_message);
    messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BOARD_ID);
    print_status();
}

void motor_messenger_test_app_loop() {
    messenger.loop();
    apply_motor_state();

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
