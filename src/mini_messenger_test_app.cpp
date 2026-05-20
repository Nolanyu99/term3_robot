#include <Arduino.h>
#include <MiniMessenger.h>
#include <WiFi.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif

#ifndef BOARD_ID
#define BOARD_ID "Terminator"
#endif

#ifndef TARGET_BOARD_ID
#define TARGET_BOARD_ID "2"
#endif

namespace {

constexpr unsigned long SEND_INTERVAL_MS = 5000;
constexpr unsigned long STATUS_INTERVAL_MS = 2000;
constexpr unsigned long REGISTER_INTERVAL_MS = 10000;

MiniMessenger messenger;
unsigned long last_send_ms = 0;
unsigned long last_status_ms = 0;
unsigned long last_register_ms = 0;

void on_message(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
    Serial.print("Message from Board ");
    Serial.print(metadata.fromBoardId);
    Serial.print(": ");

    for (size_t i = 0; i < length; ++i) {
        Serial.write(payload[i]);
    }
    Serial.println();
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

void print_connection_status() {
    Serial.print("wifi=");
    Serial.print(wifi_status_name(WiFi.status()));
    Serial.print(" mqtt=");
    Serial.print(messenger.isConnected() ? "connected" : "disconnected");

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print(" ip=");
        Serial.print(WiFi.localIP());
        Serial.print(" rssi=");
        Serial.print(WiFi.RSSI());
    }

    Serial.println();
}

}  // namespace

void mini_messenger_test_app_setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== GIGA R1 MiniMessenger test ===");
    Serial.print("group=");
    Serial.print(GROUP_ID);
    Serial.print(" board=");
    Serial.println(BOARD_ID);

    messenger.onMessage(on_message);
    messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BOARD_ID);

    Serial.println("MiniMessenger setup complete");
    print_connection_status();
}

void mini_messenger_test_app_loop() {
    messenger.loop();

    const unsigned long now_ms = millis();
    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;
        print_connection_status();
    }

    if (messenger.isConnected() &&
        (last_register_ms == 0 || now_ms - last_register_ms >= REGISTER_INTERVAL_MS)) {
        last_register_ms = now_ms;
        register_with_server();
    }

    if (!messenger.isConnected() || now_ms - last_send_ms < SEND_INTERVAL_MS) {
        return;
    }

    last_send_ms = now_ms;
    messenger.sendToBoard(TARGET_BOARD_ID, "Hello from " BOARD_ID "!");
    Serial.println("MiniMessenger message sent");
}
