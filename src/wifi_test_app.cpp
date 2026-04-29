#include <Arduino.h>
#include <WiFi.h>

#include "robot_config.hpp"

namespace {

constexpr unsigned long CONNECT_TIMEOUT_MS = 20000;
constexpr unsigned long STATUS_INTERVAL_MS = 5000;

bool credentials_are_placeholders() {
    return String(robot_config::WIFI_SSID) == "YOUR_WIFI_SSID" ||
           String(robot_config::WIFI_PASSWORD) == "YOUR_WIFI_PASSWORD";
}

void print_status_name(int status) {
    switch (status) {
        case WL_IDLE_STATUS:
            Serial.print("WL_IDLE_STATUS");
            break;
        case WL_NO_SSID_AVAIL:
            Serial.print("WL_NO_SSID_AVAIL");
            break;
        case WL_SCAN_COMPLETED:
            Serial.print("WL_SCAN_COMPLETED");
            break;
        case WL_CONNECTED:
            Serial.print("WL_CONNECTED");
            break;
        case WL_CONNECT_FAILED:
            Serial.print("WL_CONNECT_FAILED");
            break;
        case WL_CONNECTION_LOST:
            Serial.print("WL_CONNECTION_LOST");
            break;
        case WL_DISCONNECTED:
            Serial.print("WL_DISCONNECTED");
            break;
        case WL_NO_MODULE:
            Serial.print("WL_NO_MODULE");
            break;
        default:
            Serial.print("UNKNOWN(");
            Serial.print(status);
            Serial.print(')');
            break;
    }
}

void scan_networks() {
    Serial.println();
    Serial.println("Scanning WiFi networks...");

    const int network_count = WiFi.scanNetworks();
    if (network_count < 0) {
        Serial.print("Scan failed, result=");
        Serial.println(network_count);
        return;
    }

    Serial.print("Found networks: ");
    Serial.println(network_count);

    for (int i = 0; i < network_count; ++i) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(WiFi.SSID(i));
        Serial.print(" RSSI=");
        Serial.print(WiFi.RSSI(i));
        Serial.print(" dBm encryption=");
        Serial.println(WiFi.encryptionType(i));
    }
}

void connect_wifi() {
    if (credentials_are_placeholders()) {
        Serial.println();
        Serial.println("WiFi credentials are still placeholders.");
        Serial.println("Edit include/robot_config.hpp before testing connection.");
        return;
    }

    Serial.println();
    Serial.print("Connecting to SSID: ");
    Serial.println(robot_config::WIFI_SSID);

    WiFi.begin(robot_config::WIFI_SSID, robot_config::WIFI_PASSWORD);

    const unsigned long start_ms = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start_ms < CONNECT_TIMEOUT_MS) {
        delay(500);
        Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Connection failed, status=");
        print_status_name(WiFi.status());
        Serial.println();
        return;
    }

    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
}

}  // namespace

void wifi_test_app_setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== GIGA R1 WiFi test ===");

    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("WiFi module not found");
        while (true) {
            delay(1000);
        }
    }

    Serial.print("Initial WiFi status: ");
    print_status_name(WiFi.status());
    Serial.println();

    scan_networks();
    connect_wifi();
}

void wifi_test_app_loop() {
    static unsigned long last_status_ms = 0;
    const unsigned long now_ms = millis();

    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;

        Serial.print("Current WiFi status: ");
        print_status_name(WiFi.status());

        if (WiFi.status() == WL_CONNECTED) {
            Serial.print(" IP=");
            Serial.print(WiFi.localIP());
            Serial.print(" RSSI=");
            Serial.print(WiFi.RSSI());
            Serial.print(" dBm");
        }

        Serial.println();
    }
}
