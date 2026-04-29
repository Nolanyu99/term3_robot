# Arduino GIGA R1 WiFi/upload smoke test

Source experiment: local `giga_test` PlatformIO project.

## What was validated

- Arduino GIGA R1 builds with PlatformIO using `ststm32`, `giga_r1_m7`, and Arduino framework.
- Windows DFU upload works after binding the DFU interface `2341:0366` interface 0 to WinUSB with Zadig.
- On this machine, PlatformIO's automatic 1200 bps serial reset can fail with `Cannot configure port`, so the project uses a pre-upload script to skip serial touch/flushing.
- Manual upload flow works: double-click reset to enter DFU, then run PlatformIO Upload.
- Serial Monitor on the normal COM port works at `115200`.
- Basic `WiFi.begin(ssid, password)` test reaches the connection loop and prints status over serial.

## Safe WiFi test sketch

Do not commit real WiFi credentials. Put temporary credentials in a local-only config file or edit them only while testing.

```cpp
#include <Arduino.h>
#include <WiFi.h>

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 1500) {
        delay(10);
    }

    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void loop() {}
```

## Notes

- Use a 2.4 GHz WiFi network or hotspot compatibility mode first.
- Prefer an ASCII SSID while debugging. Non-ASCII SSIDs can make serial/source encoding harder to reason about.
- If upload says `No DFU capable USB device available`, the board is probably not in DFU mode.
- If upload says `Cannot claim interface`, check Zadig and set `2341:0366` interface 0 to WinUSB.
