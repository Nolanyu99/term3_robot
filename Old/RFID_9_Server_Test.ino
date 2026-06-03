#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <MFRC522_I2C.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// =====================================================
// RFID + server test
//
// Send 9 in Serial Monitor to:
//   1. scan one RFID chip
//   2. send it to the server
//   3. print the raw message received
//   4. print fertility
//   5. print x=... y=...
//
// Serial Monitor baud: 115200
// =====================================================

constexpr unsigned long SERIAL_BAUD = 115200;

// ---------- CHANGE THESE TO MATCH YOUR PROJECT ----------
constexpr const char* WIFI_SSID = "PhaseSpaceNetwork_2.4G";
constexpr const char* WIFI_PASSWORD = "8igMacNet";
constexpr const char* BROKER_HOST = "192.168.0.74";
constexpr uint16_t BROKER_PORT = 1883;

constexpr const char* TEAM_ID = "9";
constexpr const char* BOARD_ID = "dictator";
constexpr const char* SERVER_BOARD_ID = "server";
// -------------------------------------------------------

// RFID on Arduino GIGA SDA1/SCL1 using Wire1.
constexpr uint8_t RFID_ADDR = 0x28;
constexpr int RFID_RESET_PIN = -1;

MFRC522_I2C rfid(RFID_ADDR, RFID_RESET_PIN, &Wire1);

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

char boardSubscription[96];
char groupSubscription[96];

unsigned long readingNumber = 0;

char latestRawMessage[256] = {};
bool latestReplyReceived = false;

int latestFertility = -1;
int latestX = -1;
int latestY = -1;

// =====================================================
// RFID helpers
// =====================================================

void printHex2(byte value)
{
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void uidToText(char* out, size_t outSize)
{
  if (out == nullptr || outSize == 0) {
    return;
  }

  out[0] = '\0';

  for (byte i = 0; i < rfid.uid.size; ++i) {
    char part[3];
    snprintf(part, sizeof(part), "%02X", rfid.uid.uidByte[i]);
    strncat(out, part, outSize - strlen(out) - 1);
  }
}

bool i2cDevicePresent(uint8_t address)
{
  Wire1.beginTransmission(address);
  return Wire1.endTransmission() == 0;
}

bool rfidFirmwareVersionValid(byte version)
{
  return version == 0x15 ||
         version == 0x90 ||
         version == 0x91 ||
         version == 0x92 ||
         version == 0xB2;
}

void initReaderWithoutSoftReset()
{
  rfid.PCD_WriteRegister(rfid.TModeReg, 0x80);
  rfid.PCD_WriteRegister(rfid.TPrescalerReg, 0xA9);
  rfid.PCD_WriteRegister(rfid.TReloadRegH, 0x03);
  rfid.PCD_WriteRegister(rfid.TReloadRegL, 0xE8);
  rfid.PCD_WriteRegister(rfid.TxASKReg, 0x40);
  rfid.PCD_WriteRegister(rfid.ModeReg, 0x3D);
  rfid.PCD_AntennaOn();
}

void setupRFID()
{
  Wire1.begin();
  delay(50);

  if (!i2cDevicePresent(RFID_ADDR)) {
    Serial.println("rfid_error=no_i2c_device_at_0x28_on_SDA1_SCL1");
    return;
  }

  byte version = rfid.PCD_ReadRegister(rfid.VersionReg);

  Serial.print("rfid_firmware=0x");
  printHex2(version);
  Serial.println();

  if (!rfidFirmwareVersionValid(version)) {
    Serial.println("rfid_warning=firmware_version_not_recognised");
  }

  initReaderWithoutSoftReset();
}

bool scanRFIDOnce(char* uidOut, size_t uidOutSize, unsigned long timeoutMs)
{
  unsigned long startMs = millis();

  while (millis() - startMs < timeoutMs) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      uidToText(uidOut, uidOutSize);
      rfid.PICC_HaltA();
      return true;
    }

    delay(10);
  }

  return false;
}

// =====================================================
// Message parsing
// =====================================================

// This checks that a token is exactly key=...
// Example:
//   tokenHasKey("x=1", "x") returns true
//   tokenHasKey("type=isFertileReply", "y") returns false
bool tokenHasKey(const char* token, const char* key)
{
  if (token == nullptr || key == nullptr) {
    return false;
  }

  size_t keyLen = strlen(key);
  return strncmp(token, key, keyLen) == 0 && token[keyLen] == '=';
}

// Parses integer fields safely from space-separated fields.
// This prevents the bug where the parser sees the "y" in "type".
bool parseFieldIntFromTokens(const char* text, const char* key, int* out)
{
  if (text == nullptr || key == nullptr || out == nullptr) {
    return false;
  }

  char copy[256];
  strncpy(copy, text, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char* savePtr = nullptr;
  char* token = strtok_r(copy, " \t\r\n,;", &savePtr);

  while (token != nullptr) {
    if (tokenHasKey(token, key)) {
      const char* valueText = strchr(token, '=');

      if (valueText != nullptr) {
        *out = atoi(valueText + 1);
        return true;
      }
    }

    token = strtok_r(nullptr, " \t\r\n,;", &savePtr);
  }

  return false;
}

int parseFertilityValue(const char* text)
{
  if (text == nullptr) {
    return -1;
  }

  char copy[256];
  strncpy(copy, text, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  const char* keys[] = {
    "fertility",
    "fertile",
    "is_fertile",
    "state",
    "cell"
  };

  char* savePtr = nullptr;
  char* token = strtok_r(copy, " \t\r\n,;", &savePtr);

  while (token != nullptr) {
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
      if (tokenHasKey(token, keys[i])) {
        const char* valueText = strchr(token, '=');

        if (valueText == nullptr) {
          return -1;
        }

        valueText++;

        if (strcmp(valueText, "true") == 0 ||
            strcmp(valueText, "TRUE") == 0 ||
            strcmp(valueText, "yes") == 0 ||
            strcmp(valueText, "YES") == 0) {
          return 1;
        }

        if (strcmp(valueText, "false") == 0 ||
            strcmp(valueText, "FALSE") == 0 ||
            strcmp(valueText, "no") == 0 ||
            strcmp(valueText, "NO") == 0) {
          return 0;
        }

        return atoi(valueText);
      }
    }

    token = strtok_r(nullptr, " \t\r\n,;", &savePtr);
  }

  return -1;
}

bool parsePosition(const char* text, int* x, int* y)
{
  if (x != nullptr) {
    *x = -1;
  }

  if (y != nullptr) {
    *y = -1;
  }

  if (text == nullptr) {
    return false;
  }

  bool gotX = false;
  bool gotY = false;

  if (x != nullptr) {
    gotX = parseFieldIntFromTokens(text, "x", x);
  }

  if (y != nullptr) {
    gotY = parseFieldIntFromTokens(text, "y", y);
  }

  return gotX && gotY;
}

// =====================================================
// WiFi and MQTT
// =====================================================

void connectWiFi()
{
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.print("wifi_connecting_ssid=");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startMs = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 15000UL) {
    delay(250);
    Serial.print('.');
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("wifi_connected_ip=");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("wifi_error=not_connected");
  }
}

bool connectMQTT()
{
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  if (mqttClient.connected()) {
    return true;
  }

  char clientId[40];
  snprintf(clientId, sizeof(clientId), "rfid-test-%s", BOARD_ID);
  mqttClient.setId(clientId);

  Serial.print("mqtt_connecting_broker=");
  Serial.print(BROKER_HOST);
  Serial.print(':');
  Serial.println(BROKER_PORT);

  if (!mqttClient.connect(BROKER_HOST, BROKER_PORT)) {
    Serial.print("mqtt_error=");
    Serial.println(mqttClient.connectError());
    return false;
  }

  snprintf(boardSubscription, sizeof(boardSubscription),
           "lab/g/%s/from/+/to/%s",
           TEAM_ID,
           BOARD_ID);

  snprintf(groupSubscription, sizeof(groupSubscription),
           "lab/g/%s/from/+/to/all",
           TEAM_ID);

  bool sub1 = mqttClient.subscribe(boardSubscription);
  bool sub2 = mqttClient.subscribe(groupSubscription);

  Serial.print("mqtt_connected subscriptions_ok=");
  Serial.println((sub1 && sub2) ? 1 : 0);

  return sub1 && sub2;
}

bool sendIsFertileRequest(const char* uid)
{
  if (!connectMQTT()) {
    return false;
  }

  char topic[96];
  snprintf(topic, sizeof(topic),
           "lab/g/%s/from/%s/to/%s",
           TEAM_ID,
           BOARD_ID,
           SERVER_BOARD_ID);

  char message[128];
  snprintf(message, sizeof(message),
           "type=isFertile tag_id=%s board_id=%s",
           uid,
           BOARD_ID);

  Serial.print("sent_message=");
  Serial.println(message);

  mqttClient.beginMessage(topic);
  mqttClient.print(message);

  return mqttClient.endMessage() == 1;
}

bool readOneMqttMessage()
{
  int size = mqttClient.parseMessage();

  if (size <= 0) {
    return false;
  }

  size_t index = 0;

  while (mqttClient.available() && index + 1 < sizeof(latestRawMessage)) {
    int c = mqttClient.read();

    if (c < 0) {
      break;
    }

    latestRawMessage[index++] = static_cast<char>(c);
  }

  latestRawMessage[index] = '\0';

  Serial.print("raw_message=");
  Serial.println(latestRawMessage);

  latestFertility = parseFertilityValue(latestRawMessage);

  latestX = -1;
  latestY = -1;
  parsePosition(latestRawMessage, &latestX, &latestY);

  latestReplyReceived = true;

  return true;
}

bool messageLooksLikeFertilityReply(const char* message)
{
  if (message == nullptr) {
    return false;
  }

  return strstr(message, "type=isFertile") != nullptr ||
         strstr(message, "type=isFertileReply") != nullptr ||
         strstr(message, "fertile=") != nullptr ||
         strstr(message, "fertility=") != nullptr;
}

bool waitForServerReply(unsigned long timeoutMs)
{
  latestRawMessage[0] = '\0';
  latestReplyReceived = false;
  latestFertility = -1;
  latestX = -1;
  latestY = -1;

  unsigned long startMs = millis();

  while (millis() - startMs < timeoutMs) {
    if (!mqttClient.connected()) {
      connectMQTT();
    }

    if (mqttClient.connected() && readOneMqttMessage()) {
      if (messageLooksLikeFertilityReply(latestRawMessage)) {
        return true;
      }
    }

    delay(5);
  }

  return false;
}

// =====================================================
// Main test action
// =====================================================

void scanRFIDAndAskServer()
{
  readingNumber++;

  Serial.print("reading_number=");
  Serial.println(readingNumber);

  Serial.println("rfid_status=scanning");

  char uid[32];

  if (!scanRFIDOnce(uid, sizeof(uid), 5000UL)) {
    Serial.println("rfid_error=no_chip_found");
    return;
  }

  Serial.print("rfid_uid=");
  Serial.println(uid);

  connectWiFi();

  if (!sendIsFertileRequest(uid)) {
    Serial.println("server_error=could_not_send_isFertile");
    return;
  }

  bool gotReply = waitForServerReply(3000UL);

  if (!gotReply) {
    Serial.println("raw_message=NO_REPLY");
    Serial.println("fertility=-1");
    Serial.println("x=-1 y=-1");
    return;
  }

  Serial.print("fertility=");
  Serial.println(latestFertility);

  Serial.print("x=");
  Serial.print(latestX);
  Serial.print(" y=");
  Serial.println(latestY);
}

// =====================================================
// Arduino setup and loop
// =====================================================

void setup()
{
  Serial.begin(SERIAL_BAUD);

  unsigned long startMs = millis();

  while (!Serial && millis() - startMs < 3000UL) {
    delay(10);
  }

  Serial.println("RFID server test ready. Send 9 to scan one RFID chip.");

  setupRFID();
  connectWiFi();
  connectMQTT();
}

void loop()
{
  if (mqttClient.connected()) {
    mqttClient.poll();
  }

  if (Serial.available() > 0) {
    char command = static_cast<char>(Serial.read());

    if (command == '9') {
      scanRFIDAndAskServer();
    }
  }
}