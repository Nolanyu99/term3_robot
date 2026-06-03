// ======================================================
// Non-blocking WiFi/MQTT layer for server communication
// Used by Main, ArenaPathfinding, and RobotBehaviour
// ======================================================

// ============================= PLEASE CONSIDER =============================
// The robot can run with no hotspot. WiFi and MQTT reconnect attempts happen
// in messagesLoop, so movement code is not frozen by network failure.
// ===========================================================================

#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <string.h>
#include <stdio.h>


// Wifi parameters

constexpr const char* MESSAGE_WIFI_SSID     = robot_config::WIFI_SSID;
constexpr const char* MESSAGE_WIFI_PASSWORD = robot_config::WIFI_PASSWORD;

constexpr const char* MESSAGE_BROKER_HOST = "192.168.0.74";
constexpr uint16_t MESSAGE_BROKER_PORT = 1883;

constexpr const char* MESSAGE_TEAM_ID  = "9";
constexpr const char* MESSAGE_BOARD_ID = "dictator";
constexpr const char* MESSAGE_SERVER_BOARD_ID = "server";

constexpr bool MESSAGE_REQUIRE_REMOTE_ENABLE = false;

constexpr bool MESSAGE_BROADCAST_HEARTBEAT = true;
constexpr bool MESSAGE_DEBUG_PRINT_INCOMING = true;
constexpr bool MESSAGE_DEBUG_PRINT_OUTGOING = true;

constexpr unsigned long MESSAGE_WIFI_RETRY_INTERVAL_MS = 5000;
constexpr unsigned long MESSAGE_MQTT_RETRY_INTERVAL_MS = 3000;
constexpr unsigned long MESSAGE_REGISTER_INTERVAL_MS = 10000;
constexpr unsigned long MESSAGE_HEARTBEAT_TIMEOUT_MS = 3000;
constexpr unsigned long MESSAGE_HEARTBEAT_BROADCAST_INTERVAL_MS = 1000;


// Connection state

WiFiClient message_wifi_client;
MqttClient message_mqtt_client(message_wifi_client);

char message_client_id[24] = {};
char message_status_topic[96] = {};
char message_board_subscription[96] = {};
char message_group_subscription[96] = {};

bool message_messaging_started = false;
bool message_mqtt_subscribed = false;
unsigned long message_last_wifi_attempt_ms = 0;
unsigned long message_last_mqtt_attempt_ms = 0;


// Remote safety and latest server data

bool message_remote_enabled = false;
bool message_emergency_active = false;
bool message_disable_active = false;
bool message_team_emergency = false;
bool message_reentry_requested = false;

unsigned long message_last_heartbeat_ms = 0;
unsigned long message_last_register_ms = 0;
unsigned long message_last_broadcast_heartbeat_ms = 0;
unsigned long message_seq = 0;

uint8_t message_last_map[21] = {};
bool message_have_map = false;

char message_last_fertility_tag[32] = {};
int8_t message_last_fertility_x = -1;
int8_t message_last_fertility_y = -1;
uint8_t message_last_fertility_state = 0;
bool message_have_fertility_reply = false;
unsigned long message_last_fertility_reply_ms = 0;


// Parsing and topic helpers

static void copyPayloadToText(const uint8_t* payload, size_t length, char* out, size_t out_size)
{
  if (out_size == 0) {
    return;
  }

  const size_t copy_len = (length < out_size - 1) ? length : out_size - 1;
  memcpy(out, payload, copy_len);
  out[copy_len] = '\0';
}

static bool textContains(const char* text, const char* needle)
{
  return text != nullptr && needle != nullptr && strstr(text, needle) != nullptr;
}

// Returns true iff `token` starts with exactly `key=`.
// This is what lets us avoid matching the "y" inside "type" or
// the "x" inside "extra".
static bool tokenHasKey(const char* token, const char* key)
{
  if (token == nullptr || key == nullptr) {
    return false;
  }

  const size_t key_len = strlen(key);
  return strncmp(token, key, key_len) == 0 && token[key_len] == '=';
}

// Token-aware integer field parser. Splits `text` on whitespace +
// common separators, then looks for an exact `key=` token. Fixes
// the strstr() false-match bug the previous version had.
static bool parseFieldInt(const char* text, const char* key, int* out)
{
  if (text == nullptr || key == nullptr || out == nullptr) {
    return false;
  }

  char copy[256];
  strncpy(copy, text, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char* save_ptr = nullptr;
  char* token = strtok_r(copy, " \t\r\n,;", &save_ptr);

  while (token != nullptr) {
    if (tokenHasKey(token, key)) {
      const char* value_text = strchr(token, '=');
      if (value_text != nullptr) {
        *out = atoi(value_text + 1);
        return true;
      }
    }
    token = strtok_r(nullptr, " \t\r\n,;", &save_ptr);
  }

  return false;
}

// Token-aware text field parser. Same idea as parseFieldInt but
// copies the value out as a string.
static bool parseFieldText(const char* text, const char* key, char* out, size_t out_size)
{
  if (text == nullptr || key == nullptr || out == nullptr || out_size == 0) {
    return false;
  }

  out[0] = '\0';

  char copy[256];
  strncpy(copy, text, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char* save_ptr = nullptr;
  char* token = strtok_r(copy, " \t\r\n,;", &save_ptr);

  while (token != nullptr) {
    if (tokenHasKey(token, key)) {
      const char* value_text = strchr(token, '=');
      if (value_text != nullptr) {
        strncpy(out, value_text + 1, out_size - 1);
        out[out_size - 1] = '\0';
        return out[0] != '\0';
      }
    }
    token = strtok_r(nullptr, " \t\r\n,;", &save_ptr);
  }

  return false;
}

// Maps fertility-style fields to the 0..3 cell state.
// Accepts numeric (0/1/2/3) and string (true/false/yes/no) values
// across the keys: state, cell, fertility, fertile, is_fertile.
static uint8_t stateFromFertilityFields(const char* msg)
{
  int state = -1;
  if (parseFieldInt(msg, "state", &state) || parseFieldInt(msg, "cell", &state)) {
    if (state >= 0 && state <= 3) {
      return static_cast<uint8_t>(state);
    }
  }

  // Try the boolean-style fertility keys, accepting true/false/yes/no.
  const char* bool_keys[] = {"fertility", "fertile", "is_fertile"};
  for (size_t i = 0; i < sizeof(bool_keys) / sizeof(bool_keys[0]); ++i) {
    char value[16] = {};
    if (parseFieldText(msg, bool_keys[i], value, sizeof(value))) {
      if (strcmp(value, "true") == 0 || strcmp(value, "TRUE") == 0 ||
          strcmp(value, "yes") == 0 || strcmp(value, "YES") == 0 ||
          strcmp(value, "1") == 0) {
        return 2;  // fertile
      }
      if (strcmp(value, "false") == 0 || strcmp(value, "FALSE") == 0 ||
          strcmp(value, "no") == 0 || strcmp(value, "NO") == 0 ||
          strcmp(value, "0") == 0) {
        return 3;  // infertile
      }
      // Numeric fallback (e.g. "2" or "3")
      const int n = atoi(value);
      if (n >= 0 && n <= 3) {
        return static_cast<uint8_t>(n);
      }
    }
  }

  return 0;  // unknown
}

static void buildMessageTopics()
{
  snprintf(message_client_id, sizeof(message_client_id),
           "g%02d-b%02d", atoi(MESSAGE_TEAM_ID), atoi(MESSAGE_BOARD_ID));

  snprintf(message_status_topic, sizeof(message_status_topic),
           "lab/g/%s/board/%s/status", MESSAGE_TEAM_ID, MESSAGE_BOARD_ID);

  snprintf(message_board_subscription, sizeof(message_board_subscription),
           "lab/g/%s/from/+/to/%s", MESSAGE_TEAM_ID, MESSAGE_BOARD_ID);

  snprintf(message_group_subscription, sizeof(message_group_subscription),
           "lab/g/%s/from/+/to/all", MESSAGE_TEAM_ID);
}

static bool parseTopicFromBoard(const char* topic, char* from_board, size_t from_board_size)
{
  if (from_board == nullptr || from_board_size == 0) {
    return false;
  }

  from_board[0] = '\0';

  char group_id[16] = {};
  char target[16] = {};

  const int matched = sscanf(topic,
                             "lab/g/%15[^/]/from/%15[^/]/to/%15[^/]",
                             group_id,
                             from_board,
                             target);
  return matched == 3;
}

void handleIncomingMessage(const char* from_board_id, const uint8_t* payload, size_t length)
{
  if (length == 6) {
    message_team_emergency = payload[4] == 1;
    message_reentry_requested = payload[5] == 1;

    if (message_team_emergency) {
      message_remote_enabled = false;
      Serial.println("MESSAGE: team emergency active");
    }
    return;
  }

  if (length == 21) {
    memcpy(message_last_map, payload, 21);
    message_have_map = true;
    Serial.println("MESSAGE: received 21-byte map");
    return;
  }

  char msg[256];
  copyPayloadToText(payload, length, msg, sizeof(msg));

  if (textContains(msg, "type=isFertile") ||
      textContains(msg, "type=fertility") ||
      textContains(msg, "fertile=")) {
    int x = -1;
    int y = -1;

    bool have_x = parseFieldInt(msg, "x", &x) ||
                  parseFieldInt(msg, "grid_x", &x) ||
                  parseFieldInt(msg, "pos_x", &x);
    bool have_y = parseFieldInt(msg, "y", &y) ||
                  parseFieldInt(msg, "grid_y", &y) ||
                  parseFieldInt(msg, "pos_y", &y);

    if (have_x && have_y && x >= 0 && x <= 10 && y >= 0 && y <= 10) {
      char tag[32] = {};
      if (!parseFieldText(msg, "tag_id", tag, sizeof(tag)) &&
          !parseFieldText(msg, "uid", tag, sizeof(tag)) &&
          !parseFieldText(msg, "tag", tag, sizeof(tag))) {
        tag[0] = '\0';
      }

      strncpy(message_last_fertility_tag, tag, sizeof(message_last_fertility_tag) - 1);
      message_last_fertility_tag[sizeof(message_last_fertility_tag) - 1] = '\0';
      message_last_fertility_x = static_cast<int8_t>(x);
      message_last_fertility_y = static_cast<int8_t>(y);
      message_last_fertility_state = stateFromFertilityFields(msg);
      message_have_fertility_reply = true;
      message_last_fertility_reply_ms = millis();

      Serial.print("MESSAGE: fertility/localisation reply x=");
      Serial.print(message_last_fertility_x);
      Serial.print(" y=");
      Serial.print(message_last_fertility_y);
      Serial.print(" state=");
      Serial.println(message_last_fertility_state);
    }
  }

  if (textContains(msg, "type=reviveNeeded") ||
      textContains(msg, "type=robotDown") ||
      textContains(msg, "type=needsRevive") ||
      textContains(msg, "needs_revive=1")) {
    int x = -1;
    int y = -1;

    const bool have_x = parseFieldInt(msg, "x", &x) ||
                        parseFieldInt(msg, "grid_x", &x) ||
                        parseFieldInt(msg, "pos_x", &x);
    const bool have_y = parseFieldInt(msg, "y", &y) ||
                        parseFieldInt(msg, "grid_y", &y) ||
                        parseFieldInt(msg, "pos_y", &y);

    char robot_id[16] = {};
    if (!parseFieldText(msg, "robot_id", robot_id, sizeof(robot_id)) &&
        !parseFieldText(msg, "target_board", robot_id, sizeof(robot_id)) &&
        !parseFieldText(msg, "board_id", robot_id, sizeof(robot_id)) &&
        !parseFieldText(msg, "id", robot_id, sizeof(robot_id))) {
      strncpy(robot_id, from_board_id, sizeof(robot_id) - 1);
      robot_id[sizeof(robot_id) - 1] = '\0';
    }

    if (have_x && have_y && x >= 0 && x <= 10 && y >= 0 && y <= 10) {
      test8AddOrUpdateReviveTarget(robot_id,
                                   static_cast<int8_t>(x),
                                   static_cast<int8_t>(y));
    }
  }

  if (textContains(msg, "type=reviveConfirmed") ||
      textContains(msg, "type=reviveComplete") ||
      textContains(msg, "type=revived") ||
      textContains(msg, "revive_confirmed=1")) {
    char robot_id[16] = {};
    if (!parseFieldText(msg, "robot_id", robot_id, sizeof(robot_id)) &&
        !parseFieldText(msg, "target_board", robot_id, sizeof(robot_id)) &&
        !parseFieldText(msg, "board_id", robot_id, sizeof(robot_id)) &&
        !parseFieldText(msg, "id", robot_id, sizeof(robot_id))) {
      strncpy(robot_id, from_board_id, sizeof(robot_id) - 1);
      robot_id[sizeof(robot_id) - 1] = '\0';
    }

    test8MarkReviveConfirmed(robot_id);
  }

  if (textContains(msg, "type=heartbeat")) {
    message_last_heartbeat_ms = millis();

    if (textContains(msg, "enable=1")) {
      if (!message_emergency_active && !message_disable_active && !message_team_emergency) {
        message_remote_enabled = true;
      }
    } else if (textContains(msg, "enable=0")) {
      message_remote_enabled = false;
      Serial.println("MESSAGE: heartbeat disabled movement");
    }
  }

  if (textContains(msg, "type=emergency")) {
    if (textContains(msg, "enabled=true") || textContains(msg, "enable=1")) {
      message_emergency_active = true;
      message_remote_enabled = false;
      Serial.println("MESSAGE: emergency stop active");
    } else if (textContains(msg, "enabled=false") || textContains(msg, "enable=0")) {
      message_emergency_active = false;
      Serial.println("MESSAGE: emergency cleared");
    }
  }

  if (textContains(msg, "type=disable")) {
    if (textContains(msg, "enabled=false") || textContains(msg, "enable=0")) {
      message_disable_active = true;
      message_remote_enabled = false;
      Serial.println("MESSAGE: robot disabled remotely");
    } else if (textContains(msg, "enabled=true") || textContains(msg, "enable=1")) {
      message_disable_active = false;
      Serial.println("MESSAGE: robot disable cleared");
    }
  }

  if (MESSAGE_DEBUG_PRINT_INCOMING) {
    Serial.print("MESSAGE from ");
    Serial.print(from_board_id != nullptr ? from_board_id : "?");
    Serial.print(": ");
    Serial.println(msg);
  }
}

static void ensureMessageWiFiConnected()
{
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  const unsigned long now = millis();
  if (now - message_last_wifi_attempt_ms < MESSAGE_WIFI_RETRY_INTERVAL_MS) {
    return;
  }

  message_last_wifi_attempt_ms = now;
  message_mqtt_subscribed = false;
  message_mqtt_client.stop();

  Serial.print("MESSAGE: trying WiFi SSID=");
  Serial.println(MESSAGE_WIFI_SSID);

  WiFi.disconnect();
  WiFi.begin(MESSAGE_WIFI_SSID, MESSAGE_WIFI_PASSWORD);
}

static bool publishRaw(const char* topic, const uint8_t* data, size_t length, bool retain, uint8_t qos)
{
  if (!message_mqtt_client.connected() || topic == nullptr || data == nullptr || length == 0) {
    return false;
  }

  if (!message_mqtt_client.beginMessage(topic, static_cast<int>(length), retain, qos, false)) {
    return false;
  }

  const size_t written = message_mqtt_client.write(data, length);
  if (written != length) {
    message_mqtt_client.stop();
    message_mqtt_subscribed = false;
    return false;
  }

  return message_mqtt_client.endMessage() == 1;
}

static bool publishStatus(const char* status)
{
  return publishRaw(message_status_topic,
                    reinterpret_cast<const uint8_t*>(status),
                    strlen(status),
                    true,
                    1);
}

static bool subscribeMessageTopics()
{
  if (message_mqtt_subscribed) {
    return true;
  }

  if (!message_mqtt_client.subscribe(message_board_subscription)) {
    return false;
  }

  if (!message_mqtt_client.subscribe(message_group_subscription)) {
    return false;
  }

  message_mqtt_subscribed = true;
  Serial.println("MESSAGE: MQTT subscriptions ready");
  return true;
}

static bool connectMessageMqtt()
{
  message_mqtt_client.stop();
  message_mqtt_client.setId(message_client_id);

  const char* will_payload = "offline";
  message_mqtt_client.beginWill(message_status_topic,
                                strlen(will_payload),
                                true,
                                0);
  message_mqtt_client.print(will_payload);
  message_mqtt_client.endWill();

  Serial.print("MESSAGE: trying MQTT broker=");
  Serial.print(MESSAGE_BROKER_HOST);
  Serial.print(":");
  Serial.println(MESSAGE_BROKER_PORT);

  if (!message_mqtt_client.connect(MESSAGE_BROKER_HOST, MESSAGE_BROKER_PORT)) {
    Serial.print("MESSAGE: MQTT connect failed, error=");
    Serial.println(message_mqtt_client.connectError());
    return false;
  }

  message_mqtt_subscribed = false;

  if (!subscribeMessageTopics()) {
    message_mqtt_client.stop();
    message_mqtt_subscribed = false;
    return false;
  }

  publishStatus("online");
  Serial.println("MESSAGE: MQTT connected");
  return true;
}

static void ensureMessageMqttConnected()
{
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (message_mqtt_client.connected()) {
    if (!message_mqtt_subscribed) {
      subscribeMessageTopics();
    }
    return;
  }

  message_mqtt_subscribed = false;

  const unsigned long now = millis();
  if (now - message_last_mqtt_attempt_ms < MESSAGE_MQTT_RETRY_INTERVAL_MS) {
    return;
  }

  message_last_mqtt_attempt_ms = now;
  connectMessageMqtt();
}

static void pollMessages()
{
  const int message_size = message_mqtt_client.parseMessage();
  if (message_size <= 0) {
    return;
  }

  const String topic_string = message_mqtt_client.messageTopic();
  char from_board_id[16] = {};
  if (!parseTopicFromBoard(topic_string.c_str(), from_board_id, sizeof(from_board_id))) {
    while (message_mqtt_client.available()) {
      message_mqtt_client.read();
    }
    return;
  }

  if (message_size > 512) {
    while (message_mqtt_client.available()) {
      message_mqtt_client.read();
    }
    return;
  }

  uint8_t payload[512];
  size_t offset = 0;

  while (message_mqtt_client.available() && offset < sizeof(payload)) {
    const int value = message_mqtt_client.read();
    if (value < 0) {
      break;
    }
    payload[offset++] = static_cast<uint8_t>(value);
  }

  if (offset > 0) {
    handleIncomingMessage(from_board_id, payload, offset);
  }
}

void setupMessages()
{
  buildMessageTopics();

  message_messaging_started = true;
  message_last_heartbeat_ms = millis();
  message_last_register_ms = 0;
  message_last_broadcast_heartbeat_ms = 0;

  Serial.println("MESSAGE: messaging enabled; robot will keep running if WiFi is unavailable");
  ensureMessageWiFiConnected();
}

void messagesLoop()
{
  if (!message_messaging_started) {
    return;
  }

  ensureMessageWiFiConnected();
  ensureMessageMqttConnected();

  if (message_mqtt_client.connected()) {
    pollMessages();
  }

  const unsigned long now = millis();

  if (MESSAGE_REQUIRE_REMOTE_ENABLE &&
      message_remote_enabled &&
      now - message_last_heartbeat_ms > MESSAGE_HEARTBEAT_TIMEOUT_MS) {
    message_remote_enabled = false;
    Serial.println("MESSAGE: heartbeat timeout, movement disabled");
  }

  if (message_mqtt_client.connected() &&
      (now - message_last_register_ms >= MESSAGE_REGISTER_INTERVAL_MS ||
       message_last_register_ms == 0)) {
    message_last_register_ms = now;
    messagesSendRegister();
  }

  if (message_mqtt_client.connected() &&
      MESSAGE_BROADCAST_HEARTBEAT &&
      (now - message_last_broadcast_heartbeat_ms >= MESSAGE_HEARTBEAT_BROADCAST_INTERVAL_MS ||
       message_last_broadcast_heartbeat_ms == 0)) {
    message_last_broadcast_heartbeat_ms = now;
    messagesBroadcastHeartbeat();
  }
}

bool messagesConnected()
{
  return WiFi.status() == WL_CONNECTED && message_mqtt_client.connected();
}

bool messagesRemoteEnabled()
{
  return message_remote_enabled;
}

bool messagesEmergencyActive()
{
  return message_emergency_active || message_team_emergency;
}

bool messagesDisableActive()
{
  return message_disable_active;
}

bool messagesRobotAllowedToMove()
{
  if (!MESSAGE_REQUIRE_REMOTE_ENABLE) {
    return !messagesEmergencyActive() && !messagesDisableActive();
  }
  return message_remote_enabled && !messagesEmergencyActive() && !messagesDisableActive();
}

bool messagesHaveMap()
{
  return message_have_map;
}

bool messagesCopyLastMap(uint8_t* out_map21, size_t out_size)
{
  if (!message_have_map || out_map21 == nullptr || out_size < 21) {
    return false;
  }

  memcpy(out_map21, message_last_map, 21);
  return true;
}

bool messagesGetLastFertilityReply(const char* tag_id, int8_t* x, int8_t* y, uint8_t* state)
{
  if (!message_have_fertility_reply || x == nullptr || y == nullptr || state == nullptr) {
    return false;
  }

  if (tag_id != nullptr && tag_id[0] != '\0' && message_last_fertility_tag[0] != '\0' &&
      strcmp(tag_id, message_last_fertility_tag) != 0) {
    return false;
  }

  *x = message_last_fertility_x;
  *y = message_last_fertility_y;
  *state = message_last_fertility_state;
  return true;
}

bool messagesWaitForFertilityReply(const char* tag_id, unsigned long timeout_ms, int8_t* x, int8_t* y, uint8_t* state)
{
  const unsigned long start_ms = millis();
  const unsigned long previous_reply_ms = message_last_fertility_reply_ms;

  while (millis() - start_ms < timeout_ms) {
    messagesLoop();

    if (message_last_fertility_reply_ms != previous_reply_ms &&
        messagesGetLastFertilityReply(tag_id, x, y, state)) {
      return true;
    }

    delay(5);
  }

  return false;
}

bool messagesSendToBoard(const char* target_board_id, const uint8_t* data, size_t length)
{
  if (target_board_id == nullptr || data == nullptr || length == 0) {
    return false;
  }

  char topic[96];
  snprintf(topic, sizeof(topic),
           "lab/g/%s/from/%s/to/%s",
           MESSAGE_TEAM_ID,
           MESSAGE_BOARD_ID,
           target_board_id);

  return publishRaw(topic, data, length, false, 0);
}

bool messagesSendToBoard(const char* target_board_id, const char* text)
{
  if (target_board_id == nullptr || text == nullptr) {
    return false;
  }

  return messagesSendToBoard(target_board_id,
                             reinterpret_cast<const uint8_t*>(text),
                             strlen(text));
}

bool messagesSendToBoard(uint8_t target_board_id, const char* text)
{
  char target[8];
  snprintf(target, sizeof(target), "%u", target_board_id);
  return messagesSendToBoard(target, text);
}

bool messagesSendToBoard(uint8_t target_board_id, const uint8_t* data, size_t length)
{
  char target[8];
  snprintf(target, sizeof(target), "%u", target_board_id);
  return messagesSendToBoard(target, data, length);
}

bool messagesSendToGroup(const uint8_t* data, size_t length)
{
  if (data == nullptr || length == 0) {
    return false;
  }

  char topic[96];
  snprintf(topic, sizeof(topic),
           "lab/g/%s/from/%s/to/all",
           MESSAGE_TEAM_ID,
           MESSAGE_BOARD_ID);

  return publishRaw(topic, data, length, false, 0);
}

bool messagesSendToGroup(const char* text)
{
  if (text == nullptr) {
    return false;
  }

  return messagesSendToGroup(reinterpret_cast<const uint8_t*>(text), strlen(text));
}

bool messagesSendRegister()
{
  char msg[96];
  snprintf(msg, sizeof(msg),
           "type=register team_id=%s board_id=%s",
           MESSAGE_TEAM_ID,
           MESSAGE_BOARD_ID);

  const bool ok = messagesSendToBoard(MESSAGE_SERVER_BOARD_ID, msg);

  if (MESSAGE_DEBUG_PRINT_OUTGOING) {
    Serial.print("MESSAGE send register ok=");
    Serial.println(ok ? 1 : 0);
  }

  return ok;
}

bool messagesBroadcastHeartbeat()
{
  char msg[160];
  snprintf(msg, sizeof(msg),
           "type=heartbeat team_id=%s board_id=%s enabled=%d seq=%lu uptime_ms=%lu",
           MESSAGE_TEAM_ID,
           MESSAGE_BOARD_ID,
           messagesRobotAllowedToMove() ? 1 : 0,
           message_seq++,
           millis());

  const bool ok = messagesSendToGroup(msg);

  if (MESSAGE_DEBUG_PRINT_OUTGOING) {
    Serial.print("MESSAGE send heartbeat ok=");
    Serial.println(ok ? 1 : 0);
  }

  return ok;
}

bool messagesSendIsFertile(const char* tag_id)
{
  char msg[128];
  snprintf(msg, sizeof(msg),
           "type=isFertile tag_id=%s board_id=%s",
           tag_id,
           MESSAGE_BOARD_ID);

  return messagesSendToBoard(MESSAGE_SERVER_BOARD_ID, msg);
}

bool messagesSendSeedPlanted(const char* tag_id)
{
  char msg[128];
  snprintf(msg, sizeof(msg),
           "type=seedPlanted tag_id=%s board_id=%s",
           tag_id,
           MESSAGE_BOARD_ID);

  return messagesSendToBoard(MESSAGE_SERVER_BOARD_ID, msg);
}

bool messagesSendOpenAirlock(char airlock, const char* tag_id)
{
  char msg[160];
  snprintf(msg, sizeof(msg),
           "type=openAirlock airlock=%c tag_id=%s board_id=%s",
           airlock,
           tag_id,
           MESSAGE_BOARD_ID);

  return messagesSendToBoard(MESSAGE_SERVER_BOARD_ID, msg);
}

bool messagesSendReviveRequest(const char* target_team, const char* target_board)
{
  char msg[128];
  snprintf(msg, sizeof(msg),
           "type=reviveRequest target_team=%s target_board=%s",
           target_team,
           target_board);

  return messagesSendToBoard(MESSAGE_SERVER_BOARD_ID, msg);
}

bool messagesSendGetMap()
{
  char msg[96];
  snprintf(msg, sizeof(msg),
           "type=getMap board_id=%s",
           MESSAGE_BOARD_ID);

  return messagesSendToBoard(MESSAGE_SERVER_BOARD_ID, msg);
}
