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

// This just allowes the robot to run without the heartbeat for conveniencee, TO BE CHANGED
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

// isFertile reply
char message_last_fertility_tag[32] = {};
int8_t message_last_fertility_x = -1;
int8_t message_last_fertility_y = -1;
uint8_t message_last_fertility_state = 0;
bool message_have_fertility_reply = false;
unsigned long message_last_fertility_reply_ms = 0;


// Parsing and topic helpers

// Copies binary MQTT payloads into a safe C string
static void copyPayloadToText(const uint8_t* payload, size_t length, char* out, size_t out_size)
{
  if (out_size == 0) {
    return;
  }

  const size_t copy_len = (length < out_size - 1) ? length : out_size - 1;
  memcpy(out, payload, copy_len);
  out[copy_len] = '\0';
}

// Null-safe substring check
static bool textContains(const char* text, const char* needle)
{
  return text != nullptr && needle != nullptr && strstr(text, needle) != nullptr;
}


// Reads fields such as x=3 or state=2
static bool parseFieldInt(const char* text, const char* key, int* out)
{
  if (text == nullptr || key == nullptr || out == nullptr) {
    return false;
  }

  const char* p = strstr(text, key);
  if (p == nullptr) {
    return false;
  }

  p += strlen(key);
  if (*p != '=') {
    return false;
  }
  ++p;

  *out = atoi(p);
  return true;
}

// Reads text fields such as tag_id=04A1B2
static bool parseFieldText(const char* text, const char* key, char* out, size_t out_size)
{
  if (text == nullptr || key == nullptr || out == nullptr || out_size == 0) {
    return false;
  }

  out[0] = '\0';
  const char* p = strstr(text, key);
  if (p == nullptr) {
    return false;
  }

  p += strlen(key);
  if (*p != '=') {
    return false;
  }
  ++p;

  size_t i = 0;
  while (*p != '\0' && *p != ' ' && *p != ',' && i + 1 < out_size) {
    out[i++] = *p++;
  }
  out[i] = '\0';
  return i > 0;
}

// Converts fertility fields into map state 0 to 3
static uint8_t stateFromFertilityFields(const char* msg)
{
  int state = -1;
  if (parseFieldInt(msg, "state", &state) || parseFieldInt(msg, "cell", &state)) {
    if (state >= 0 && state <= 3) {
      return static_cast<uint8_t>(state);
    }
  }

  int fertile = -1;
  if (parseFieldInt(msg, "fertile", &fertile) || parseFieldInt(msg, "is_fertile", &fertile)) {
    return fertile ? 2 : 3;
  }

  return 0;
}

// Builds client id, status topic, and subscriptions
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

// Extracts the sender board id from the MQTT topic
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

// -----------------------------------------------------
// Incoming message handler
// -----------------------------------------------------

// Updates safety flags, getMap cache, and fertility replies
void handleIncomingMessage(const char* from_board_id, const uint8_t* payload, size_t length)
{
  // Team status broadcast: 6 bytes.
  // Byte 4 = emergency, byte 5 = re-entry requested.
  if (length == 6) {
    message_team_emergency = payload[4] == 1;
    message_reentry_requested = payload[5] == 1;

    if (message_team_emergency) {
      message_remote_enabled = false;
      Serial.println("MESSAGE: team emergency active");
    }
    return;
  }

  // Occupancy map broadcast/reply: 21 bytes.
  if (length == 21) {
    memcpy(message_last_map, payload, 21);
    message_have_map = true;
    Serial.println("MESSAGE: received 21-byte map");
    return;
  }

  char msg[256];
  copyPayloadToText(payload, length, msg, sizeof(msg));

  // Fertility/localisation reply. The server should include the grid position.
  // Accepted position keys: x/y, grid_x/grid_y, pos_x/pos_y.
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

  // Test 8 revive target and confirmation messages.
  // Accepted target examples:
  //   type=reviveNeeded robot_id=4 x=3 y=6
  //   type=robotDown board_id=otherRobot grid_x=3 grid_y=6
  // Accepted confirmation examples:
  //   type=reviveConfirmed robot_id=4
  //   type=reviveComplete board_id=otherRobot
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

  // Server heartbeat: type=heartbeat enable=1/0 seq=... time_left=...
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

  // Emergency stop: type=emergency enabled=true
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

  // Individual disable: type=disable enabled=false reason=stranded
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

// -----------------------------------------------------
// Connection management
// -----------------------------------------------------

// Starts WiFi reconnect attempts without blocking driving
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

// Publishes bytes only when MQTT is connected
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

// Publishes retained online/offline state
static bool publishStatus(const char* status)
{
  return publishRaw(message_status_topic,
                    reinterpret_cast<const uint8_t*>(status),
                    strlen(status),
                    true,
                    1);
}

// Subscribes to direct board and group messages
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

// Connects MQTT and registers the robot with the server
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

// Keeps MQTT alive and reconnects after disconnection
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

// Reads pending MQTT packets and dispatches them
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

// -----------------------------------------------------
// Setup and loop
// -----------------------------------------------------

// Prepares topics and begins the first WiFi attempt
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

// Services WiFi, MQTT, register, and heartbeat
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

// -----------------------------------------------------
// Safety accessors
// -----------------------------------------------------

// Ensures wifi connection
bool messagesConnected()
{
  return WiFi.status() == WL_CONNECTED && message_mqtt_client.connected();
}

// Reports latest heartbeat enable flag
bool messagesRemoteEnabled()
{
  return message_remote_enabled;
}

// Reports emergency stop state
bool messagesEmergencyActive()
{
  return message_emergency_active || message_team_emergency;
}

// Reports remote disable state
bool messagesDisableActive()
{
  return message_disable_active;
}

// Central movement permission check
bool messagesRobotAllowedToMove()
{
  if (!MESSAGE_REQUIRE_REMOTE_ENABLE) {
    return !messagesEmergencyActive() && !messagesDisableActive();
  }
  return message_remote_enabled && !messagesEmergencyActive() && !messagesDisableActive();
}

// To check if a map was received
bool messagesHaveMap()
{
  return message_have_map;
}

// Copies the latest 21-byte map without exposing internals
bool messagesCopyLastMap(uint8_t* out_map21, size_t out_size)
{
  if (!message_have_map || out_map21 == nullptr || out_size < 21) {
    return false;
  }

  memcpy(out_map21, message_last_map, 21);
  return true;
}

// Returns the latest matching isFertile position reply
bool messagesGetLastFertilityReply(const char* tag_id, int8_t* x, int8_t* y, uint8_t* state)
{
  if (!message_have_fertility_reply || x == nullptr || y == nullptr || state == nullptr) {
    return false;
  }

  // If the server did not echo the tag, accept the latest reply. Otherwise,
  // require the reply to match the scanned UID.
  if (tag_id != nullptr && tag_id[0] != '\0' && message_last_fertility_tag[0] != '\0' &&
      strcmp(tag_id, message_last_fertility_tag) != 0) {
    return false;
  }

  *x = message_last_fertility_x;
  *y = message_last_fertility_y;
  *state = message_last_fertility_state;
  return true;
}

// Sends code time to receive an isFertile reply while servicing MQTT
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

// -----------------------------------------------------
// Send helpers
// -----------------------------------------------------

// Sends text or binary data to one board
bool messagesSendToBoard(const char* target_board_id, const char* text)
{
  if (target_board_id == nullptr || text == nullptr) {
    return false;
  }

  return messagesSendToBoard(target_board_id,
                             reinterpret_cast<const uint8_t*>(text),
                             strlen(text));
}

// Sends text or binary data to one board
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

// More of the same
bool messagesSendToBoard(uint8_t target_board_id, const char* text)
{
  char target[8];
  snprintf(target, sizeof(target), "%u", target_board_id);
  return messagesSendToBoard(target, text);
}

// More of the same
bool messagesSendToBoard(uint8_t target_board_id, const uint8_t* data, size_t length)
{
  char target[8];
  snprintf(target, sizeof(target), "%u", target_board_id);
  return messagesSendToBoard(target, data, length);
}

// Sends text or binary data to all boards in the group
bool messagesSendToGroup(const char* text)
{
  if (text == nullptr) {
    return false;
  }

  return messagesSendToGroup(reinterpret_cast<const uint8_t*>(text), strlen(text));
}

// More of the same
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

// -----------------------------------------------------
// Challenge protocol helpers
// -----------------------------------------------------

// Announces this board to the server periodically
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

// Broadcasts that this board is alive
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

// Asks the server for tag fertility and grid position
bool messagesSendIsFertile(const char* tag_id)
{
  char msg[128];
  snprintf(msg, sizeof(msg),
           "type=isFertile tag_id=%s board_id=%s",
           tag_id,
           MESSAGE_BOARD_ID);

  return messagesSendToBoard(MESSAGE_SERVER_BOARD_ID, msg);
}

// Tells the server which tag was seeded
bool messagesSendSeedPlanted(const char* tag_id)
{
  char msg[128];
  snprintf(msg, sizeof(msg),
           "type=seedPlanted tag_id=%s board_id=%s",
           tag_id,
           MESSAGE_BOARD_ID);

  return messagesSendToBoard(MESSAGE_SERVER_BOARD_ID, msg);
}

// Requests an airlock opening from the server
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

// Requests help for another team/board
bool messagesSendReviveRequest(const char* target_team, const char* target_board)
{
  char msg[128];
  snprintf(msg, sizeof(msg),
           "type=reviveRequest target_team=%s target_board=%s",
           target_team,
           target_board);

  return messagesSendToBoard(MESSAGE_SERVER_BOARD_ID, msg);
}

// Requests the latest map from the server.
bool messagesSendGetMap()
{
  char msg[96];
  snprintf(msg, sizeof(msg),
           "type=getMap board_id=%s",
           MESSAGE_BOARD_ID);

  return messagesSendToBoard(MESSAGE_SERVER_BOARD_ID, msg);
}