// =====================================================
// RFIDServerTest.ino
// Canonical scan-and-query helper plus a '9' serial diagnostic.
//
// scanRFIDAndQueryServer() is the single shared implementation
// used by both:
//   - runRFIDServerTest()        (manual '9' command)
//   - arenaUpdatePoseFromRFID()  (autonomous mission code)
//
// Both call sites get the same scan-then-isFertile-then-wait
// behavior, the same parser, and the same logging shape.
// =====================================================

#include <Arduino.h>

constexpr unsigned long RFID_TEST_SCAN_TIMEOUT_MS = 5000UL;
constexpr unsigned long RFID_TEST_REPLY_TIMEOUT_MS = 3000UL;

// Default short timeouts for autonomous use, where we don't want
// to block the control loop for several seconds per scan.
constexpr unsigned long RFID_AUTO_REPLY_TIMEOUT_MS = 1200UL;

static unsigned long rfid_test_reading_number = 0;

// Convert the current rfid.uid into a hex text string.
void rfidUidToTextLocal(char* out, size_t out_size)
{
  if (out == nullptr || out_size == 0) {
    return;
  }

  out[0] = '\0';

  for (byte i = 0; i < rfid.uid.size; ++i) {
    char part[3];
    snprintf(part, sizeof(part), "%02X", rfid.uid.uidByte[i]);
    strncat(out, part, out_size - strlen(out) - 1);
  }
}

// Try once to read a tag without blocking. Returns true and writes
// the UID hex string if a card was present.
bool tryScanRFIDOnce(char* uid_out, size_t uid_out_size)
{
  if (!(rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial())) {
    return false;
  }

  rfidUidToTextLocal(uid_out, uid_out_size);
  rfid.PICC_HaltA();
  return true;
}

// Block up to scan_timeout_ms waiting for a tag, then on success
// send isFertile and wait up to reply_timeout_ms for the reply.
// Outputs uid + grid x/y + cell state (0..3). Returns true only
// if both the scan and the server reply succeeded.
bool scanRFIDAndQueryServer(unsigned long scan_timeout_ms,
                            unsigned long reply_timeout_ms,
                            char* uid_out, size_t uid_out_size,
                            int8_t* x_out, int8_t* y_out, uint8_t* state_out)
{
  if (uid_out == nullptr || uid_out_size == 0 ||
      x_out == nullptr || y_out == nullptr || state_out == nullptr) {
    return false;
  }

  uid_out[0] = '\0';
  *x_out = -1;
  *y_out = -1;
  *state_out = 0;

  // Phase 1: scan a tag.
  const unsigned long scan_start_ms = millis();
  bool scanned = false;
  while (millis() - scan_start_ms < scan_timeout_ms) {
    if (tryScanRFIDOnce(uid_out, uid_out_size)) {
      scanned = true;
      break;
    }
    messagesLoop();  // keep MQTT alive while we wait
    delay(10);
  }

  if (!scanned) {
    return false;
  }

  // Phase 2: send isFertile and wait for the matching reply.
  if (!messagesSendIsFertile(uid_out)) {
    Serial.print("rfid_query_send_failed uid=");
    Serial.println(uid_out);
    return false;
  }

  const bool got_reply = messagesWaitForFertilityReply(uid_out,
                                                       reply_timeout_ms,
                                                       x_out, y_out, state_out);
  return got_reply;
}

// Manual diagnostic triggered by the '9' serial command.
void runRFIDServerTest()
{
  rfid_test_reading_number++;

  Serial.print("reading_number=");
  Serial.println(rfid_test_reading_number);

  Serial.println("rfid_status=scanning");

  char uid[32];
  int8_t x = -1;
  int8_t y = -1;
  uint8_t state = 0;

  const bool ok = scanRFIDAndQueryServer(RFID_TEST_SCAN_TIMEOUT_MS,
                                         RFID_TEST_REPLY_TIMEOUT_MS,
                                         uid, sizeof(uid),
                                         &x, &y, &state);

  if (uid[0] == '\0') {
    Serial.println("rfid_error=no_chip_found");
    return;
  }

  Serial.print("rfid_uid=");
  Serial.println(uid);

  if (!ok) {
    Serial.println("raw_message=NO_REPLY");
    Serial.println("fertility=-1");
    Serial.println("x=-1 y=-1");
    return;
  }

  // state: 0 unknown, 1 seeded, 2 fertile, 3 infertile.
  const int fertility = (state == 2) ? 1 : (state == 3 ? 0 : -1);

  Serial.print("server_state=");
  Serial.println(state);

  Serial.print("fertility=");
  Serial.println(fertility);

  Serial.print("x=");
  Serial.print(x);
  Serial.print(" y=");
  Serial.println(y);
}
