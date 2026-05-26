//fixing id length by string length
String readRFID() {
    if (!Serial1.available()) return "";

    String raw = "";
    while (Serial1.available()) {
      char c = Serial1.read();
      if ((c >= '0' && c <= '9') ||
          (c >= 'A' && c <= 'F') ||
          (c >= 'a' && c <= 'f')) {
        raw += c;
      }
      delay(2);
    }
    raw.toUpperCase();
    raw.trim();
    if (raw.length() == 0) return "";

    if (raw.length() == VALID_TAG_LENGTH) {
      return raw;
    } else if (raw.length() > VALID_TAG_LENGTH && raw.length() % VALID_TAG_LENGTH == 0) {
      Serial.println("[RFID] Multiple reads, using first tag.");
      return raw.substring(0, VALID_TAG_LENGTH);
    } else {
      Serial.print("[RFID] Unexpected length (");
      Serial.print(raw.length());
      Serial.println("), discarding.");
      return "";
    }
  }




unsigned long lastRFIDRead = 0;
const unsigned long RFID_COOLDOWN = 1500;

if (millis() - lastRFIDRead < RFID_COOLDOWN) {
    // still in cooldown, do nothing
} else {
    String tag = readRFID();
    if (tag.length() > 0) {
        lastRFIDRead = millis();  // start cooldown
        // do something with tag
    }
}

