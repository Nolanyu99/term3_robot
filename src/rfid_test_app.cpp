#include <Arduino.h>
#include <MFRC522_I2C.h>
#include <Wire.h>

namespace {

constexpr uint8_t RFID_ADDR_1 = 0x28;
constexpr uint8_t RFID_ADDR_2 = 0x3C;

MFRC522_I2C rfid_wire_28(RFID_ADDR_1, -1, &Wire);
MFRC522_I2C rfid_wire_3c(RFID_ADDR_2, -1, &Wire);
MFRC522_I2C rfid_wire1_28(RFID_ADDR_1, -1, &Wire1);
MFRC522_I2C rfid_wire1_3c(RFID_ADDR_2, -1, &Wire1);
#if WIRE_HOWMANY > 2
MFRC522_I2C rfid_wire2_28(RFID_ADDR_1, -1, &Wire2);
MFRC522_I2C rfid_wire2_3c(RFID_ADDR_2, -1, &Wire2);
#endif

MFRC522_I2C* active_reader = nullptr;
const char* active_bus_name = "none";
uint8_t active_address = 0;

bool i2c_device_present(TwoWire& bus, uint8_t address) {
    bus.beginTransmission(address);
    return bus.endTransmission() == 0;
}

bool rfid_firmware_version_valid(byte version) {
    return version == 0x15 ||
           version == 0x90 ||
           version == 0x91 ||
           version == 0x92 ||
           version == 0xB2;
}

void print_hex2(byte value) {
    if (value < 0x10) {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

void scan_bus(TwoWire& bus, const char* name) {
    Serial.print("Scanning ");
    Serial.print(name);
    Serial.println("...");

    bool found_any = false;
    for (uint8_t address = 1; address < 127; ++address) {
        if (i2c_device_present(bus, address)) {
            found_any = true;
            Serial.print("  I2C device at 0x");
            print_hex2(address);
            Serial.println();
        }
    }

    if (!found_any) {
        Serial.println("  no I2C devices found");
    }
}

bool try_reader(
    MFRC522_I2C& reader,
    TwoWire& bus,
    const char* bus_name,
    uint8_t address) {
    const bool present = i2c_device_present(bus, address);
    Serial.print("Probe ");
    Serial.print(bus_name);
    Serial.print(" addr=0x");
    print_hex2(address);
    Serial.print(" present=");
    Serial.print(present ? 1 : 0);

    if (!present) {
        Serial.println();
        return false;
    }

    const byte version = reader.PCD_ReadRegister(reader.VersionReg);
    Serial.print(" version=0x");
    print_hex2(version);

    const bool valid = rfid_firmware_version_valid(version);
    Serial.print(" valid=");
    Serial.println(valid ? 1 : 0);

    if (!valid) {
        return false;
    }

    active_reader = &reader;
    active_bus_name = bus_name;
    active_address = address;
    return true;
}

bool choose_reader() {
    if (try_reader(rfid_wire_28, Wire, "Wire D20/D21", RFID_ADDR_1)) {
        return true;
    }
    if (try_reader(rfid_wire_3c, Wire, "Wire D20/D21", RFID_ADDR_2)) {
        return true;
    }
    if (try_reader(rfid_wire1_28, Wire1, "Wire1", RFID_ADDR_1)) {
        return true;
    }
    if (try_reader(rfid_wire1_3c, Wire1, "Wire1", RFID_ADDR_2)) {
        return true;
    }
#if WIRE_HOWMANY > 2
    if (try_reader(rfid_wire2_28, Wire2, "Wire2 D9/D8", RFID_ADDR_1)) {
        return true;
    }
    if (try_reader(rfid_wire2_3c, Wire2, "Wire2 D9/D8", RFID_ADDR_2)) {
        return true;
    }
#endif
    return false;
}

void init_reader_without_soft_reset(MFRC522_I2C& reader) {
    // PCD_Init() can loop forever on this WS1850S unit because its soft reset
    // status bit does not clear like a classic MFRC522. Apply the same runtime
    // register configuration while skipping PCD_Reset().
    reader.PCD_WriteRegister(reader.TModeReg, 0x80);
    reader.PCD_WriteRegister(reader.TPrescalerReg, 0xA9);
    reader.PCD_WriteRegister(reader.TReloadRegH, 0x03);
    reader.PCD_WriteRegister(reader.TReloadRegL, 0xE8);
    reader.PCD_WriteRegister(reader.TxASKReg, 0x40);
    reader.PCD_WriteRegister(reader.ModeReg, 0x3D);
    reader.PCD_AntennaOn();
}

}  // namespace

void rfid_test_app_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
    }

    Serial.println();
    Serial.println(F("=== WS1850S RFID diagnostic test ==="));
    Serial.println(F("GIGA Wire pins: Wire=D20(SDA)/D21(SCL), Wire2=D9(SDA)/D8(SCL)."));

    Wire.begin();
    Wire.setClock(100000);
    Wire1.begin();
    Wire1.setClock(100000);
#if WIRE_HOWMANY > 2
    Wire2.begin();
    Wire2.setClock(100000);
#endif

    scan_bus(Wire, "Wire D20/D21");
    scan_bus(Wire1, "Wire1");
#if WIRE_HOWMANY > 2
    scan_bus(Wire2, "Wire2 D9/D8");
#endif

    if (!choose_reader()) {
        Serial.println(F("No MFRC522-compatible reader found at 0x28 or 0x3C."));
        Serial.println(F("Check VCC, GND, SDA/SCL pins, and whether the module is I2C not UART/SPI."));
        return;
    }

    Serial.print(F("Selected RFID reader on "));
    Serial.print(active_bus_name);
    Serial.print(F(" addr=0x"));
    print_hex2(active_address);
    Serial.println();

    init_reader_without_soft_reset(*active_reader);

    Serial.print(F("Firmware version after no-reset init: 0x"));
    print_hex2(active_reader->PCD_ReadRegister(active_reader->VersionReg));
    Serial.println();
    Serial.println(F("Place a tag near the reader..."));
}

void rfid_test_app_loop() {
    if (active_reader == nullptr) {
        delay(500);
        return;
    }

    if (!active_reader->PICC_IsNewCardPresent()) {
        return;
    }
    if (!active_reader->PICC_ReadCardSerial()) {
        return;
    }

    Serial.print(F("UID: "));
    for (byte i = 0; i < active_reader->uid.size; i++) {
        if (active_reader->uid.uidByte[i] < 0x10) {
            Serial.print('0');
        }
        Serial.print(active_reader->uid.uidByte[i], HEX);
        Serial.print(' ');
    }
    Serial.println();

    active_reader->PICC_HaltA();
    delay(300);
}
