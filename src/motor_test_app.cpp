#include <Arduino.h>
#include <Motoron.h>
#include <Wire.h>

namespace {

constexpr uint8_t MOTORON_ADDRESS = 0x10;
constexpr uint8_t MOTOR_LEFT = 1;
constexpr uint8_t MOTOR_RIGHT = 2;
constexpr uint8_t MOTOR_AUX = 3;
constexpr int16_t TEST_SPEED = 300;
constexpr uint16_t ACCEL = 800;
constexpr uint16_t DECEL = 800;
constexpr uint16_t LOGIC_REFERENCE_MV = 3300;
constexpr uint32_t LOW_VIN_WARNING_MV = 4500;
constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 5000;
constexpr unsigned long STATUS_INTERVAL_MS = 1000;
constexpr unsigned long RETRY_INTERVAL_MS = 3000;

MotoronI2C mc(MOTORON_ADDRESS);
TwoWire* motoron_bus = nullptr;
const char* motoron_bus_name = "not_found";

struct I2cBus {
    TwoWire* bus;
    const char* name;
    const char* pins;
};

I2cBus i2c_buses[] = {
    {&Wire, "Wire", "D20(SDA)/D21(SCL)"},
    {&Wire1, "Wire1", "D102(SDA1)/D101(SCL1)"},
    {&Wire2, "Wire2", "D9(SDA2)/D8(SCL2)"},
};

struct DiagnosticStep {
    const char* name;
    int16_t left_speed;
    int16_t right_speed;
    unsigned long duration_ms;
};

const DiagnosticStep diagnostic_steps[] = {
    {"stop", 0, 0, 1000},
    {"left_motor_only", TEST_SPEED, 0, 2000},
    {"stop", 0, 0, 1000},
    {"right_motor_only", 0, TEST_SPEED, 2000},
    {"stop", 0, 0, 1000},
    {"both_forward", TEST_SPEED, TEST_SPEED, 2000},
    {"stop", 0, 0, 1000},
    {"both_reverse", -TEST_SPEED, -TEST_SPEED, 2000},
    {"stop", 0, 0, 1000},
    {"turn_left", -TEST_SPEED, TEST_SPEED, 2000},
    {"stop", 0, 0, 1000},
};

constexpr size_t STEP_COUNT =
    sizeof(diagnostic_steps) / sizeof(diagnostic_steps[0]);

bool motoron_ready = false;
size_t step_index = 0;
unsigned long step_start_ms = 0;
unsigned long last_status_ms = 0;
unsigned long last_retry_ms = 0;
int16_t requested_left_speed = 0;
int16_t requested_right_speed = 0;

void checkpoint(const char* message) {
    Serial.println(message);
    Serial.flush();
}

void print_hex8(uint8_t value) {
    if (value < 0x10) {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

void print_hex16(uint16_t value) {
    if (value < 0x1000) {
        Serial.print('0');
    }
    if (value < 0x0100) {
        Serial.print('0');
    }
    if (value < 0x0010) {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

bool check_last_error(const char* operation) {
    const uint8_t error = mc.getLastError();
    Serial.print(operation);
    Serial.print(" error=");
    Serial.println(error);
    Serial.flush();

    if (error == 0) {
        return true;
    }

    motoron_ready = false;
    Serial.println("diagnosis=I2C_COMMUNICATION_ERROR");
    return false;
}

bool scan_i2c_bus_for_motoron(const I2cBus& candidate) {
    bool motoron_found_on_bus = false;
    bool any_device_found = false;

    candidate.bus->begin();
    candidate.bus->setClock(100000);

    Serial.print("Scanning ");
    Serial.print(candidate.name);
    Serial.print(" pins=");
    Serial.print(candidate.pins);
    Serial.println("...");

    for (uint8_t address = 1; address < 127; ++address) {
        candidate.bus->beginTransmission(address);
        if (candidate.bus->endTransmission() != 0) {
            continue;
        }

        any_device_found = true;
        Serial.print("i2c_device=0x");
        print_hex8(address);
        if (address == MOTORON_ADDRESS) {
            motoron_found_on_bus = true;
            motoron_bus = candidate.bus;
            motoron_bus_name = candidate.name;
            Serial.print(" expected_motoron");
        }
        Serial.println();
    }

    if (!any_device_found) {
        Serial.println("i2c_scan=no_devices");
    }

    return motoron_found_on_bus;
}

bool scan_all_i2c_buses_for_motoron() {
    motoron_bus = nullptr;
    motoron_bus_name = "not_found";

    for (const I2cBus& candidate : i2c_buses) {
        scan_i2c_bus_for_motoron(candidate);
    }

    if (motoron_bus == nullptr) {
        Serial.println("diagnosis=MOTORON_NOT_FOUND_ON_ANY_I2C_BUS");
        Serial.println("check=SDA_SCL_IOREF_GND_headers_soldering_and_logic_power");
        return false;
    }

    Serial.print("selected_motoron_bus=");
    Serial.println(motoron_bus_name);
    return true;
}

void configure_motor(uint8_t motor) {
    mc.setMaxAcceleration(motor, ACCEL);
    mc.setMaxDeceleration(motor, DECEL);
}

bool initialize_motoron() {
    motoron_ready = false;

    checkpoint("1. Starting GIGA I2C bus scan");

    if (!scan_all_i2c_buses_for_motoron()) {
        return false;
    }

    checkpoint("2. Reinitializing Motoron at 0x10");
    mc.setBus(motoron_bus);
    mc.reinitialize();
    delay(10);
    if (!check_last_error("reinitialize")) {
        return false;
    }

    checkpoint("3. Disabling CRC");
    mc.disableCrc();
    delay(10);
    if (!check_last_error("disable_crc")) {
        return false;
    }

    checkpoint("4. Clearing reset and motor fault flags");
    mc.clearResetFlag();
    mc.clearMotorFaultUnconditional();
    if (!check_last_error("clear_flags")) {
        return false;
    }

    checkpoint("5. Setting command timeout and acceleration limits");
    mc.setCommandTimeoutMilliseconds(2000);
    configure_motor(MOTOR_LEFT);
    configure_motor(MOTOR_RIGHT);
    configure_motor(MOTOR_AUX);
    if (!check_last_error("configure")) {
        return false;
    }

    uint16_t product_id = 0;
    uint16_t firmware_version = 0;
    mc.getFirmwareVersion(&product_id, &firmware_version);
    if (!check_last_error("read_firmware")) {
        return false;
    }

    Serial.print("motoron_product=0x");
    print_hex16(product_id);
    Serial.print(" firmware=0x");
    print_hex16(firmware_version);
    Serial.println();

    motoron_ready = true;
    checkpoint("6. Motoron communication ready");
    return true;
}

void print_status_flags(uint16_t flags) {
    bool printed = false;

    auto print_flag = [&printed](const char* name) {
        if (printed) {
            Serial.print(',');
        }
        Serial.print(name);
        printed = true;
    };

    if (flags & (1 << MOTORON_STATUS_FLAG_PROTOCOL_ERROR)) {
        print_flag("PROTO");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_CRC_ERROR)) {
        print_flag("CRC");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT_LATCHED)) {
        print_flag("TIMEOUT_LATCH");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_MOTOR_FAULT_LATCHED)) {
        print_flag("FAULT_LATCH");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_NO_POWER_LATCHED)) {
        print_flag("NO_POWER_LATCH");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_UART_ERROR)) {
        print_flag("UART");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_RESET)) {
        print_flag("RESET");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT)) {
        print_flag("TIMEOUT");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_MOTOR_FAULTING)) {
        print_flag("FAULT");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_NO_POWER)) {
        print_flag("NO_POWER");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_ERROR_ACTIVE)) {
        print_flag("ERROR");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_MOTOR_OUTPUT_ENABLED)) {
        print_flag("OUTPUT_ENABLED");
    }
    if (flags & (1 << MOTORON_STATUS_FLAG_MOTOR_DRIVING)) {
        print_flag("DRIVING");
    }

    if (!printed) {
        Serial.print("none");
    }
}

void print_diagnosis(uint16_t flags, uint32_t vin_mv) {
    if (flags & (1 << MOTORON_STATUS_FLAG_NO_POWER)) {
        Serial.print(" diagnosis=NO_MOTOR_POWER");
    } else if (vin_mv < LOW_VIN_WARNING_MV) {
        Serial.print(" diagnosis=VIN_LOW_OR_MISSING");
    } else if (flags & (1 << MOTORON_STATUS_FLAG_MOTOR_FAULTING)) {
        Serial.print(" diagnosis=MOTOR_FAULT");
    } else if (flags & (1 << MOTORON_STATUS_FLAG_ERROR_ACTIVE)) {
        Serial.print(" diagnosis=CONTROLLER_ERROR");
    } else {
        Serial.print(" diagnosis=COMMUNICATION_AND_VIN_OK");
    }
}

void set_motor_speeds(int16_t left_speed, int16_t right_speed) {
    requested_left_speed = left_speed;
    requested_right_speed = right_speed;

    if (!motoron_ready) {
        return;
    }

    mc.setSpeedNow(MOTOR_LEFT, left_speed);
    mc.setSpeedNow(MOTOR_RIGHT, right_speed);
    mc.setSpeedNow(MOTOR_AUX, 0);
    if (mc.getLastError() != 0) {
        Serial.print("set_speed error=");
        Serial.println(mc.getLastError());
        Serial.println("diagnosis=I2C_COMMUNICATION_ERROR");
        motoron_ready = false;
    }
}

void apply_step() {
    const DiagnosticStep& step = diagnostic_steps[step_index];
    step_start_ms = millis();
    set_motor_speeds(step.left_speed, step.right_speed);

    Serial.print("step=");
    Serial.print(step_index + 1);
    Serial.print('/');
    Serial.print(STEP_COUNT);
    Serial.print(" name=");
    Serial.print(step.name);
    Serial.print(" requested=");
    Serial.print(step.left_speed);
    Serial.print(',');
    Serial.print(step.right_speed);
    Serial.print(" duration_ms=");
    Serial.println(step.duration_ms);
    Serial.flush();
}

void print_status() {
    if (!motoron_ready) {
        Serial.println("status=motoron_not_ready");
        return;
    }

    const uint16_t flags = mc.getStatusFlags();
    const uint32_t vin_mv =
        mc.getVinVoltageMv(LOGIC_REFERENCE_MV, MotoronVinSenseType::Motoron550);
    const int16_t target_left = mc.getTargetSpeed(MOTOR_LEFT);
    const int16_t target_right = mc.getTargetSpeed(MOTOR_RIGHT);
    const int16_t current_left = mc.getCurrentSpeed(MOTOR_LEFT);
    const int16_t current_right = mc.getCurrentSpeed(MOTOR_RIGHT);

    if (mc.getLastError() != 0) {
        Serial.print("read_status error=");
        Serial.println(mc.getLastError());
        Serial.println("diagnosis=I2C_COMMUNICATION_ERROR");
        motoron_ready = false;
        return;
    }

    Serial.print("status requested=");
    Serial.print(requested_left_speed);
    Serial.print(',');
    Serial.print(requested_right_speed);
    Serial.print(" target=");
    Serial.print(target_left);
    Serial.print(',');
    Serial.print(target_right);
    Serial.print(" current=");
    Serial.print(current_left);
    Serial.print(',');
    Serial.print(current_right);
    Serial.print(" vin=");
    Serial.print(vin_mv);
    Serial.print("mV flags=0x");
    print_hex16(flags);
    Serial.print('[');
    print_status_flags(flags);
    Serial.print(']');
    print_diagnosis(flags, vin_mv);
    Serial.println();
    Serial.flush();
}

}  // namespace

void motor_test_app_setup() {
    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== Motoron M3S550 diagnostic ===");
    Serial.println("Lift the wheels before running this test.");
    Serial.println("USB powers the Arduino but does not replace Motoron VIN motor power.");
    Serial.println("Expected Motoron I2C address: 0x10.");
    Serial.println("Scanning GIGA Wire, Wire1, and Wire2 to identify the connected pins.");
    Serial.println("Sequence: left only, right only, both forward, both reverse, turn left.");

    last_retry_ms = millis();
    if (initialize_motoron()) {
        apply_step();
        print_status();
    }
}

void motor_test_app_loop() {
    const unsigned long now_ms = millis();

    if (!motoron_ready) {
        if (now_ms - last_retry_ms >= RETRY_INTERVAL_MS) {
            last_retry_ms = now_ms;
            Serial.println();
            Serial.println("Retrying Motoron initialization...");
            if (initialize_motoron()) {
                step_index = 0;
                apply_step();
                print_status();
            }
        }
        delay(10);
        return;
    }

    if (now_ms - step_start_ms >= diagnostic_steps[step_index].duration_ms) {
        step_index = (step_index + 1) % STEP_COUNT;
        apply_step();
    }

    if (now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
        last_status_ms = now_ms;
        print_status();
    }

    delay(10);
}
