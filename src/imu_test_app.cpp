#include <Arduino.h>
#include <Motoron.h>
#include <Wire.h>
#include <math.h>

namespace {

constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr unsigned long PRINT_INTERVAL_MS = 100;
constexpr unsigned long TURN_TIMEOUT_MS = 5000;
constexpr uint16_t GYRO_BIAS_SAMPLE_COUNT = 200;
constexpr unsigned long GYRO_BIAS_SAMPLE_DELAY_MS = 5;
constexpr float ITG320X_LSB_PER_DPS = 14.375f;
constexpr float GYRO_Z_DEADBAND_DPS = 0.4f;
constexpr float TURN_TARGET_DEG = 90.0f;

constexpr uint8_t MOTOR_LEFT = 1;
constexpr uint8_t MOTOR_RIGHT = 2;
constexpr int16_t FORWARD_SPEED = 300;
constexpr int16_t TURN_SPEED = 300;
constexpr int8_t LEFT_FORWARD_SIGN = 1;
constexpr int8_t RIGHT_FORWARD_SIGN = 1;
constexpr uint8_t RIGHT_MOTOR_TRIM_PERCENT = 95;
constexpr uint16_t MOTOR_ACCEL = 800;
constexpr uint16_t MOTOR_DECEL = 800;

constexpr uint8_t ADXL345_ADDRESS = 0x53;
constexpr uint8_t ADXL345_DEVID = 0x00;
constexpr uint8_t ADXL345_POWER_CTL = 0x2D;
constexpr uint8_t ADXL345_DATA_FORMAT = 0x31;
constexpr uint8_t ADXL345_DATAX0 = 0x32;

constexpr uint8_t ITG320X_ADDRESS = 0x68;
constexpr uint8_t ITG320X_WHO_AM_I = 0x00;
constexpr uint8_t ITG320X_SMPLRT_DIV = 0x15;
constexpr uint8_t ITG320X_DLPF_FS = 0x16;
constexpr uint8_t ITG320X_TEMP_OUT_H = 0x1B;
constexpr uint8_t ITG320X_PWR_MGM = 0x3E;

constexpr uint8_t HMC5883L_ADDRESS = 0x1E;
constexpr uint8_t HMC5883L_CONFIG_A = 0x00;
constexpr uint8_t HMC5883L_CONFIG_B = 0x01;
constexpr uint8_t HMC5883L_MODE = 0x02;
constexpr uint8_t HMC5883L_DATA_X_MSB = 0x03;
constexpr uint8_t HMC5883L_ID_A = 0x0A;

constexpr uint8_t QMC5883L_ADDRESS = 0x0D;
constexpr uint8_t QMC5883L_DATA_X_LSB = 0x00;
constexpr uint8_t QMC5883L_STATUS = 0x06;
constexpr uint8_t QMC5883L_CONTROL_1 = 0x09;
constexpr uint8_t QMC5883L_SET_RESET = 0x0B;

constexpr uint8_t BMP280_ADDRESS_1 = 0x76;
constexpr uint8_t BMP280_ADDRESS_2 = 0x77;
constexpr uint8_t BMP280_CHIP_ID = 0xD0;
constexpr uint8_t BMP280_RESET = 0xE0;
constexpr uint8_t BMP280_CTRL_MEAS = 0xF4;
constexpr uint8_t BMP280_CONFIG = 0xF5;
constexpr uint8_t BMP280_PRESS_MSB = 0xF7;

unsigned long last_print_ms = 0;
MotoronI2C motoron;
TwoWire* imu_bus = &Wire;
const char* imu_bus_name = "Wire D20/D21";
bool motoron_ready = false;
bool adxl345_ready = false;
bool itg320x_ready = false;
bool hmc5883l_ready = false;
bool bmp280_ready = false;
bool compass_is_qmc5883 = false;
uint8_t bmp280_address = BMP280_ADDRESS_1;
float gyro_z_bias_dps = 0.0f;
float turn_angle_deg = 0.0f;
unsigned long last_gyro_update_us = 0;
unsigned long turn_start_ms = 0;
int8_t turn_direction = 0;

enum class DriveMode {
    Idle,
    Forward,
    TurnInPlace,
};

DriveMode drive_mode = DriveMode::Idle;

struct Axis3 {
    int16_t x;
    int16_t y;
    int16_t z;
};

constexpr int16_t trim_right_speed(int16_t speed) {
    return static_cast<int16_t>(
        (static_cast<int32_t>(speed) * RIGHT_MOTOR_TRIM_PERCENT) / 100);
}

const char* drive_mode_name() {
    switch (drive_mode) {
        case DriveMode::Idle:
            return "idle";
        case DriveMode::Forward:
            return "forward";
        case DriveMode::TurnInPlace:
            return "turn";
        default:
            return "unknown";
    }
}

bool write_register(TwoWire& bus, uint8_t address, uint8_t reg, uint8_t value) {
    bus.beginTransmission(address);
    bus.write(reg);
    bus.write(value);
    return bus.endTransmission() == 0;
}

bool read_registers(
    TwoWire& bus,
    uint8_t address,
    uint8_t start_reg,
    uint8_t* buffer,
    uint8_t length) {
    bus.beginTransmission(address);
    bus.write(start_reg);
    if (bus.endTransmission(false) != 0) {
        return false;
    }

    const uint8_t received = bus.requestFrom(address, length);
    if (received != length) {
        while (bus.available() > 0) {
            bus.read();
        }
        return false;
    }

    for (uint8_t i = 0; i < length; ++i) {
        buffer[i] = static_cast<uint8_t>(bus.read());
    }
    return true;
}

bool read_u8(TwoWire& bus, uint8_t address, uint8_t reg, uint8_t& value) {
    return read_registers(bus, address, reg, &value, 1);
}

bool probe_address(TwoWire& bus, uint8_t address) {
    bus.beginTransmission(address);
    return bus.endTransmission() == 0;
}

void print_hex_byte(uint8_t value) {
    if (value < 0x10) {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

uint8_t scan_i2c_bus(TwoWire& bus, const char* name) {
    Serial.print("I2C scan ");
    Serial.print(name);
    Serial.println(':');
    uint8_t found_count = 0;
    for (uint8_t address = 1; address < 127; ++address) {
        if (probe_address(bus, address)) {
            Serial.print("  found 0x");
            print_hex_byte(address);
            Serial.println();
            ++found_count;
        }
    }

    if (found_count == 0) {
        Serial.println("  no I2C devices found");
    }
    return found_count;
}

uint8_t count_expected_devices(TwoWire& bus) {
    uint8_t count = 0;
    const uint8_t expected_addresses[] = {
        HMC5883L_ADDRESS,
        QMC5883L_ADDRESS,
        ADXL345_ADDRESS,
        ITG320X_ADDRESS,
        BMP280_ADDRESS_1,
        BMP280_ADDRESS_2,
    };

    for (const uint8_t address : expected_addresses) {
        if (probe_address(bus, address)) {
            ++count;
        }
    }
    return count;
}

void select_best_i2c_bus() {
    imu_bus = &Wire;
    imu_bus_name = "Wire D20/D21";
    uint8_t best_score = count_expected_devices(Wire);

    const uint8_t wire1_score = count_expected_devices(Wire1);
    if (wire1_score > best_score) {
        imu_bus = &Wire1;
        imu_bus_name = "Wire1 SDA1/SCL1";
        best_score = wire1_score;
    }

#if WIRE_HOWMANY > 2
    const uint8_t wire2_score = count_expected_devices(Wire2);
    if (wire2_score > best_score) {
        imu_bus = &Wire2;
        imu_bus_name = "Wire2 D9(SDA)/D8(SCL)";
        best_score = wire2_score;
    }
#endif

    Serial.print("Selected bus: ");
    Serial.print(imu_bus_name);
    Serial.print(" expected_devices=");
    Serial.println(best_score);
}

bool setup_adxl345() {
    uint8_t device_id = 0;
    if (!read_u8(*imu_bus, ADXL345_ADDRESS, ADXL345_DEVID, device_id)) {
        Serial.println("ADXL345: not found at 0x53");
        return false;
    }

    Serial.print("ADXL345: id=0x");
    print_hex_byte(device_id);
    Serial.println(device_id == 0xE5 ? " ok" : " unexpected");

    const bool configured =
        write_register(*imu_bus, ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 0x08) &&
        write_register(*imu_bus, ADXL345_ADDRESS, ADXL345_POWER_CTL, 0x08);
    if (!configured) {
        Serial.println("ADXL345: configuration failed");
    }
    return configured;
}

bool setup_itg320x() {
    uint8_t who_am_i = 0;
    if (!read_u8(*imu_bus, ITG320X_ADDRESS, ITG320X_WHO_AM_I, who_am_i)) {
        Serial.println("ITG320x gyro: not found at 0x68");
        return false;
    }

    Serial.print("ITG320x gyro: whoami=0x");
    print_hex_byte(who_am_i);
    Serial.println();

    const bool configured =
        write_register(*imu_bus, ITG320X_ADDRESS, ITG320X_PWR_MGM, 0x00) &&
        write_register(*imu_bus, ITG320X_ADDRESS, ITG320X_SMPLRT_DIV, 0x07) &&
        write_register(*imu_bus, ITG320X_ADDRESS, ITG320X_DLPF_FS, 0x1E);
    if (!configured) {
        Serial.println("ITG320x gyro: configuration failed");
    }
    return configured;
}

bool setup_hmc5883l() {
    uint8_t id[3] = {};
    if (read_registers(*imu_bus, HMC5883L_ADDRESS, HMC5883L_ID_A, id, sizeof(id))) {
        compass_is_qmc5883 = false;

        Serial.print("HMC5883L compass: id=");
        Serial.write(id[0]);
        Serial.write(id[1]);
        Serial.write(id[2]);
        Serial.println();

        const bool configured =
            write_register(*imu_bus, HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x70) &&
            write_register(*imu_bus, HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0x20) &&
            write_register(*imu_bus, HMC5883L_ADDRESS, HMC5883L_MODE, 0x00);
        if (!configured) {
            Serial.println("HMC5883L compass: configuration failed");
        }
        return configured;
    }

    if (probe_address(*imu_bus, QMC5883L_ADDRESS)) {
        compass_is_qmc5883 = true;
        Serial.println("QMC/VCM5883L compass: found at 0x0D");

        const bool configured =
            write_register(*imu_bus, QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01) &&
            write_register(*imu_bus, QMC5883L_ADDRESS, QMC5883L_CONTROL_1, 0x1D);
        if (!configured) {
            Serial.println("QMC/VCM5883L compass: configuration failed");
        }
        return configured;
    }

    Serial.println("Compass: not found at 0x1E or 0x0D");
    return false;
}

bool setup_bmp280() {
    const uint8_t candidates[] = {BMP280_ADDRESS_1, BMP280_ADDRESS_2};
    uint8_t chip_id = 0;

    for (const uint8_t address : candidates) {
        if (read_u8(*imu_bus, address, BMP280_CHIP_ID, chip_id)) {
            bmp280_address = address;
            Serial.print("BMP280: address=0x");
            print_hex_byte(address);
            Serial.print(" id=0x");
            print_hex_byte(chip_id);
            Serial.println(chip_id == 0x58 ? " ok" : " unexpected");

            const bool reset_sent = write_register(*imu_bus, address, BMP280_RESET, 0xB6);
            delay(5);
            const bool measurement_configured =
                reset_sent &&
                write_register(*imu_bus, address, BMP280_CONFIG, 0xA0) &&
                write_register(*imu_bus, address, BMP280_CTRL_MEAS, 0x27);
            if (!measurement_configured) {
                Serial.println("BMP280: configuration failed");
            }
            return measurement_configured;
        }
    }

    Serial.println("BMP280: not found at 0x76 or 0x77");
    return false;
}

bool read_itg320x(Axis3& gyro, float& temperature_c) {
    uint8_t data[8] = {};
    if (!read_registers(*imu_bus, ITG320X_ADDRESS, ITG320X_TEMP_OUT_H, data, sizeof(data))) {
        return false;
    }

    const int16_t raw_temp =
        static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    gyro.x = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
    gyro.y = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);
    gyro.z = static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) | data[7]);
    temperature_c = 35.0f + (static_cast<float>(raw_temp) + 13200.0f) / 280.0f;
    return true;
}

float gyro_z_to_dps(int16_t raw_z) {
    return static_cast<float>(raw_z) / ITG320X_LSB_PER_DPS;
}

float corrected_gyro_z_dps(int16_t raw_z) {
    float value = gyro_z_to_dps(raw_z) - gyro_z_bias_dps;
    if (value > -GYRO_Z_DEADBAND_DPS && value < GYRO_Z_DEADBAND_DPS) {
        value = 0.0f;
    }
    return value;
}

void reset_turn_angle() {
    turn_angle_deg = 0.0f;
    last_gyro_update_us = micros();
}

void checkpoint(const char* message) {
    Serial.println(message);
    Serial.flush();
}

void configure_motor(uint8_t motor) {
    motoron.setMaxAcceleration(motor, MOTOR_ACCEL);
    motoron.setMaxDeceleration(motor, MOTOR_DECEL);
}

void set_motor_speeds(int16_t left_speed, int16_t right_speed) {
    if (!motoron_ready) {
        return;
    }

    motoron.setSpeed(MOTOR_LEFT, left_speed);
    motoron.setSpeed(MOTOR_RIGHT, right_speed);
    if (motoron.getLastError() != 0) {
        Serial.print("motoron_error=");
        Serial.println(motoron.getLastError());
        motoron_ready = false;
    }
}

void stop_motors() {
    set_motor_speeds(0, 0);
    drive_mode = DriveMode::Idle;
    turn_direction = 0;
}

void begin_motoron() {
    checkpoint("Motoron: reinitialize");
    motoron.setBus(&Wire1);
    motoron.reinitialize();
    delay(10);

    checkpoint("Motoron: disable CRC");
    motoron.disableCrc();
    delay(10);

    checkpoint("Motoron: clear reset/fault");
    motoron.clearResetFlag();
    motoron.clearMotorFaultUnconditional();
    motoron.setCommandTimeoutMilliseconds(2000);
    configure_motor(MOTOR_LEFT);
    configure_motor(MOTOR_RIGHT);

    motoron_ready = motoron.getLastError() == 0;
    Serial.print("motoron_ready=");
    Serial.print(motoron_ready ? 1 : 0);
    Serial.print(" error=");
    Serial.println(motoron.getLastError());
    stop_motors();
}

void calibrate_gyro_z_bias() {
    if (!itg320x_ready) {
        return;
    }

    Serial.println("Keep IMU still: calibrating gyro Z bias...");
    float sum_dps = 0.0f;
    uint16_t sample_count = 0;
    for (uint16_t i = 0; i < GYRO_BIAS_SAMPLE_COUNT; ++i) {
        Axis3 gyro = {};
        float temperature_c = 0.0f;
        if (read_itg320x(gyro, temperature_c)) {
            sum_dps += gyro_z_to_dps(gyro.z);
            ++sample_count;
        }
        delay(GYRO_BIAS_SAMPLE_DELAY_MS);
    }

    if (sample_count > 0) {
        gyro_z_bias_dps = sum_dps / static_cast<float>(sample_count);
    }

    reset_turn_angle();
    Serial.print("gyro_z_bias_dps=");
    Serial.println(gyro_z_bias_dps, 3);
    Serial.println("Send r=reset turn angle, c=recalibrate gyro bias.");
}

void update_turn_angle() {
    if (!itg320x_ready) {
        return;
    }

    const unsigned long now_us = micros();
    if (last_gyro_update_us == 0) {
        last_gyro_update_us = now_us;
        return;
    }

    Axis3 gyro = {};
    float temperature_c = 0.0f;
    if (!read_itg320x(gyro, temperature_c)) {
        last_gyro_update_us = now_us;
        return;
    }

    const float dt_s =
        static_cast<float>(now_us - last_gyro_update_us) / 1000000.0f;
    last_gyro_update_us = now_us;
    turn_angle_deg += corrected_gyro_z_dps(gyro.z) * dt_s;
}

void start_forward() {
    drive_mode = DriveMode::Forward;
    turn_direction = 0;
    Serial.println("drive=forward");
}

void start_turn(int8_t direction) {
    if (!motoron_ready) {
        Serial.println("turn_error=motoron_not_ready");
        return;
    }
    if (!itg320x_ready) {
        Serial.println("turn_error=gyro_not_ready");
        return;
    }

    turn_direction = direction >= 0 ? 1 : -1;
    turn_start_ms = millis();
    reset_turn_angle();
    drive_mode = DriveMode::TurnInPlace;

    Serial.print("turn_start=");
    Serial.print(turn_direction > 0 ? "left" : "right");
    Serial.print(" target_deg=");
    Serial.println(TURN_TARGET_DEG, 1);
}

void update_drive_control() {
    if (drive_mode == DriveMode::Idle) {
        set_motor_speeds(0, 0);
        return;
    }

    if (drive_mode == DriveMode::Forward) {
        set_motor_speeds(LEFT_FORWARD_SIGN * FORWARD_SPEED,
                         trim_right_speed(RIGHT_FORWARD_SIGN * FORWARD_SPEED));
        return;
    }

    if (drive_mode != DriveMode::TurnInPlace) {
        return;
    }

    if (fabsf(turn_angle_deg) >= TURN_TARGET_DEG) {
        Serial.print("turn_done angle=");
        Serial.println(turn_angle_deg, 1);
        stop_motors();
        return;
    }

    if (millis() - turn_start_ms >= TURN_TIMEOUT_MS) {
        Serial.print("turn_timeout angle=");
        Serial.println(turn_angle_deg, 1);
        stop_motors();
        return;
    }

    if (turn_direction > 0) {
        set_motor_speeds(-LEFT_FORWARD_SIGN * TURN_SPEED,
                         trim_right_speed(RIGHT_FORWARD_SIGN * TURN_SPEED));
    } else {
        set_motor_speeds(LEFT_FORWARD_SIGN * TURN_SPEED,
                         trim_right_speed(-RIGHT_FORWARD_SIGN * TURN_SPEED));
    }
}

void process_serial_commands() {
    while (Serial.available() > 0) {
        const char command = static_cast<char>(Serial.read());
        if (command == 'f' || command == 'F') {
            start_forward();
        } else if (command == 'x' || command == 'X') {
            stop_motors();
            Serial.println("drive=stop");
        } else if (command == 'l' || command == 'L') {
            start_turn(1);
        } else if (command == 'r' || command == 'R') {
            start_turn(-1);
        } else if (command == 'z' || command == 'Z') {
            reset_turn_angle();
            Serial.println("turn_angle_reset=1");
        } else if (command == 'c' || command == 'C') {
            stop_motors();
            calibrate_gyro_z_bias();
        }
    }
}

void print_status() {
    Axis3 gyro = {};
    float gyro_temp_c = 0.0f;

    Serial.print("mode=");
    Serial.print(drive_mode_name());
    Serial.print(" motoron=");
    Serial.print(motoron_ready ? 1 : 0);
    Serial.print(' ');
    Serial.print("ready=");
    Serial.print(adxl345_ready ? 'A' : '-');
    Serial.print(itg320x_ready ? 'G' : '-');
    Serial.print(hmc5883l_ready ? 'M' : '-');
    Serial.print(bmp280_ready ? 'B' : '-');

    if (itg320x_ready && read_itg320x(gyro, gyro_temp_c)) {
        Serial.print(" gyro_z=");
        Serial.print(gyro_z_to_dps(gyro.z), 2);
        Serial.print(" gyro_z_corr=");
        Serial.print(corrected_gyro_z_dps(gyro.z), 2);
        Serial.print(" turn_deg=");
        Serial.print(turn_angle_deg, 1);
    }

    Serial.println();
}

}  // namespace

void imu_test_app_setup() {
    Serial.begin(115200);
    const unsigned long serial_wait_start_ms = millis();
    while (!Serial && millis() - serial_wait_start_ms < SERIAL_WAIT_TIMEOUT_MS) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== DFRobot SEN0140 10DOF IMU I2C test ===");
    Serial.println("Try GIGA Wire=D20(SDA)/D21(SCL), Wire1=SDA1/SCL1, or Wire2=D9(SDA)/D8(SCL).");

    Wire.begin();
    Wire.setClock(100000);
    Wire1.begin();
    Wire1.setClock(100000);
#if WIRE_HOWMANY > 2
    Wire2.begin();
    Wire2.setClock(100000);
#endif
    delay(100);

    begin_motoron();

    scan_i2c_bus(Wire, "Wire D20/D21");
    scan_i2c_bus(Wire1, "Wire1 SDA1/SCL1");
#if WIRE_HOWMANY > 2
    scan_i2c_bus(Wire2, "Wire2 D9(SDA)/D8(SCL)");
#endif

    select_best_i2c_bus();
    adxl345_ready = setup_adxl345();
    itg320x_ready = setup_itg320x();
    hmc5883l_ready = setup_hmc5883l();
    bmp280_ready = setup_bmp280();
    calibrate_gyro_z_bias();

    Serial.println("Ready flags: A=accelerometer G=gyro M=magnetometer B=barometer.");
    Serial.println("Commands: f=forward, x=stop, l=left 90, r=right 90, c=calibrate gyro, z=reset angle.");
    if (!adxl345_ready && !itg320x_ready && !hmc5883l_ready && !bmp280_ready) {
        Serial.println("No SEN0140 devices found. Check VCC, GND, SDA/SCL order, and common ground.");
    }
}

void imu_test_app_loop() {
    process_serial_commands();
    update_turn_angle();
    update_drive_control();

    const unsigned long now_ms = millis();
    if (now_ms - last_print_ms >= PRINT_INTERVAL_MS) {
        last_print_ms = now_ms;
        print_status();
    }
}
