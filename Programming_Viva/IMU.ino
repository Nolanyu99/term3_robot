// =====================================================
// Detects/configures IMU
// Used by Main and Turning
// =====================================================

// Writes one register over I2C
bool imu_write_register(TwoWire& bus, uint8_t address, uint8_t reg, uint8_t value)
{
  bus.beginTransmission(address);
  bus.write(reg);
  bus.write(value);
  return bus.endTransmission() == 0;
}

// Reads a block of I2C registers
bool imu_read_registers(TwoWire& bus, uint8_t address, uint8_t start_reg, uint8_t* buffer, uint8_t length)
{
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

// Reads one I2C register
bool imu_read_u8(TwoWire& bus, uint8_t address, uint8_t reg, uint8_t& value)
{
  return imu_read_registers(bus, address, reg, &value, 1);
}

// Checks whether a device responds at an address
bool imu_probe_address(TwoWire& bus, uint8_t address)
{
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

// Prints a byte as two hex digits for debugging
void imu_print_hex_byte(uint8_t value)
{
  if (value < 0x10) {
    Serial.print('0');
  }

  Serial.print(value, HEX);
}

// Scores how many expected IMU devices are visible
uint8_t imu_count_expected_devices(TwoWire& bus)
{
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
    if (imu_probe_address(bus, address)) {
      ++count;
    }
  }

  return count;
}

// Chooses the I2C bus with the most IMU devices
void select_best_imu_i2c_bus()
{
  imu_bus = &Wire;
  imu_bus_name = "Wire D20/D21";

  uint8_t best_score = imu_count_expected_devices(Wire);

  const uint8_t wire1_score = imu_count_expected_devices(Wire1);

  if (wire1_score > best_score) {
    imu_bus = &Wire1;
    imu_bus_name = "Wire1 SDA1/SCL1";
    best_score = wire1_score;
  }

  #if WIRE_HOWMANY > 2
    const uint8_t wire2_score = imu_count_expected_devices(Wire2);

    if (wire2_score > best_score) {
      imu_bus = &Wire2;
      imu_bus_name = "Wire2 D9/D8";
      best_score = wire2_score;
    }
  #endif

  Serial.print("IMU bus=");
  Serial.print(imu_bus_name);
  Serial.print(" expected_devices=");
  Serial.println(best_score);
}

// Configures the accelerometer
bool setup_adxl345()
{
  uint8_t device_id = 0;

  if (!imu_read_u8(*imu_bus, ADXL345_ADDRESS, ADXL345_DEVID, device_id)) {
    Serial.println("ADXL345: not found");
    return false;
  }

  Serial.print("ADXL345 id=0x");
  imu_print_hex_byte(device_id);
  Serial.println(device_id == 0xE5 ? " ok" : " unexpected");

  return imu_write_register(*imu_bus, ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 0x08) &&
         imu_write_register(*imu_bus, ADXL345_ADDRESS, ADXL345_POWER_CTL, 0x08);
}

// Configures the gyro
bool setup_itg320x()
{
  uint8_t who_am_i = 0;

  if (!imu_read_u8(*imu_bus, ITG320X_ADDRESS, ITG320X_WHO_AM_I, who_am_i)) {
    Serial.println("ITG320x gyro: not found");
    return false;
  }

  Serial.print("ITG320x gyro whoami=0x");
  imu_print_hex_byte(who_am_i);
  Serial.println();

  return imu_write_register(*imu_bus, ITG320X_ADDRESS, ITG320X_PWR_MGM, 0x00) &&
         imu_write_register(*imu_bus, ITG320X_ADDRESS, ITG320X_SMPLRT_DIV, 0x07) &&
         imu_write_register(*imu_bus, ITG320X_ADDRESS, ITG320X_DLPF_FS, 0x1E);
}

// Configures HMC/QMC compass
bool setup_hmc5883l()
{
  uint8_t id[3] = {};

  if (imu_read_registers(*imu_bus, HMC5883L_ADDRESS, HMC5883L_ID_A, id, sizeof(id))) {
    compass_is_qmc5883 = false;

    Serial.print("HMC5883L compass id=");
    Serial.write(id[0]);
    Serial.write(id[1]);
    Serial.write(id[2]);
    Serial.println();

    return imu_write_register(*imu_bus, HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x70) &&
           imu_write_register(*imu_bus, HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0x20) &&
           imu_write_register(*imu_bus, HMC5883L_ADDRESS, HMC5883L_MODE, 0x00);
  }

  if (imu_probe_address(*imu_bus, QMC5883L_ADDRESS)) {
    compass_is_qmc5883 = true;

    Serial.println("QMC5883L compass found");

    return imu_write_register(*imu_bus, QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01) &&
           imu_write_register(*imu_bus, QMC5883L_ADDRESS, QMC5883L_CONTROL_1, 0x1D);
  }

  Serial.println("Compass: not found");
  return false;
}

// Configures the pressure sensor (to make sure it doesn't cause issues really)
bool setup_bmp280()
{
  const uint8_t candidates[] = {BMP280_ADDRESS_1, BMP280_ADDRESS_2};

  for (const uint8_t address : candidates) {
    uint8_t chip_id = 0;

    if (imu_read_u8(*imu_bus, address, BMP280_CHIP_ID, chip_id)) {
      bmp280_address = address;

      Serial.print("BMP280 address=0x");
      imu_print_hex_byte(address);
      Serial.print(" id=0x");
      imu_print_hex_byte(chip_id);
      Serial.println(chip_id == 0x58 ? " ok" : " unexpected");

      const bool reset_sent = imu_write_register(*imu_bus, address, BMP280_RESET, 0xB6);
      delay(5);

      return reset_sent &&
             imu_write_register(*imu_bus, address, BMP280_CONFIG, 0xA0) &&
             imu_write_register(*imu_bus, address, BMP280_CTRL_MEAS, 0x27);
    }
  }

  Serial.println("BMP280: not found");
  return false;
}

// Reads raw gyro data and temperature
bool read_itg320x(Axis3& gyro, float& temperature_c)
{
  uint8_t data[8] = {};

  if (!imu_read_registers(*imu_bus, ITG320X_ADDRESS, ITG320X_TEMP_OUT_H, data, sizeof(data))) {
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

// Converts raw Z rate to degrees per second
float gyro_z_to_dps(int16_t raw_z)
{
  return static_cast<float>(raw_z) / ITG320X_LSB_PER_DPS;
}

// Applies bias and deadband
float corrected_gyro_z_dps(int16_t raw_z)
{
  float value = gyro_z_to_dps(raw_z) - gyro_z_bias_dps;

  if (value > -GYRO_Z_DEADBAND_DPS && value < GYRO_Z_DEADBAND_DPS) {
    value = 0.0f;
  }

  return value;
}

// Resets accumulated heading angle
void reset_turn_angle()
{
  turn_angle_deg = 0.0f;
  last_gyro_update_us = micros();
}

// Integrates gyro Z into turn_angle_deg
void update_turn_angle()
{
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

// Measures gyro offset while the robot is still
void calibrate_gyro_z_bias()
{
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
}

// Starts I2C and configures available IMU sensors
void beginIMU()
{
  Wire.begin();
  Wire.setClock(100000);

  Wire1.begin();
  Wire1.setClock(100000);

  #if WIRE_HOWMANY > 2
    Wire2.begin();
    Wire2.setClock(100000);
  #endif

  delay(100);

  select_best_imu_i2c_bus();

  adxl345_ready = setup_adxl345();
  itg320x_ready = setup_itg320x();
  hmc5883l_ready = setup_hmc5883l();
  bmp280_ready = setup_bmp280();

  Serial.println("IMU detected. Gyro bias calibration will happen after IR calibration.");

  Serial.print("IMU ready=");
  Serial.print(adxl345_ready ? 'A' : '-');
  Serial.print(itg320x_ready ? 'G' : '-');
  Serial.print(hmc5883l_ready ? 'M' : '-');
  Serial.print(bmp280_ready ? 'B' : '-');
  Serial.println();
}

// Prints which IMU devices are ready for debug
void printIMUStatus()
{
  Axis3 gyro = {};
  float gyro_temp_c = 0.0f;

  Serial.print(" imu_ready=");
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
}