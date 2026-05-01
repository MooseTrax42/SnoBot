/**
 * IMU Debug Test with Temperature Display
 * Tests BNO055 IMU with explicit NDOF mode and shows all sensor data
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);  // Try 0x28 first
bool imuReady = false;
uint32_t initTime = 0;

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n\n=== BNO055 IMU Debug Test with Temperature ===\n");

  // Test 1: I2C Scanner
  Serial.println("Test 1: I2C Scanner");
  Wire.begin();
  Wire.setClock(100000);  // 100kHz
  delay(10);  // Give IMU time to respond

  Serial.println("Scanning I2C bus...");
  int deviceCount = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      deviceCount++;
    }
  }
  if (deviceCount == 0) {
    Serial.println("  ERROR: No I2C devices found!");
    Serial.println("  Check wiring: SDA, SCL, GND, 3.3V");
  } else {
    Serial.print("  Found ");
    Serial.print(deviceCount);
    Serial.println(" device(s)");
  }
  Serial.println();

  // Test 2: Read Chip ID directly
  Serial.println("Test 2: Direct Chip ID Read");
  Wire.beginTransmission(0x28);
  Wire.write(0x00);  // Chip ID register
  if (Wire.endTransmission(false) == 0) {
    if (Wire.requestFrom(0x28, 1) == 1) {
      byte chipId = Wire.read();
      Serial.print("  Chip ID at 0x28: 0x");
      if (chipId < 16) Serial.print("0");
      Serial.print(chipId, HEX);
      if (chipId == 0xA0) {
        Serial.println(" ✓ CORRECT - BNO055 detected!");
      } else {
        Serial.println(" ✗ WRONG - should be 0xA0");
      }
    }
  } else {
    Serial.println("  No response at 0x28, trying 0x29...");
    Wire.beginTransmission(0x29);
    Wire.write(0x00);
    if (Wire.endTransmission(false) == 0) {
      if (Wire.requestFrom(0x29, 1) == 1) {
        byte chipId = Wire.read();
        Serial.print("  Chip ID at 0x29: 0x");
        if (chipId < 16) Serial.print("0");
        Serial.print(chipId, HEX);
        if (chipId == 0xA0) {
          Serial.println(" ✓ CORRECT - BNO055 on alternate address!");
          Serial.println("  ** Change IMU_I2C_ADDRESS to BNO055_ADDRESS_B (0x29) **");
        } else {
          Serial.println(" ✗ WRONG");
        }
      }
    }
  }
  Serial.println();

  // Test 3: Initialize with NDOF mode
  Serial.println("Test 3: Initialize BNO055 with NDOF Mode");
  if (bno.begin()) {
    Serial.println("  ✓ BNO055 initialized!");
    delay(50);

    // Set external crystal and wait for it to stabilize
    bno.setExtCrystalUse(true);
    delay(10);  // Wait for crystal to stabilize (min 7ms per datasheet)
    Serial.println("  ✓ External crystal enabled");

    // Explicitly set NDOF mode for full sensor fusion
    // NDOF requires all 9 sensors to initialize - takes time
    bno.setMode(OPERATION_MODE_NDOF);
    Serial.println("  ⏳ Setting NDOF mode (9-axis fusion)...");
    delay(700);  // Wait for NDOF mode initialization (600-700ms per datasheet)
    Serial.println("  ✓ NDOF mode ready");

    imuReady = true;
    initTime = millis();

    // Read and display initial status
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("  Initial Calibration: sys=");
    Serial.print(system);
    Serial.print(" gyro=");
    Serial.print(gyro);
    Serial.print(" accel=");
    Serial.print(accel);
    Serial.print(" mag=");
    Serial.println(mag);

    uint8_t sysStatus, selfTest, sysError;
    bno.getSystemStatus(&sysStatus, &selfTest, &sysError);
    Serial.print("  System Status: ");
    Serial.print(sysStatus);
    Serial.print(" (0=idle, 1=system error, 2=init peripherals, 3=system init, 4=executing, 5=running, 6=running without fusion)");
    Serial.println();
    Serial.print("  Self Test: 0x");
    if (selfTest < 16) Serial.print("0");
    Serial.print(selfTest, HEX);
    Serial.print("  Error: ");
    Serial.println(sysError);

    // Read temperature
    int8_t temp = bno.getTemp();
    Serial.print("  Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");

  } else {
    Serial.println("  ✗ FAILED: Could not initialize BNO055");
    Serial.println("  Possible causes:");
    Serial.println("    - Wrong I2C address (check ADR pin)");
    Serial.println("    - Wiring issue (SDA/SCL/GND/VCC)");
    Serial.println("    - Bad sensor");
    Serial.println("    - Insufficient power");
  }
  Serial.println();

  if (imuReady) {
    Serial.println("=== Live Data (will update every 500ms) ===");
    Serial.println("Move the IMU to help calibration:");
    Serial.println("  - Keep still for 3s (gyro)");
    Serial.println("  - Tilt in 6 orientations (accel)");
    Serial.println("  - Move in figure-8 (magnetometer)");
    Serial.println();
    Serial.println("Columns: Euler(roll,pitch,yaw) | Accel(x,y,z) | Gyro(x,y,z) | Temp | Cal(S,G,A,M)");
    Serial.println("------------------------------------------------------------------------------");
  } else {
    Serial.println("=== Test Failed - IMU not responding ===");
  }
}

void loop() {
  if (!imuReady) {
    delay(1000);
    return;
  }

  // Check how long since init
  uint32_t elapsed = millis() - initTime;
  bool warmupComplete = elapsed >= 1000;

  // Read sensor data with small delays to avoid I2C bus stress
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  delay(10);  // Give I2C bus time to settle
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  delay(10);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  delay(10);
  int8_t temp = bno.getTemp();
  delay(10);

  // Read calibration status
  uint8_t system, gyroC, accelC, mag;
  bno.getCalibration(&system, &gyroC, &accelC, &mag);

  // Display data
  Serial.print(warmupComplete ? " " : "W");  // W = warming up

  // Euler angles (orientation)
  Serial.print(" R:");
  Serial.print(euler.y(), 1);  // Roll
  Serial.print(" P:");
  Serial.print(euler.z(), 1);  // Pitch
  Serial.print(" Y:");
  Serial.print(euler.x(), 1);  // Yaw (heading)

  // Accelerometer
  Serial.print(" | A:");
  Serial.print(accel.x(), 2);
  Serial.print(",");
  Serial.print(accel.y(), 2);
  Serial.print(",");
  Serial.print(accel.z(), 2);

  // Gyroscope
  Serial.print(" | G:");
  Serial.print(gyro.x(), 2);
  Serial.print(",");
  Serial.print(gyro.y(), 2);
  Serial.print(",");
  Serial.print(gyro.z(), 2);

  // Temperature
  Serial.print(" | T:");
  Serial.print(temp);
  Serial.print("°C");

  // Calibration status
  Serial.print(" | Cal:");
  Serial.print(system);
  Serial.print(",");
  Serial.print(gyroC);
  Serial.print(",");
  Serial.print(accelC);
  Serial.print(",");
  Serial.print(mag);

  // Status indicators
  if (!warmupComplete) {
    Serial.print(" [WARMING UP]");
  } else if (system == 3 && gyroC == 3 && accelC == 3 && mag == 3) {
    Serial.print(" [FULLY CALIBRATED ✓]");
  } else if (system >= 2) {
    Serial.print(" [USABLE]");
  } else if (system >= 1) {
    Serial.print(" [CALIBRATING...]");
  } else {
    Serial.print(" [NOT CALIBRATED - MOVE IMU!]");
  }

  Serial.println();
  delay(500);
}
