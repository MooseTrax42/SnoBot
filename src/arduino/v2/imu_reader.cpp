/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Adafruit BNO055 IMU reader implementation.
*/

#include "imu_reader.h"

namespace {
constexpr float kRadToDeg = 180.0f / PI;

uint8_t readReg(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return 0xFF;
    }
    if (Wire.requestFrom(addr, (uint8_t)1) != 1) {
        return 0xFF;
    }
    if (!Wire.available()) {
        return 0xFF;
    }
    return Wire.read();
}

bool allZero(float a, float b, float c, float d, float e, float f, float g, float h, float i) {
    return (a == 0.0f && b == 0.0f && c == 0.0f &&
        d == 0.0f && e == 0.0f && f == 0.0f &&
        g == 0.0f && h == 0.0f && i == 0.0f);
}
}

// Global instance.
ImuReader imuReader;

ImuReader::ImuReader() :
    bno(55, IMU_I2C_ADDRESS),
    initialized(false),
    available(false),
    zeroStreak(0),
    lastChipCheckMs(0),
    initCompleteTimeMs(0),
    readyMessageSent(false)
{
}

bool ImuReader::begin() {
    if (initialized) {
        return available;
    }

    Wire.begin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);
    delay(10);  // Give IMU time to respond after I2C init

    if (!checkChipId()) {
        if (IMU_BOOT_LOG) {
            Serial.println("{\"type\":\"IMU\",\"data\":{\"msg\":\"chip_id_not_detected\"}}");
        }
    }

    available = bno.begin();
    if (available) {
        delay(50);  // I2C settling delay
        if (IMU_USE_EXT_CRYSTAL) {
            bno.setExtCrystalUse(true);
            delay(10);  // Wait for external crystal to stabilize (min 7ms per datasheet)
        }

        // Explicitly set to NDOF mode for full sensor fusion
        // NDOF = Nine Degrees of Freedom (accel + gyro + mag fusion)
        bno.setMode(OPERATION_MODE_NDOF);
        delay(700);  // Wait for NDOF mode initialization (600-700ms per datasheet)

        initCompleteTimeMs = millis();  // Track when init completed for warm-up period
    }

    initialized = true;
    if (IMU_BOOT_LOG) {
        logStatus("boot");
    }
    return available;
}

bool ImuReader::read(float& roll, float& pitch, float& yaw,
    float& accelX, float& accelY, float& accelZ,
    float& gyroX, float& gyroY, float& gyroZ) {
    if (!initialized) {
        begin();
    }

    if (!available) {
        roll = pitch = yaw = 0.0f;
        accelX = accelY = accelZ = 0.0f;
        gyroX = gyroY = gyroZ = 0.0f;
        return false;
    }

    uint32_t now = millis();

    // Wait for warm-up period before trusting IMU data
    if (IMU_WARMUP_MS > 0 && initCompleteTimeMs > 0) {
        uint32_t warmupElapsed = now - initCompleteTimeMs;
        if (warmupElapsed < IMU_WARMUP_MS) {
            // Still in warm-up period, return zeros but don't mark as error
            roll = pitch = yaw = 0.0f;
            accelX = accelY = accelZ = 0.0f;
            gyroX = gyroY = gyroZ = 0.0f;
            return false;  // Not ready yet
        } else if (!readyMessageSent && IMU_BOOT_LOG) {
            // Just became ready, send one-time message
            Serial.println("{\"type\":\"IMU\",\"data\":{\"msg\":\"ready\"}}");
            readyMessageSent = true;
        }
    }
    if (IMU_CHIP_ID_CHECK_MS > 0 && (now - lastChipCheckMs) >= IMU_CHIP_ID_CHECK_MS) {
        lastChipCheckMs = now;
        uint8_t chipId = 0;
        if (!checkChipId(&chipId)) {
            if (IMU_BOOT_LOG) {
                Serial.print("{\"type\":\"IMU\",\"data\":{\"msg\":\"bad_chip_id\",\"chip_id\":\"0x");
                Serial.print(chipId, HEX);
                Serial.println("\"}}");
            }
            attemptRecovery("chip_id");
            roll = pitch = yaw = 0.0f;
            accelX = accelY = accelZ = 0.0f;
            gyroX = gyroY = gyroZ = 0.0f;
            return false;
        }
    }

    // Read sensor data with small delays to avoid I2C bus stress
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    delay(10);  // Give I2C bus time to settle
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    delay(10);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    delay(10);

    // Adafruit BNO055 Euler vector order: heading (x), roll (y), pitch (z).
    yaw = euler.x();
    roll = euler.y();
    pitch = euler.z();

    accelX = accel.x();
    accelY = accel.y();
    accelZ = accel.z();

    // BNO055 gyro vector is radians/sec; convert to degrees/sec.
    gyroX = gyro.x() * kRadToDeg;
    gyroY = gyro.y() * kRadToDeg;
    gyroZ = gyro.z() * kRadToDeg;

    if (allZero(roll, pitch, yaw, accelX, accelY, accelZ, gyroX, gyroY, gyroZ)) {
        if (zeroStreak < 255) {
            zeroStreak++;
        }
        if (zeroStreak == IMU_ZERO_STREAK_WARN && IMU_BOOT_LOG) {
            Serial.println("{\"type\":\"IMU\",\"data\":{\"msg\":\"zero_samples_detected\"}}");
            logStatus("zero_warn");
        }
        if (zeroStreak >= IMU_ZERO_STREAK_RESET) {
            attemptRecovery("zero_streak");
            zeroStreak = 0;
            return false;
        }
    } else {
        zeroStreak = 0;
    }

    return true;
}

float ImuReader::readTemp() {
    if (!initialized) {
        begin();
    }

    if (!available) {
        return 0.0f;
    }

    return static_cast<float>(bno.getTemp());
}

bool ImuReader::readMag(float& magX, float& magY, float& magZ) {
    if (!initialized) {
        begin();
    }

    if (!available) {
        magX = magY = magZ = 0.0f;
        return false;
    }

    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    magX = mag.x();
    magY = mag.y();
    magZ = mag.z();
    return true;
}

bool ImuReader::readSystemStatus(uint8_t& systemStatus, uint8_t& selfTest, uint8_t& systemError) {
    if (!initialized) {
        begin();
    }

    if (!available) {
        systemStatus = 0;
        selfTest = 0;
        systemError = 0;
        return false;
    }

    bno.getSystemStatus(&systemStatus, &selfTest, &systemError);
    return true;
}

bool ImuReader::readCalibration(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag) {
    if (!initialized) {
        begin();
    }

    if (!available) {
        sys = gyro = accel = mag = 0;
        return false;
    }

    bno.getCalibration(&sys, &gyro, &accel, &mag);
    return true;
}

bool ImuReader::checkChipId(uint8_t* outId) {
    uint8_t id = readReg(IMU_I2C_ADDRESS, 0x00);
    if (outId) {
        *outId = id;
    }
    return id == IMU_CHIP_ID;
}

void ImuReader::logStatus(const char* label) {
    if (!available) {
        Serial.print("{\"type\":\"IMU\",\"data\":{\"msg\":\"");
        Serial.print(label);
        Serial.println("\",\"available\":false}}");
        return;
    }

    uint8_t systemStatus = 0;
    uint8_t selfTest = 0;
    uint8_t systemError = 0;
    bno.getSystemStatus(&systemStatus, &selfTest, &systemError);

    uint8_t sys = 0;
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    Serial.print("{\"type\":\"IMU\",\"data\":{\"msg\":\"");
    Serial.print(label);
    Serial.print("\",\"status\":");
    Serial.print(systemStatus);
    Serial.print(",\"selftest\":\"0x");
    Serial.print(selfTest, HEX);
    Serial.print("\",\"error\":");
    Serial.print(systemError);
    Serial.print(",\"cal\":{\"sys\":");
    Serial.print(sys);
    Serial.print(",\"gyro\":");
    Serial.print(gyro);
    Serial.print(",\"accel\":");
    Serial.print(accel);
    Serial.print(",\"mag\":");
    Serial.print(mag);
    Serial.println("}}}");
}

void ImuReader::attemptRecovery(const char* reason) {
    if (IMU_BOOT_LOG) {
        Serial.print("{\"type\":\"IMU\",\"data\":{\"msg\":\"recovery_attempt\",\"reason\":\"");
        Serial.print(reason);
        Serial.println("\"}}");
    }
    available = bno.begin();
    if (available) {
        delay(50);  // I2C settling delay
        if (IMU_USE_EXT_CRYSTAL) {
            bno.setExtCrystalUse(true);
            delay(10);  // Wait for external crystal to stabilize
        }

        // Restore NDOF mode after recovery
        bno.setMode(OPERATION_MODE_NDOF);
        delay(700);  // Wait for NDOF mode initialization (600-700ms per datasheet)

        initCompleteTimeMs = millis();  // Reset warm-up period after recovery
        readyMessageSent = false;  // Reset ready message flag
        if (IMU_BOOT_LOG) {
            logStatus("recovered");
        }
    } else if (IMU_BOOT_LOG) {
        Serial.println("{\"type\":\"IMU\",\"data\":{\"msg\":\"recovery_failed\"}}");
    }
    zeroStreak = 0;
}
