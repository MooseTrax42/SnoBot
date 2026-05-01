/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Adafruit BNO055 IMU reader.
*/

#ifndef SBCP_IMU_READER_H
#define SBCP_IMU_READER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Allow override of the I2C address at compile time if needed.
#ifndef IMU_I2C_ADDRESS
#define IMU_I2C_ADDRESS BNO055_ADDRESS_A
#endif

#ifndef IMU_I2C_CLOCK_HZ
#define IMU_I2C_CLOCK_HZ 100000
#endif

#ifndef IMU_USE_EXT_CRYSTAL
#define IMU_USE_EXT_CRYSTAL 1
#endif

#ifndef IMU_BOOT_LOG
#define IMU_BOOT_LOG 1
#endif

#ifndef IMU_CHIP_ID
#define IMU_CHIP_ID 0xA0
#endif

#ifndef IMU_CHIP_ID_CHECK_MS
#define IMU_CHIP_ID_CHECK_MS 500
#endif

#ifndef IMU_ZERO_STREAK_WARN
#define IMU_ZERO_STREAK_WARN 10
#endif

#ifndef IMU_ZERO_STREAK_RESET
#define IMU_ZERO_STREAK_RESET 20
#endif

#ifndef IMU_WARMUP_MS
#define IMU_WARMUP_MS 1000  // Wait 1 second after init before trusting data
#endif

/**
* Simple IMU reader wrapper for the Adafruit BNO055.
*/
class ImuReader {
public:
    /**
    * Constructor.
    */
    ImuReader();

    /**
    * Initialize I2C and the BNO055.
    * @return True if the sensor was detected.
    */
    bool begin();

    /**
    * Read orientation (deg), acceleration (m/s^2), and gyro (deg/s).
    * @return True if a valid reading was produced.
    */
    bool read(float& roll, float& pitch, float& yaw,
        float& accelX, float& accelY, float& accelZ,
        float& gyroX, float& gyroY, float& gyroZ);

    /**
    * Read the IMU temperature in degrees C.
    * @return Temperature in degrees C, or 0.0 if unavailable.
    */
    float readTemp();

    /**
    * Read temperature (legacy name for compatibility).
    */
    float readTemperatureC() { return readTemp(); }

    /**
    * Read magnetometer data (uT).
    * @return True if a valid reading was produced.
    */
    bool readMag(float& magX, float& magY, float& magZ);

    /**
    * Read system status and self-test/error flags.
    * @return True if the IMU is available.
    */
    bool readSystemStatus(uint8_t& systemStatus, uint8_t& selfTest, uint8_t& systemError);

    /**
    * Read calibration levels (0-3 each).
    * @return True if the IMU is available.
    */
    bool readCalibration(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag);

    /**
    * Check if the IMU is available.
    */
    bool isAvailable() const { return available; }

    /**
    * Check if the IMU has completed its warm-up period and is ready for use.
    */
    bool isReady() const {
        if (!available || !initialized) return false;
        if (IMU_WARMUP_MS > 0 && initCompleteTimeMs > 0) {
            return (millis() - initCompleteTimeMs) >= IMU_WARMUP_MS;
        }
        return true;
    }

private:
    bool checkChipId(uint8_t* outId = nullptr);
    void logStatus(const char* label);
    void attemptRecovery(const char* reason);

    Adafruit_BNO055 bno;
    bool initialized;
    bool available;
    uint8_t zeroStreak;
    uint32_t lastChipCheckMs;
    uint32_t initCompleteTimeMs;  // Track when initialization completed
    bool readyMessageSent;  // Track if "ready" message has been sent
};

// Global IMU reader instance.
extern ImuReader imuReader;

#endif // SBCP_IMU_READER_H
