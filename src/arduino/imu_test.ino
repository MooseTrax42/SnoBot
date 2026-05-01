#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ===================== USER SETTINGS =====================
static const uint32_t SERIAL_BAUD = 115200;

// BNO055 I2C address:
//  - Most Adafruit boards default to 0x28
//  - Some boards / ADR pin pulled high use 0x29
static const uint8_t BNO_ADDR = 0x28;

// Conservative I2C speed (try 50000 if still flaky)
static const uint32_t I2C_CLOCK_HZ = 100000;

static const uint16_t LOOP_DELAY_MS = 100;

// Stuck-at-zero detection
static const uint8_t ZERO_STREAK_WARN = 10;   // print debug
static const uint8_t ZERO_STREAK_RESET = 20;  // reinit

// Check CHIP_ID (0x00 should read 0xA0). If not, treat as bad I2C and recover.
static const bool ENABLE_CHIP_ID_CHECK = true;
static const uint16_t CHIP_ID_CHECK_EVERY_MS = 500;

// Print status once per second
static const bool PRINT_STATUS_EVERY_SEC = true;
// =========================================================

Adafruit_BNO055 bno(55, BNO_ADDR);

// ------------------ STATE ------------------
static uint8_t zeroStreak = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastChipCheckMs = 0;

// ------------------ LOW-LEVEL I2C ------------------
static uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(reg);
  // repeated start
  if (Wire.endTransmission(false) != 0) return 0xFF;

  if (Wire.requestFrom(BNO_ADDR, (uint8_t)1) != 1) return 0xFF;
  if (!Wire.available()) return 0xFF;
  return Wire.read();
}

static bool chipIdOK(uint8_t *out_id = nullptr) {
  uint8_t id = readReg(0x00);  // CHIP_ID
  if (out_id) *out_id = id;
  return (id == 0xA0);
}

// ------------------ DEBUG ------------------
static void printBNOStatus() {
  uint8_t system_status, self_test, system_error;
  bno.getSystemStatus(&system_status, &self_test, &system_error);

  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.print("BNO: SYS_STATUS=");
  Serial.print(system_status);
  Serial.print(" SELFTEST=0x");
  Serial.print(self_test, HEX);
  Serial.print(" SYS_ERR=");
  Serial.print(system_error);
  Serial.print(" CAL(s,g,a,m)=");
  Serial.print(sys);
  Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print(accel);
  Serial.print(",");
  Serial.println(mag);
}

static bool isAllZeroOrientation(const sensors_event_t &event) {
  // Exact compare on purpose: when stuck/config/bad reads, these are often literal 0.0
  return (event.orientation.x == 0.0f && event.orientation.y == 0.0f && event.orientation.z == 0.0f);
}

// ------------------ INIT / RECOVERY ------------------
static bool initBNO() {
  Serial.println("Initializing BNO055...");

  // Optional quick sanity check: do we even see a valid CHIP_ID on the bus?
  if (ENABLE_CHIP_ID_CHECK) {
    uint8_t id;
    if (!chipIdOK(&id)) {
      Serial.print("WARN: CHIP_ID read not 0xA0 (got 0x");
      Serial.print(id, HEX);
      Serial.println("). I2C may be unstable or wrong address.");
    }
  }

  if (!bno.begin()) {
    Serial.println("ERROR: No BNO055 detected. Check wiring / address (0x28 vs 0x29).");
    return false;
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 init OK.");
  printBNOStatus();
  return true;
}

static void attemptRecovery(const char *reason) {
  Serial.print("RECOVERY: ");
  Serial.println(reason);

  // If I2C is corrupted, reinit may or may not succeed; still worth trying.
  if (!initBNO()) {
    Serial.println("RECOVERY FAILED: bno.begin() failed.");
  }
  zeroStreak = 0;
}

// ------------------ ARDUINO ------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Serial.println();
  Serial.println("BNO055 Orientation + I2C Sanity Check + Auto-Recovery");

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  if (!initBNO()) {
    // Don't hang silently; keep printing.
    while (true) {
      Serial.println("HALT: BNO055 not detected.");
      delay(1000);
    }
  }
}

void loop() {
  const uint32_t now = millis();

  // Periodic CHIP_ID sanity check: catches bad I2C reads early.
  if (ENABLE_CHIP_ID_CHECK && (now - lastChipCheckMs >= CHIP_ID_CHECK_EVERY_MS)) {
    lastChipCheckMs = now;
    uint8_t id;
    if (!chipIdOK(&id)) {
      Serial.print("I2C BAD READ: CHIP_ID != 0xA0 (got 0x");
      Serial.print(id, HEX);
      Serial.println(").");
      attemptRecovery("Bad CHIP_ID (I2C corruption/hang suspected)");
      delay(LOOP_DELAY_MS);
      return;
    }
  }

  // Read orientation
  sensors_event_t event;
  bno.getEvent(&event);

  // Print data
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.print("\tTemp (C): ");
  Serial.println(bno.getTemp());

  // Stuck-at-zero detection
  if (isAllZeroOrientation(event)) {
    if (zeroStreak < 255) zeroStreak++;
  } else {
    zeroStreak = 0;
  }

  // Status output
  if (PRINT_STATUS_EVERY_SEC && (now - lastStatusMs >= 1000)) {
    lastStatusMs = now;
    printBNOStatus();
  }

  // Warn / recover
  if (zeroStreak == ZERO_STREAK_WARN) {
    Serial.println("WARN: Orientation stuck at zeros.");
    printBNOStatus();
  }
  if (zeroStreak >= ZERO_STREAK_RESET) {
    attemptRecovery("Too many consecutive zero orientation samples");
  }

  delay(LOOP_DELAY_MS);
}
