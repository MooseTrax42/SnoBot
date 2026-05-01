#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "controller_protocol.h"
#include "controller_ui.h"

#ifndef CONTROLLER_ANALOG_MAX
#define CONTROLLER_ANALOG_MAX 4095
#endif

namespace {
constexpr uint32_t kVelocityPeriodMs = 100;  // 10 Hz
constexpr uint32_t kChutePeriodMs = 180;

constexpr float kMaxLinearMps = 1.5f;
constexpr float kMaxAngularRadPs = 3.14f;
constexpr float kStickDeadzone = 0.08f;
constexpr float kChuteStepDeg = 2.0f;
}

ControllerProtocol gProtocol;
ControllerUi gUi;

bool gDriveRequested = false;
bool gSentMotion = false;
bool gSaltOn = false;
bool gAugerOn = false;
bool gRecording = false;
bool gHasPath = false;

float gLastChuteSent = 1000.0f;
uint32_t gLastVelMs = 0;
uint32_t gLastChuteMs = 0;

int gPrevEmergency = LOW;
int gPrevStart = LOW;
int gPrevPower = LOW;
int gPrevSalt = LOW;
int gPrevAuger = LOW;
int gPrevPath = LOW;
int gPrevClear = LOW;

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float normalizeAxis(int raw) {
  const float half = CONTROLLER_ANALOG_MAX * 0.5f;
  float value = ((float)raw - half) / half;
  value = clampf(value, -1.0f, 1.0f);
  if (fabsf(value) < kStickDeadzone) {
    return 0.0f;
  }
  return value;
}

float analogToChuteAngle(int raw) {
  float normalized = (float)raw / (float)CONTROLLER_ANALOG_MAX;
  normalized = clampf(normalized, 0.0f, 1.0f);
  return -90.0f + (normalized * 180.0f);
}

void updateStatusLeds() {
  digitalWrite(SaltLEDPin, gSaltOn ? HIGH : LOW);
  digitalWrite(AugerLEDPin, gAugerOn ? HIGH : LOW);
  digitalWrite(PathLEDPin, gRecording ? HIGH : LOW);
}

void setupPins() {
  pinMode(EMERPin, INPUT);
  pinMode(SaltPini, INPUT);
  pinMode(AugerPini, INPUT);
  pinMode(StartPini, INPUT);
  pinMode(PowerOn, INPUT);
  pinMode(PathPin, INPUT);
  pinMode(ClearPin, INPUT);

  pinMode(SaltLEDPin, OUTPUT);
  pinMode(AugerLEDPin, OUTPUT);
  pinMode(PathLEDPin, OUTPUT);

  gPrevEmergency = digitalRead(EMERPin);
  gPrevSalt = digitalRead(SaltPini);
  gPrevAuger = digitalRead(AugerPini);
  gPrevStart = digitalRead(StartPini);
  gPrevPower = digitalRead(PowerOn);
  gPrevPath = digitalRead(PathPin);
  gPrevClear = digitalRead(ClearPin);
}

void syncFromTelemetry() {
  if (!gProtocol.hasTelemetry()) return;
  const TelemetryState& t = gProtocol.telemetry();
  gSaltOn = t.saltOn;
  gAugerOn = t.augerOn;
  gRecording = strcmp(t.recState, "recording") == 0;
  if (strcmp(t.recState, "done") == 0) {
    gHasPath = true;
  }
}

void onConnectionChange(bool connected) {
  if (connected) {
    gProtocol.clearError();
    gProtocol.sendSimpleCmd("ping");
    return;
  }
  gDriveRequested = false;
  gSentMotion = false;
}

void handleButtonsAndModes() {
  const int emergencyNow = digitalRead(EMERPin);
  const int startNow = digitalRead(StartPini);
  const int powerNow = digitalRead(PowerOn);
  const int saltNow = digitalRead(SaltPini);
  const int augerNow = digitalRead(AugerPini);
  const int pathNow = digitalRead(PathPin);
  const int clearNow = digitalRead(ClearPin);

  const bool emergencyRise = (emergencyNow == HIGH && gPrevEmergency == LOW);
  const bool startRise = (startNow == HIGH && gPrevStart == LOW);
  const bool powerFall = (powerNow == LOW && gPrevPower == HIGH);
  const bool saltRise = (saltNow == HIGH && gPrevSalt == LOW);
  const bool augerRise = (augerNow == HIGH && gPrevAuger == LOW);
  const bool pathRise = (pathNow == HIGH && gPrevPath == LOW);
  const bool clearRise = (clearNow == HIGH && gPrevClear == LOW);

  gPrevEmergency = emergencyNow;
  gPrevStart = startNow;
  gPrevPower = powerNow;
  gPrevSalt = saltNow;
  gPrevAuger = augerNow;
  gPrevPath = pathNow;
  gPrevClear = clearNow;

  if (emergencyRise) {
    if (gProtocol.sendSimpleCmd("estop")) {
      gDriveRequested = false;
      gSentMotion = false;
    }
  }

  if (startRise) {
    gProtocol.sendSimpleCmd("resume");
    if (gProtocol.sendModeCmd("MANUAL")) {
      gDriveRequested = true;
    }
  }

  if (powerFall) {
    gProtocol.sendSimpleCmd("stop");
    gProtocol.sendModeCmd("IDLE");
    gDriveRequested = false;
    gSentMotion = false;
  }

  if (saltRise) {
    const bool target = !gSaltOn;
    if (gProtocol.sendOnOffCmd("salt", target)) {
      gSaltOn = target;
    }
  }

  if (augerRise) {
    const bool target = !gAugerOn;
    if (gProtocol.sendOnOffCmd("auger", target)) {
      gAugerOn = target;
    }
  }

  if (pathRise) {
    if (!gRecording) {
      if (gProtocol.sendSimpleCmd("rec_start")) {
        gRecording = true;
        gHasPath = true;
      }
    } else {
      if (gProtocol.sendSimpleCmd("rec_stop")) {
        gRecording = false;
        gHasPath = true;
      }
    }
  }

  if (clearRise && !gRecording && gHasPath) {
    if (gProtocol.sendSimpleCmd("rec_reset")) {
      gHasPath = false;
    }
  }

  updateStatusLeds();
}

void handleDriveControl() {
  if (!gProtocol.isConnected()) return;
  if (!gDriveRequested) return;
  if (gProtocol.hasTelemetry() && !gProtocol.telemetry().motionEnabled) return;
  if ((uint32_t)(millis() - gLastVelMs) < kVelocityPeriodMs) return;
  gLastVelMs = millis();

  const float linearAxis = -normalizeAxis(analogRead(LJYPin));
  const float angularAxis = normalizeAxis(analogRead(RJXPin));
  const float v = linearAxis * kMaxLinearMps;
  const float w = angularAxis * kMaxAngularRadPs;

  const bool active = (fabsf(v) > 0.01f) || (fabsf(w) > 0.05f);
  if (active) {
    if (gProtocol.sendVelCmd(v, w)) {
      gSentMotion = true;
    }
  } else if (gSentMotion) {
    if (gProtocol.sendSimpleCmd("stop")) {
      gSentMotion = false;
    }
  }
}

void handleChuteControl() {
  if (!gProtocol.isConnected()) return;
  if ((uint32_t)(millis() - gLastChuteMs) < kChutePeriodMs) return;
  gLastChuteMs = millis();

  const int raw = analogRead(ChutePini);
  const float angle = analogToChuteAngle(raw);
  if (fabsf(angle - gLastChuteSent) < kChuteStepDeg) return;
  if (gProtocol.sendChuteCmd(angle)) {
    gLastChuteSent = angle;
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  setupPins();
  gUi.begin();
  gProtocol.begin();
  updateStatusLeds();
}

void loop() {
  gProtocol.loop();

  bool connectedNow = false;
  if (gProtocol.pollConnectionChanged(&connectedNow)) {
    onConnectionChange(connectedNow);
  }

  syncFromTelemetry();
  handleButtonsAndModes();
  handleDriveControl();
  handleChuteControl();
  gProtocol.sendHeartbeatIfIdle();
  gUi.draw(gProtocol, gSaltOn, gAugerOn, gRecording);
}
