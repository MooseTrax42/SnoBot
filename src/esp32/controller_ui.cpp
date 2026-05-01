#include "controller_ui.h"

namespace {
constexpr uint32_t kUiPeriodMs = 250;
}

ControllerUi::ControllerUi() : lcd_(21, 22, 23) {}

void ControllerUi::begin() {
  lcd_.init(SCREEN_WIDTH, SCREEN_HEIGHT);
  lcd_.setRotation(0);
  lcd_.fillScreen(ST77XX_BLACK);
  lcd_.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  lcd_.setTextSize(1);
}

void ControllerUi::draw(const ControllerProtocol& protocol, bool saltOn, bool augerOn, bool recording) {
  if ((uint32_t)(millis() - lastUiMs_) < kUiPeriodMs) return;
  lastUiMs_ = millis();

  const TelemetryState& t = protocol.telemetry();
  const bool hasTelemetry = protocol.hasTelemetry();
  const char* error = protocol.lastError();

  lcd_.fillScreen(ST77XX_BLACK);
  lcd_.setTextSize(1);
  lcd_.setTextWrap(false);
  lcd_.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  lcd_.setCursor(0, 0);
  lcd_.print("BT: ");
  lcd_.print(protocol.isConnected() ? "CONNECTED" : "DISCONNECTED");

  lcd_.setCursor(0, 14);
  lcd_.print("STATE: ");
  lcd_.print(hasTelemetry ? t.state : "WAITING");

  lcd_.setCursor(0, 28);
  lcd_.print("BAT: ");
  if (hasTelemetry && t.batteryValid) {
    lcd_.print(t.battery, 1);
    lcd_.print("V");
  } else {
    lcd_.print("n/a");
  }

  lcd_.setCursor(0, 42);
  lcd_.print("v:");
  lcd_.print(t.v, 2);
  lcd_.print(" w:");
  lcd_.print(t.w, 2);

  lcd_.setCursor(0, 56);
  lcd_.print("aug:");
  lcd_.print(augerOn ? "ON " : "OFF");
  lcd_.print(" sal:");
  lcd_.print(saltOn ? "ON" : "OFF");

  lcd_.setCursor(0, 70);
  lcd_.print("ch:");
  lcd_.print(t.chute, 1);
  lcd_.print(" rec:");
  lcd_.print(recording ? "recording" : t.recState);

  lcd_.setCursor(0, 84);
  lcd_.print("rwp:");
  lcd_.print(t.recWaypoints);
  lcd_.print(" f:");
  lcd_.print(t.faults);

  lcd_.setCursor(0, 98);
  if (t.estop) {
    lcd_.setTextColor(ST77XX_RED, ST77XX_BLACK);
    lcd_.print("E-STOP ACTIVE");
  } else if (hasTelemetry && !t.motionEnabled) {
    lcd_.setTextColor(ST77XX_RED, ST77XX_BLACK);
    lcd_.print("DRIVE LOCKED");
  } else {
    lcd_.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    lcd_.print("DRIVE READY");
  }

  if (error != nullptr && error[0] != '\0') {
    lcd_.setTextColor(ST77XX_RED, ST77XX_BLACK);
    lcd_.setCursor(0, 112);
    lcd_.print("ERR: ");
    lcd_.print(error);
  }
}

