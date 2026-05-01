#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"

#if CONTROLLER_TRANSPORT_BT_SPP
#include <BluetoothSerial.h>
#endif

#ifndef CONTROLLER_BT_NAME
#define CONTROLLER_BT_NAME "SnoBot_Controller"
#endif

#ifndef JETSON_BT_NAME
#define JETSON_BT_NAME "DESKTOP_NVA4SUO"
#endif

struct TelemetryState {
  char state[18] = "BOOT";
  char recState[12] = "n/a";
  bool motionEnabled = false;
  bool estop = false;
  bool augerOn = false;
  bool saltOn = false;
  bool batteryValid = false;
  int faults = 0;
  int recWaypoints = 0;
  float battery = 0.0f;
  float v = 0.0f;
  float w = 0.0f;
  float vTarget = 0.0f;
  float wTarget = 0.0f;
  float chute = 0.0f;
  uint32_t updatedAtMs = 0;
};

class ControllerProtocol {
 public:
#if CONTROLLER_TRANSPORT_BT_SPP
  ControllerProtocol();
#else
  explicit ControllerProtocol(HardwareSerial& link = CONTROLLER_LINK_SERIAL);
#endif

  bool begin();
  void loop();

  bool isConnected() const;
  bool pollConnectionChanged(bool* connectedNow);

  bool hasTelemetry() const;
  const TelemetryState& telemetry() const;

  const char* lastError() const;
  void clearError();

  bool sendSimpleCmd(const char* cmd);
  bool sendModeCmd(const char* mode);
  bool sendVelCmd(float v, float w);
  bool sendOnOffCmd(const char* cmd, bool on);
  bool sendChuteCmd(float angleDeg);
  void sendHeartbeatIfIdle();

 private:
  struct AckState {
    bool pending = false;
    bool hasSeq = false;
    bool ok = false;
    uint32_t seq = 0;
    char cmd[20] = {0};
    char err[96] = {0};
  };

  template <size_t N>
  bool sendCommand(StaticJsonDocument<N>& doc, bool expectAck, uint32_t timeoutMs);

  void tryReconnect();
  void pumpBluetooth();
  bool waitForAck(uint32_t expectedSeq, const char* expectedCmd, uint32_t timeoutMs);
  void handleIncomingLine(const char* line);
  void handleAck(const JsonVariantConst& root);
  void handleTelemetry(const JsonVariantConst& root);
#if CONTROLLER_TRANSPORT_BT_SPP
  bool connectToHost();
#endif
  void setError(const char* msg);
  static void copyText(char* dst, const char* src, size_t dstLen);
  static float clampf(float x, float lo, float hi);

#if CONTROLLER_TRANSPORT_BT_SPP
  BluetoothSerial bt_;
#else
  HardwareSerial& link_;
#endif
  TelemetryState telemetry_;
  AckState ack_;

  char rxLine_[512] = {0};
  size_t rxLen_ = 0;
  char lastError_[96] = {0};

  uint32_t nextSeq_ = 1;
  uint32_t lastTxMs_ = 0;
#if CONTROLLER_TRANSPORT_BT_SPP
  uint32_t lastConnectAttemptMs_ = 0;
  uint8_t btDisconnectedStreak_ = 0;
#endif

  bool haveTelemetry_ = false;
  bool connectedNow_ = false;
  bool connectedLastLoop_ = false;
  bool connectionChanged_ = false;
};
