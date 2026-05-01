#include "controller_protocol.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

namespace {
// Classic BT RFCOMM on laptops can occasionally exceed sub-500 ms roundtrip.
// Keep this conservative so transient scheduling jitter does not flap links.
constexpr uint32_t kAckTimeoutMs = 1200;
constexpr uint32_t kHeartbeatMs = 1000;
constexpr float kMaxLinearMps = 1.5f;
constexpr float kMaxAngularRadPs = 3.14f;
#if CONTROLLER_TRANSPORT_BT_SPP
constexpr uint32_t kReconnectEveryMs = 3000;
constexpr uint8_t kDisconnectConfirmSamples = 6;
#endif
}  // namespace

#if CONTROLLER_TRANSPORT_BT_SPP
ControllerProtocol::ControllerProtocol() = default;
#else
ControllerProtocol::ControllerProtocol(HardwareSerial& link) : link_(link) {}
#endif

bool ControllerProtocol::begin() {
#if CONTROLLER_TRANSPORT_BT_SPP
  if (!bt_.begin(CONTROLLER_BT_NAME, true)) {
    setError("Bluetooth init failed");
    return false;
  }
  connectedNow_ = connectToHost();
  btDisconnectedStreak_ = 0;
#else
  link_.begin(CONTROLLER_LINK_BAUD);
  connectedNow_ = true;
#endif
  connectedLastLoop_ = connectedNow_;
  connectionChanged_ = connectedNow_;
  return true;
}

void ControllerProtocol::loop() {
#if CONTROLLER_TRANSPORT_BT_SPP
  tryReconnect();
#endif
  pumpBluetooth();

#if CONTROLLER_TRANSPORT_BT_SPP
  if (bt_.connected()) {
    connectedNow_ = true;
    btDisconnectedStreak_ = 0;
  } else if (connectedNow_) {
    if (btDisconnectedStreak_ < 255) {
      btDisconnectedStreak_++;
    }
    if (btDisconnectedStreak_ >= kDisconnectConfirmSamples) {
      connectedNow_ = false;
    }
  } else {
    connectedNow_ = false;
  }
#else
  connectedNow_ = true;
#endif
  if (connectedNow_ != connectedLastLoop_) {
    connectedLastLoop_ = connectedNow_;
    connectionChanged_ = true;
  }
}

bool ControllerProtocol::isConnected() const {
  return connectedNow_;
}

bool ControllerProtocol::pollConnectionChanged(bool* connectedNow) {
  if (!connectionChanged_) {
    return false;
  }
  connectionChanged_ = false;
  if (connectedNow != nullptr) {
    *connectedNow = connectedNow_;
  }
  return true;
}

bool ControllerProtocol::hasTelemetry() const {
  return haveTelemetry_;
}

const TelemetryState& ControllerProtocol::telemetry() const {
  return telemetry_;
}

const char* ControllerProtocol::lastError() const {
  return lastError_;
}

void ControllerProtocol::clearError() {
  lastError_[0] = '\0';
}

bool ControllerProtocol::sendSimpleCmd(const char* cmd) {
  StaticJsonDocument<64> doc;
  doc["cmd"] = cmd;
  return sendCommand(doc, true, kAckTimeoutMs);
}

bool ControllerProtocol::sendModeCmd(const char* mode) {
  StaticJsonDocument<96> doc;
  doc["cmd"] = "mode";
  doc["mode"] = mode;
  return sendCommand(doc, true, kAckTimeoutMs);
}

bool ControllerProtocol::sendVelCmd(float v, float w) {
  StaticJsonDocument<96> doc;
  doc["cmd"] = "vel";
  doc["v"] = clampf(v, -kMaxLinearMps, kMaxLinearMps);
  doc["w"] = clampf(w, -kMaxAngularRadPs, kMaxAngularRadPs);
  return sendCommand(doc, true, kAckTimeoutMs);
}

bool ControllerProtocol::sendOnOffCmd(const char* cmd, bool on) {
  StaticJsonDocument<96> doc;
  doc["cmd"] = cmd;
  doc["on"] = on;
  return sendCommand(doc, true, kAckTimeoutMs);
}

bool ControllerProtocol::sendChuteCmd(float angleDeg) {
  StaticJsonDocument<96> doc;
  doc["cmd"] = "chute";
  doc["angle"] = clampf(angleDeg, -90.0f, 90.0f);
  return sendCommand(doc, true, kAckTimeoutMs);
}

void ControllerProtocol::sendHeartbeatIfIdle() {
  if (!isConnected()) return;
  if ((uint32_t)(millis() - lastTxMs_) < kHeartbeatMs) return;
  StaticJsonDocument<48> doc;
  doc["cmd"] = "hb";
  sendCommand(doc, false, 0);
}

template <size_t N>
bool ControllerProtocol::sendCommand(StaticJsonDocument<N>& doc, bool expectAck, uint32_t timeoutMs) {
  if (!connectedNow_) {
    setError("Bluetooth not connected");
    return false;
  }

  char cmdName[20];
  copyText(cmdName, doc["cmd"] | "", sizeof(cmdName));

  uint32_t seq = 0;
  if (expectAck) {
    seq = nextSeq_++;
    doc["seq"] = seq;
  }

#if CONTROLLER_TRANSPORT_BT_SPP
  if (serializeJson(doc, bt_) == 0) {
#else
  if (serializeJson(doc, link_) == 0) {
#endif
    setError("Failed to serialize command");
#if CONTROLLER_TRANSPORT_BT_SPP
    connectedNow_ = false;
#endif
    return false;
  }
#if CONTROLLER_TRANSPORT_BT_SPP
  if (bt_.write('\n') != 1) {
    setError("BT write failed");
    connectedNow_ = false;
    return false;
  }
#else
  link_.write('\n');
#endif
  lastTxMs_ = millis();

  if (!expectAck) {
    return true;
  }
  return waitForAck(seq, cmdName, timeoutMs);
}

void ControllerProtocol::tryReconnect() {
#if CONTROLLER_TRANSPORT_BT_SPP
  if (connectedNow_) return;
  if ((uint32_t)(millis() - lastConnectAttemptMs_) < kReconnectEveryMs) return;
  lastConnectAttemptMs_ = millis();
  const char* addrText = JETSON_BT_ADDR;
  if (addrText != nullptr && addrText[0] != '\0') {
    Serial.print("[BT] Connecting to ");
    Serial.print(addrText);
    Serial.print(" ch ");
    Serial.println((int)JETSON_BT_CHANNEL);
  } else {
    Serial.print("[BT] Connecting to ");
    Serial.println(JETSON_BT_NAME);
  }
  if (connectToHost()) {
    connectedNow_ = true;
    btDisconnectedStreak_ = 0;
  }
#endif
}

#if CONTROLLER_TRANSPORT_BT_SPP
bool ControllerProtocol::connectToHost() {
  const char* addrText = JETSON_BT_ADDR;
  if (addrText != nullptr && addrText[0] != '\0') {
    unsigned int b0, b1, b2, b3, b4, b5;
    int matched = sscanf(addrText, "%02x:%02x:%02x:%02x:%02x:%02x", &b0, &b1, &b2, &b3, &b4, &b5);
    if (matched == 6) {
      uint8_t addr[6] = {
        (uint8_t)b0,
        (uint8_t)b1,
        (uint8_t)b2,
        (uint8_t)b3,
        (uint8_t)b4,
        (uint8_t)b5,
      };
      bool ok = bt_.connect(addr, JETSON_BT_CHANNEL);
      if (!ok) {
        setError("BT connect by MAC/channel failed");
        // Fallback for platforms where RFCOMM by addr/channel is flaky.
        if (bt_.connect(JETSON_BT_NAME)) {
          clearError();
          return true;
        }
      }
      return ok;
    }
    setError("Invalid JETSON_BT_ADDR format");
    return false;
  }

  bool ok = bt_.connect(JETSON_BT_NAME);
  if (!ok) {
    setError("BT connect by name failed");
  }
  return ok;
}
#endif

void ControllerProtocol::pumpBluetooth() {
#if CONTROLLER_TRANSPORT_BT_SPP
  while (bt_.available() > 0) {
    char c = (char)bt_.read();
#else
  while (link_.available() > 0) {
    char c = (char)link_.read();
#endif
    if (c == '\r') continue;
    if (c == '\n') {
      if (rxLen_ > 0) {
        rxLine_[rxLen_] = '\0';
        handleIncomingLine(rxLine_);
        rxLen_ = 0;
      }
      continue;
    }
    if (rxLen_ < sizeof(rxLine_) - 1) {
      rxLine_[rxLen_++] = c;
    } else {
      rxLen_ = 0;
    }
  }
}

bool ControllerProtocol::waitForAck(uint32_t expectedSeq, const char* expectedCmd, uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((uint32_t)(millis() - start) < timeoutMs) {
    pumpBluetooth();
    if (ack_.pending) {
      const bool match = (ack_.hasSeq && ack_.seq == expectedSeq) ||
                         (!ack_.hasSeq && strcmp(ack_.cmd, expectedCmd) == 0);
      if (match) {
        const bool ok = ack_.ok;
        if (!ok) {
          if (ack_.err[0] != '\0') {
            setError(ack_.err);
          } else {
            setError("Command rejected");
          }
        } else {
          clearError();
        }
        ack_.pending = false;
        return ok;
      }
      ack_.pending = false;
    }
    delay(1);
  }
  char msg[96];
  snprintf(msg, sizeof(msg), "ACK timeout on %s", expectedCmd);
  setError(msg);
  // Do not force disconnect on a single ACK timeout. Let higher-level
  // reconnect logic react only to actual link loss from bt_.connected().
  return false;
}

void ControllerProtocol::handleIncomingLine(const char* line) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    return;
  }

  JsonVariantConst root = doc.as<JsonVariantConst>();
  if (root.containsKey("ack")) {
    handleAck(root);
    return;
  }
  if ((root["t"] | 0) == 1) {
    handleTelemetry(root);
  }
}

void ControllerProtocol::handleAck(const JsonVariantConst& root) {
  ack_.pending = true;
  ack_.hasSeq = root.containsKey("seq");
  ack_.ok = root["ok"] | false;
  ack_.seq = root["seq"] | 0;
  copyText(ack_.cmd, root["ack"] | "", sizeof(ack_.cmd));
  copyText(ack_.err, root["err"] | "", sizeof(ack_.err));
}

void ControllerProtocol::handleTelemetry(const JsonVariantConst& root) {
  copyText(telemetry_.state, root["st"] | telemetry_.state, sizeof(telemetry_.state));
  copyText(telemetry_.recState, root["rec"] | telemetry_.recState, sizeof(telemetry_.recState));
  telemetry_.motionEnabled = (root["mot"] | 0) != 0;
  telemetry_.estop = (root["e"] | 0) != 0;
  telemetry_.augerOn = (root["aug"] | 0) != 0;
  telemetry_.saltOn = (root["sal"] | 0) != 0;
  telemetry_.chute = root["ch"] | telemetry_.chute;
  telemetry_.recWaypoints = root["rwp"] | telemetry_.recWaypoints;

  JsonVariantConst bat = root["bat"];
  telemetry_.batteryValid = !bat.isNull();
  if (telemetry_.batteryValid) {
    telemetry_.battery = bat.as<float>();
  }

  JsonVariantConst v = root["v"];
  if (v.is<JsonArrayConst>()) {
    JsonArrayConst arr = v.as<JsonArrayConst>();
    if (arr.size() >= 2) {
      telemetry_.v = arr[0] | telemetry_.v;
      telemetry_.w = arr[1] | telemetry_.w;
    }
  }

  JsonVariantConst vt = root["vt"];
  if (vt.is<JsonArrayConst>()) {
    JsonArrayConst arr = vt.as<JsonArrayConst>();
    if (arr.size() >= 2) {
      telemetry_.vTarget = arr[0] | telemetry_.vTarget;
      telemetry_.wTarget = arr[1] | telemetry_.wTarget;
    }
  }

  telemetry_.faults = 0;
  JsonVariantConst f = root["f"];
  if (f.is<JsonArrayConst>()) {
    telemetry_.faults = (int)f.as<JsonArrayConst>().size();
  }

  telemetry_.updatedAtMs = millis();
  haveTelemetry_ = true;
}

void ControllerProtocol::setError(const char* msg) {
  copyText(lastError_, msg, sizeof(lastError_));
  Serial.print("[ERR] ");
  Serial.println(lastError_);
}

void ControllerProtocol::copyText(char* dst, const char* src, size_t dstLen) {
  if (dstLen == 0) return;
  if (src == nullptr) src = "";
  strncpy(dst, src, dstLen - 1);
  dst[dstLen - 1] = '\0';
}

float ControllerProtocol::clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
