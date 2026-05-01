#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#include "config.h"
#include "controller_protocol.h"

class ControllerUi {
 public:
  ControllerUi();

  void begin();
  void draw(const ControllerProtocol& protocol, bool saltOn, bool augerOn, bool recording);

 private:
  Adafruit_ST7789 lcd_;
  uint32_t lastUiMs_ = 0;
};

