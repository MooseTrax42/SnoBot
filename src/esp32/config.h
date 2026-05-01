// Config for the remote controller.

#ifndef ESP32_REMOTE_CONFIG_H
#define ESP32_REMOTE_CONFIG_H

// Transport selection:
// 1 = ESP32 Classic Bluetooth SPP client (recommended for ESP-WROOM-32)
// 0 = UART transport (e.g., external BT serial module)
#ifndef CONTROLLER_TRANSPORT_BT_SPP
#if defined(ARDUINO_ARCH_ESP32)
#define CONTROLLER_TRANSPORT_BT_SPP 1
#else
#define CONTROLLER_TRANSPORT_BT_SPP 0
#endif
#endif

// UART transport defaults (used when CONTROLLER_TRANSPORT_BT_SPP = 0).
#ifndef CONTROLLER_LINK_SERIAL
#define CONTROLLER_LINK_SERIAL Serial1
#endif

#ifndef CONTROLLER_LINK_BAUD
#define CONTROLLER_LINK_BAUD 115200
#endif

// Bluetooth target settings for SPP client mode.
// Name-based connect is convenient, but Windows RFCOMM is often more reliable
// with an explicit MAC + channel.
#ifndef JETSON_BT_CHANNEL
#define JETSON_BT_CHANNEL 4
#endif

// Optional explicit remote MAC (format: "AA:BB:CC:DD:EE:FF").
// Leave empty to use name-based connect via JETSON_BT_NAME.
#ifndef JETSON_BT_ADDR
#define JETSON_BT_ADDR "D0:AB:D5:22:7A:7B"
#endif

// Serial input debugging (prints controller raw/normalized inputs).
#ifndef CONTROLLER_DEBUG_INPUTS
#define CONTROLLER_DEBUG_INPUTS 1
#endif

#ifndef CONTROLLER_DEBUG_INPUT_PERIOD_MS
#define CONTROLLER_DEBUG_INPUT_PERIOD_MS 200
#endif

// ===== SCREEN ===== //
#define SCREEN_WIDTH 135  // OLED display width,  in pixels.
#define SCREEN_HEIGHT 240 // OLED display height, in pixels.

// Board fallback aliases for analog-style pins on cores that do not define A0/A3/A4/A6/A7.
#ifndef A0
#define A0 36
#endif
#ifndef A3
#define A3 39
#endif
#ifndef A4
#define A4 32
#endif
#ifndef A6
#define A6 34
#endif
#ifndef A7
#define A7 35
#endif
#ifndef A16
#define A16 16
#endif
#ifndef A17
#define A17 17
#endif

// ===== PINS ===== //

// Inputs.
#define EMERPin 19                 // Pin4EMERGENCY_GPIO19
#define SaltPini 16                // Pin4SaltButt_GPIO16
#define AugerPini 17               // Pin4AugerButt_GPIO17
#define StartPini 4                // Pin4StartButt_GPIO4
#define PowerOn 2                  // Pin4PowerSwitch_GPIO2
#define PathPin 18                 // Pin4Path_GPIO18
#define ClearPin 5                 // Pin4Clear_GPIO5
#define ChutePini A4               // Pin4ChutePot_A4
#define LJXPin A0                  // Pin4LJoystickX_A0
#define LJYPin A3                  // Pin4LJoystickY_A3
#define RJXPin A6                  // Pin4RJoystickX_A6
#define RJYPin A7                  // Pin4RJoystickY_A7
#define LWheelEncoder A17          // Pin4LWheelEncoder_A17
#define RWheelEncoder A16          // Pin4RWheelEncoder_A16

// Outputs.
#define SaltLEDPin 33              // Pin4SaltLED_GPIO33
#define AugerLEDPin 25             // Pin4AugerLED_GPIO25
#define PathLEDPin 26              // Pin4PathLED_GPIO26

#endif  // ESP32_REMOTE_CONFIG_H
