/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified config information.
*/

#ifndef SBCP_CONFIG_H
#define SBCP_CONFIG_H

#include <Arduino.h>

// Protocol version.
#define SBCP_VERSION "0.3.0"
#define SBCP_MIN_VERSION "0.3.0"
#define SBCP_MAX_VERSION "0.3.9"

// Drive motors.
#define PIN_MOTOR_LEFT_PWM_L 5      // Left motor forward PWM.
#define PIN_MOTOR_LEFT_PWM_R 6      // Left motor reverse PWM.
#define PIN_MOTOR_RIGHT_PWM_L 9     // Right motor forward PWM.
#define PIN_MOTOR_RIGHT_PWM_R 10    // Right motor reverse PWM.
#define PIN_MOTOR_DRIVE_ENABLE 12   // Motor driver enable.

// Encoders.
#define PIN_ENCODER_LEFT_A 2        // Left encoder channel A.
#define PIN_ENCODER_LEFT_B 4        // Left encoder channel B.
#define PIN_ENCODER_RIGHT_A 7       // Right encoder channel A.
#define PIN_ENCODER_RIGHT_B 8       // Right encoder channel B.

// Actuators.
#define PIN_AUGER_ENABLE A5         // Auger motor enable.
#define PIN_SALT_ENABLE A4          // Salt dispenser enable.
#define PIN_CHUTE_PWM 11            // Chute servo PWM.

// Safety and sensors.
#define PIN_ESTOP 3                 // E-stop button (INT, active LOW).
#define PIN_BATTERY_SENSE A0        // Battery voltage divider.
#define ESTOP_USE_PIN 0             // Set to 0 to ignore the physical E-stop pin (software only).

// Status indicators.
#define PIN_LED_STATUS 13           // Status LED (built-in).
#define PIN_LED_ERROR A3            // Error indicator.
#define PIN_LED_HEAD A2             // Headlight control.

// Drivetrain geometry.
#define WHEELBASE 0.588625f        // Distance between wheels (meters).
#define WHEEL_RADIUS 0.127f         // Wheel radius (meters).
#define DRIVE_MAX_LIN_VEL 1.35f
#define DRIVE_MAX_ANG_VEL 3.14f

// Encoder specifications.
#define ENCODER_TICKS_PER_REV 600   // Encoder resolution (ticks/revolution).
#define GEAR_RATIO 18.0f            // Motor gearbox ratio.

// Chute servo limits.
#define CHUTE_MIN_ANGLE 0           // Minimum servo angle (degrees).
#define CHUTE_MAX_ANGLE 180         // Maximum servo angle (degrees).
#define CHUTE_CENTER_ANGLE 90       // Centered position (degrees).

// Battery protections.
#define BATTERY_CRITICAL_V 43.0f    // Below this, refuse all motion.
#define BATTERY_WARNING_V 44.0f     // Below this, warning only.
#define BATTERY_VOLTAGE_DIVIDER 11.0f  // Voltage divider ratio (13S LiPo).

// Temperature protection.
#define TEMP_CRITICAL_C 75.0f       // Above this, thermal shutdown.
#define TEMP_WARNING_C 60.0f        // Above this, warning.

// Motor current limits.
#define AUGER_MOTOR_CURRENT_MAX_A 30.0f     // Maximum safe motor current.
#define AUGER_MOTOR_CURRENT_WARNING_A 25.0f // Warning threshold.
#define DRIVE_MOTOR_CURRENT_MAX_A 12.0f
#define DRIVE_MOTOR_CURRENT_WARNING_A 10.f

// Serial communication.
#define SERIAL_BAUD 115200          // Baud rate for Jetson communication.
#define JSON_BUFFER_SIZE 512        // JSON command buffer size.
#define JSON_RESPONSE_SIZE 768      // JSON response/sensor buffer size.

// Safety timeout.
#define WATCHDOG_TIMEOUT_MS 1500    // Stop if no command recieved.

// Motor control.
#define MOTOR_PWM_MAX 255           // Maximum PWM value (8-bit).
#define MOTOR_PWM_DEADBAND 10       // Minimum PWM to overcome friction.

// Rates.
#define SENSOR_STREAM_PERIOD_MS 200  // Sensor stream rate is 5 Hz.
#define MAIN_LOOP_PERIOD_MS 25       // Main loop target is 40 Hz.

// Debug heartbeat LED.
#define HEARTBEAT_LED_ENABLE 1
#define HEARTBEAT_PERIOD_MS 500

#endif // SBCP_CONFIG_H
