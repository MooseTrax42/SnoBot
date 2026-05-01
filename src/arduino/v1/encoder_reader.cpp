/**
 * SBCP (SnoBot Communication Protocol) v0.3.0
 * Quadrature encoder reader implementation.
 */

#include "encoder_reader.h"

// Static member initialization
volatile int32_t EncoderReader::leftCount = 0;
volatile int32_t EncoderReader::rightCount = 0;
volatile uint8_t EncoderReader::lastLeftB = 0;
volatile uint8_t EncoderReader::lastRightB = 0;

// Global instance
EncoderReader encoders;

EncoderReader::EncoderReader() : initialized(false) {
}

void EncoderReader::begin() {
    if (initialized) {
        return;
    }

    // Configure encoder pins as inputs with pullups
    pinMode(PIN_ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(PIN_ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_RIGHT_B, INPUT_PULLUP);

    // Read initial state of B channels
    lastLeftB = digitalRead(PIN_ENCODER_LEFT_B);
    lastRightB = digitalRead(PIN_ENCODER_RIGHT_B);

    // Reset counts
    leftCount = 0;
    rightCount = 0;

    // Attach interrupts to channel A (rising edge)
    // Pin 2 = interrupt 0 (left encoder A)
    // Pin 7 = interrupt 4 (right encoder A) on Arduino Uno R4 WiFi
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_A), leftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_A), rightISR, RISING);

    initialized = true;
}

void EncoderReader::reset() {
    noInterrupts();
    leftCount = 0;
    rightCount = 0;
    interrupts();
}

void EncoderReader::resetLeft() {
    noInterrupts();
    leftCount = 0;
    interrupts();
}

void EncoderReader::resetRight() {
    noInterrupts();
    rightCount = 0;
    interrupts();
}

void EncoderReader::setCounts(int32_t left, int32_t right) {
    noInterrupts();
    leftCount = left;
    rightCount = right;
    interrupts();
}

// Left encoder ISR
// Quadrature encoding: when A rises, check B to determine direction
// If B is HIGH, we're moving backward; if LOW, moving forward
void EncoderReader::leftISR() {
    uint8_t bState = digitalRead(PIN_ENCODER_LEFT_B);
    if (bState == HIGH) {
        leftCount--;  // Backward
    } else {
        leftCount++;  // Forward
    }
}

// Right encoder ISR
// Same logic as left encoder
void EncoderReader::rightISR() {
    uint8_t bState = digitalRead(PIN_ENCODER_RIGHT_B);
    if (bState == HIGH) {
        rightCount--;  // Backward
    } else {
        rightCount++;  // Forward
    }
}
