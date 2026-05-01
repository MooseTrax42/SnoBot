/**
 * SBCP (SnoBot Communication Protocol) v0.3.0
 * Quadrature encoder reader implementation.
 */

#include "encoder_reader.h"

// Static member initialization
volatile int32_t EncoderReader::leftCount = 0;
volatile int32_t EncoderReader::rightCount = 0;
volatile uint8_t EncoderReader::lastLeftA = HIGH;
volatile uint8_t EncoderReader::lastRightA = HIGH;

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

    // Read initial state of A channels
    lastLeftA = digitalRead(PIN_ENCODER_LEFT_A);
    lastRightA = digitalRead(PIN_ENCODER_RIGHT_A);

    // Reset counts
    leftCount = 0;
    rightCount = 0;

    // Attach interrupts to channel A (change).
    // Left A on D2 and right A on D3 for broad board compatibility.
    int leftInterrupt = digitalPinToInterrupt(PIN_ENCODER_LEFT_A);
    int rightInterrupt = digitalPinToInterrupt(PIN_ENCODER_RIGHT_A);
    if (leftInterrupt < 0 || rightInterrupt < 0) {
        Serial.println("[EncoderReader] ERROR: encoder A pin is not interrupt-capable");
        initialized = false;
        return;
    }
    attachInterrupt(leftInterrupt, leftISR, CHANGE);
    attachInterrupt(rightInterrupt, rightISR, CHANGE);

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
// Quadrature encoding: when A toggles, check B to determine direction
void EncoderReader::leftISR() {
    uint8_t aState = digitalRead(PIN_ENCODER_LEFT_A);
    uint8_t bState = digitalRead(PIN_ENCODER_LEFT_B);

    if (aState == lastLeftA) {
        return;
    }

    if (aState == bState) {
        leftCount++;
    } else {
        leftCount--;
    }

    lastLeftA = aState;
}

// Right encoder ISR
// Right side is mirrored relative to left, so invert sign to keep
// "forward robot motion" positive on both wheels.
void EncoderReader::rightISR() {
    uint8_t aState = digitalRead(PIN_ENCODER_RIGHT_A);
    uint8_t bState = digitalRead(PIN_ENCODER_RIGHT_B);

    if (aState == lastRightA) {
        return;
    }

    if (aState == bState) {
        rightCount--;
    } else {
        rightCount++;
    }

    lastRightA = aState;
}
