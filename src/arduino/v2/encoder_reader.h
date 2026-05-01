/**
 * SBCP (SnoBot Communication Protocol) v0.3.0
 * Quadrature encoder reader for odometry.
 */

#ifndef SBCP_ENCODER_READER_H
#define SBCP_ENCODER_READER_H

#include <Arduino.h>
#include "config.h"

/**
 * Quadrature encoder reader class.
 * Handles interrupt-based encoder tick counting with direction detection.
 */
class EncoderReader {
public:
    /**
     * Constructor.
     */
    EncoderReader();

    /**
     * Initialize encoders and attach interrupts.
     * Must be called in setup() before using encoders.
     */
    void begin();

    /**
     * Get left encoder count.
     * @return Accumulated ticks (positive = forward, negative = reverse)
     */
    int32_t getLeftCount() const { return leftCount; }

    /**
     * Get right encoder count.
     * @return Accumulated ticks (positive = forward, negative = reverse)
     */
    int32_t getRightCount() const { return rightCount; }

    /**
     * Reset both encoder counts to zero.
     */
    void reset();

    /**
     * Reset left encoder count to zero.
     */
    void resetLeft();

    /**
     * Reset right encoder count to zero.
     */
    void resetRight();

    /**
     * Set encoder counts manually (for calibration/testing).
     * @param left Left encoder count
     * @param right Right encoder count
     */
    void setCounts(int32_t left, int32_t right);

    /**
     * Static interrupt service routines (must be static for attachInterrupt).
     * These are called automatically by hardware interrupts.
     */
    static void leftISR();
    static void rightISR();

private:
    // Encoder counts (volatile because modified in ISR)
    static volatile int32_t leftCount;
    static volatile int32_t rightCount;

    // Pin state tracking
    static volatile uint8_t lastLeftA;
    static volatile uint8_t lastRightA;

    // Initialization state
    bool initialized;
};

// Global encoder reader instance
extern EncoderReader encoders;

#endif // SBCP_ENCODER_READER_H
