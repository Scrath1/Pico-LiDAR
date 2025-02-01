#ifndef DOME_ANGLE_H
#define DOME_ANGLE_H

#include <pico/stdlib.h>

#include "prj_config.h"

typedef struct {
    // stores the base angle as determined by hall sensor pulses
    // in 90 degree steps with rollover at 360 degree => [0, 90, 180, 270, 0 ...]
    uint16_t angleBase;
    // Uses a microsend timestamp of when the angle was last incremented to
    // allow for calculation of actual current angle with current timestamp
    uint32_t timeOfAngleIncrement_us;

    /**
     * @brief Attempts to calculate the current angle position based on
     *  the hall effect sensor pulses, the time since the last pulse and
     *  the current rotation speed
     * @param domeRPM [IN] Rotation speed of dome
     * @return expected current angle of dome in degrees
     */
    inline uint16_t calculateCurrentAngle(uint32_t domeRPM) {
        const uint32_t curTime_us = time_us_32();
        // calculate time in us for a full rotation
        const uint32_t fullRotationTime_us = 1e6 / (domeRPM / 60);
        // ideal time in us between each hall effect pulse
        const uint32_t pulseRotationTime_us = fullRotationTime_us / PULSES_PER_REV;
        const uint32_t timeSinceLastPulse_us = curTime_us - timeOfAngleIncrement_us;
        uint16_t angleIncrement = (360 / PULSES_PER_REV) / (pulseRotationTime_us / timeSinceLastPulse_us);
        return (angleBase + angleIncrement) % 360;
    }
} dome_angle_t;

#endif  // DOME_ANGLE_H