#ifndef STATUS_H
#define STATUS_H

#include <cstdint>
#include "rpm_data.h"
#include "dome_angle.h"

typedef struct{
    rpm_data_t measuredRPM;
    bool stableTargetRPM;
    uint32_t stableTargetRPMCount;
    uint32_t sensorTaskInterval_ms;
    dome_angle_t domeAngle;
    uint16_t pwmOutputLevel;
}status_t;

#endif // STATUS_H