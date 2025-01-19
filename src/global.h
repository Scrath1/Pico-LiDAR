#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdint.h>

extern volatile uint16_t targetRPM;
extern volatile uint16_t measuredRPM;
extern volatile uint32_t lastRPMUpdateTime_us;
extern volatile uint32_t expectedRPMUpdateInterval_us;

#endif // GLOBAL_H