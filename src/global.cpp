#include "global.h"

volatile uint16_t targetRPM = 0;
volatile uint16_t measuredRPM = 0;
volatile uint32_t lastRPMUpdateTime_us = 0;
volatile uint32_t expectedRPMUpdateInterval_us = 0;