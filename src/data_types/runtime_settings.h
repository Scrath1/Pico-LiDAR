#ifndef RUNTIME_SETTINGS_H
#define RUNTIME_SETTINGS_H
#include "setting_template.h"

typedef struct {
    // PID K_p variable
    setting<float> pid_kp;
    // PID K_i variable
    setting<float> pid_ki;
    // PID K_d variable
    setting<float> pid_kd;
    // RPM the dome is supposed to spin at
    setting<uint32_t> targetRPM;
    // Motor enable signal
    setting<bool> enableMotor;
    // Maximum amount of time per measurement for the VL53L0X
    setting<uint32_t> vl53l0xMeasurementTimingBudget_us;
    // Number of measurement points per full revolution of the dome
    setting<uint16_t> dataPointsPerRev;
} runtime_settings_t;

#endif  // RUNTIME_SETTINGS_H