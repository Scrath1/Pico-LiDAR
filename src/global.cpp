#include "global.h"
#include "prj_config.h"

runtime_settings_t runtimeSettings = {
    .pid_kp = {K_P, "K_p"},
    .pid_ki = {K_I, "K_i"},
    .pid_kd = {K_D, "K_d"},
    .targetRPM = {MOTOR_TARGET_SPEED, "targetRPM"},
    .enableMotor = {0, "enableMotor"},
    .vl53l0xMeasurementTimingBudget_us = {RANGE_ACQUISITION_TIME_BUDGET_MS * 1000, "VL53L0X_Measurement_Budget"},
    .dataPointsPerRev = {DEFAULT_SCANPOINTS_PER_REV, "scanpoints"}
};

// will be handled at runtime. Only do a zero initialization
status_t status = {
    .measuredRPM = {.rpm = 0, .timestampLastChanged_ms = 0},
    .stableTargetRPM = false,
    .stableTargetRPMCount = 0,
    .sensorTaskInterval_ms = 0,
    .domeAngle = {.angleBase = 0, .timeOfAngleIncrement_us = 0},
    .pwmOutputLevel = 0
};