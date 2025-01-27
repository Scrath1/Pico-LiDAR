#ifndef RUNTIME_SETTINGS_H
#define RUNTIME_SETTINGS_H

typedef struct{
    struct{
        float kp;
        float ki;
        float kd;
        uint32_t targetRPM;
    } pid_controller;
    bool enableMotor;
    // indicates whether the targetRPM value has been
    // reached by measuredRPM and is relatively stable
    bool stableTargetRPM;
} runtime_settings_t;

#endif // RUNTIME_SETTINGS_H