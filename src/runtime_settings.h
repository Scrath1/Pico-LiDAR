#ifndef RUNTIME_SETTINGS_H
#define RUNTIME_SETTINGS_H

typedef struct{
    struct{
        float kp;
        float ki;
        float kd;
    } pid_controller;
} runtime_settings_t;

#endif // RUNTIME_SETTINGS_H