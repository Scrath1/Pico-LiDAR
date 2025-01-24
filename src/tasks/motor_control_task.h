#ifndef MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include "signals/signal_types.h"
#include "runtime_settings.h"

#define MOTOR_CONTROL_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define MOTOR_CONTROL_TASK_NAME ("mtrCtrlTsk")
#define MOTOR_CONTROL_TASK_PRIORITY (configMAX_PRIORITIES-3)

#if (MOTOR_CONTROL_TASK_PRIORITY >= configMAX_PRIORITIES)
    #error "Motor control task priority too high"
#endif

typedef struct {
    rpm_signal_t& measuredRPMSignal;
    runtime_settings_signal_t& runtimeSettingsSignal;
} motorControlTaskParams_t;

extern TaskHandle_t motorCtrlTaskHandle;
void motorControlTask(void* pvParameters);

#endif // MOTOR_CONTROL_TASK_H