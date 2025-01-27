#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include "signals/signal_types.h"
#include "runtime_settings.h"

#define SENSOR_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define SENSOR_TASK_NAME ("sensorTsk")
#define SENSOR_TASK_PRIORITY (configMAX_PRIORITIES-1)

#if (SENSOR_TASK_PRIORITY >= configMAX_PRIORITIES)
    #error "Serial interface task priority too high"
#endif

typedef struct {
    rpm_signal_t& measuredRPMSignal;
    runtime_settings_signal_t& runtimeSettingsSignal;
    dome_angle_signal_t& domeAngleSignal;
} sensorTaskParams_t;

extern TaskHandle_t sensorTaskHandle;
void sensorTask(void* pvParameters);

#endif // SENSOR_TASK_H