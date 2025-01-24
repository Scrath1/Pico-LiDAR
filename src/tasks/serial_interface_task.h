#ifndef SERIAL_INTERFACE_TASK_H
#define SERIAL_INTERFACE_TASK_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include "signals/signal_types.h"
#include "runtime_settings.h"

#define SERIAL_INTERFACE_TASK_STACK_SIZE (1024)
#define SERIAL_INTERFACE_TASK_NAME ("serInterfaceTsk")
#define SERIAL_INTERFACE_TASK_PRIORITY (configMAX_PRIORITIES-4)

#if (SERIAL_INTERFACE_TASK_PRIORITY >= configMAX_PRIORITIES)
    #error "Serial interface task priority too high"
#endif

typedef struct {
    rpm_signal_t& measuredRPMSignal;
    runtime_settings_signal_t& runtimeSettingsSignal;
} serialInterfaceTaskParams_t;

extern TaskHandle_t serialInterfaceTaskHandle;
void serialInterfaceTask(void* pvParameters);

#endif // SERIAL_INTERFACE_TASK_H