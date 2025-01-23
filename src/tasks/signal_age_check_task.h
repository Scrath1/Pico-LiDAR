#ifndef SIGNAL_AGE_CHECK_TASK_H
#define SIGNAL_AGE_CHECK_TASK_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include "signals/signal_types.h"

#define SIGNAL_AGE_CHECK_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define SIGNAL_AGE_CHECK_TASK_NAME ("sigAgeChkTsk")
#define SIGNAL_AGE_CHECK_TASK_PRIORITY (configMAX_PRIORITIES-3)

typedef struct {
    rpm_signal_t& measuredRPMSignal;
} signalAgeCheckTaskParams_t;

extern TaskHandle_t signalAgeCheckTaskHandle;
void signalAgeCheckTask(void* pvParameters);

#endif // SIGNAL_AGE_CHECK_TASK_H