#ifndef MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include "signals/signal_types.h"

#define MOTOR_CONTROL_TASK_STACK_SIZE (1024)
#define MOTOR_CONTROL_TASK_NAME ("mtrCtrlTsk")
#define MOTOR_CONTROL_TASK_PRIORITY (configMAX_PRIORITIES-3)

#if (MOTOR_CONTROL_TASK_PRIORITY >= configMAX_PRIORITIES)
    #error "Motor control task priority too high"
#endif

extern TaskHandle_t motorCtrlTaskHandle;
void motorControlTask(void* pvParameters);

#endif // MOTOR_CONTROL_TASK_H