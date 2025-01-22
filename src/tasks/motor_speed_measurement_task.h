#ifndef MOTOR_SPEED_MEASUREMENT_TASK_H
#define MOTOR_SPEED_MEASUREMENT_TASK_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "signals/signal_types.h"

#define MOTOR_SPEED_MEASUREMENT_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define MOTOR_SPEED_MEASUREMENT_TASK_NAME ("mtrSpdMeasTsk")
#define MOTOR_SPEED_MEASUREMENT_TASK_PRIORITY (configMAX_PRIORITIES-2)

#if (MOTOR_SPEED_MEASUREMENT_TASK_PRIORITY >= configMAX_PRIORITIES)
    #error "Motor control task priority too high"
#endif

extern TaskHandle_t motorSpeedMeasurementTaskHandle;
extern QueueHandle_t hallSensorIntervalTimeQueue;
void motorSpeedMeasurementTask(void* pvParameters);

#endif // MOTOR_SPEED_MEASUREMENT_TASK_H