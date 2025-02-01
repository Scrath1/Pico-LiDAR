#ifndef SERIAL_INTERFACE_TASK_H
#define SERIAL_INTERFACE_TASK_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#define SERIAL_INTERFACE_TASK_STACK_SIZE (1024)
#define SERIAL_INTERFACE_TASK_NAME ("serInterfaceTsk")
#define SERIAL_INTERFACE_TASK_PRIORITY (configMAX_PRIORITIES - 4)

#if (SERIAL_INTERFACE_TASK_PRIORITY >= configMAX_PRIORITIES)
    #error "Serial interface task priority too high"
#endif

extern TaskHandle_t serialInterfaceTaskHandle;
void serialInterfaceTask(void* pvParameters);

#endif  // SERIAL_INTERFACE_TASK_H