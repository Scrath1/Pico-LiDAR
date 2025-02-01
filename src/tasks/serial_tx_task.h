#ifndef SERIAL_TX_TASK_H
#define SERIAL_TX_TASK_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#include "prj_config.h"

#define SERIAL_TX_TASK_STACK_SIZE (256)
#define SERIAL_TX_TASK_NAME ("serTxTsk")
// this priority is quite high but since this task runs on
// its own core it works
#define SERIAL_TX_TASK_PRIORITY (configMAX_PRIORITIES - 4)
// Only run on core 1, as indicated by bit 1 being set
#define SERIAL_TX_TASK_CORE_MASK (1 << 1)

#if (SERIAL_TX_TASK_PRIORITY >= configMAX_PRIORITIES)
    #error "Serial Tx task priority too high"
#endif

extern TaskHandle_t serialTxTaskHandle;

void putString(const char* str, uint32_t strLen);

void serialTxTask(void* pvParameters);

#endif  // SERIAL_TX_TASK_H