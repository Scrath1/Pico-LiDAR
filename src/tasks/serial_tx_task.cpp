#include "serial_tx_task.h"

#include <ulog.h>

#include <cstring>

#include "prj_config.h"
#include "ring_buffer.h"
#include "serial_print.h"

RING_BUFFER_DEF(txBuffer, TX_BUFFER_LEN + 1);
TaskHandle_t serialTxTaskHandle;

SemaphoreHandle_t putStringMutex;

void putString(const char* str, uint32_t strLen) {
    if(pdPASS == xSemaphoreTake(putStringMutex, portMAX_DELAY)){
        for(uint32_t i = 0; i < strLen-1; i++) {
            ring_buffer_put(&txBuffer, str[i], portMAX_DELAY);
        }
        xSemaphoreGive(putStringMutex);
    }
}

void serialTxTask(void* pvParameters) {
    volatile bool* txTaskReady = (bool*)(pvParameters);
    if(RC_SUCCESS != ring_buffer_init(&txBuffer)) {
        ULOG_CRITICAL("Failed to initialize serial tx ringbuffer");
        configASSERT(false);
    }
    putStringMutex = xSemaphoreCreateMutex();
    if(NULL == putStringMutex){
        ULOG_CRITICAL("Failed to create mutex for putString function");
        configASSERT(false);
    }

    *txTaskReady = true;
    ULOG_TRACE("Starting Serial Tx task loop");
    for(;;) {
        char c = 0;
        RC_t err = RC_ERROR;
        // print each byte received from the ringbuffer
        while(RC_SUCCESS == ring_buffer_get(&txBuffer, &c, 10)) {
            SERIAL_PORT.print(c);
        }
        // This task runs on a separate core and thus does not need to sleep
        // It is also not on the maximum priority so if necessary it can be
        // preempted
    }
}