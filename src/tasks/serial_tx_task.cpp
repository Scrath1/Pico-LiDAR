#include "serial_tx_task.h"

#include <ulog.h>

#include "prj_config.h"
#include "ring_buffer.h"
#include <cstring>
#include "serial_print.h"

RING_BUFFER_DEF(txBuffer, TX_BUFFER_LEN+1);
TaskHandle_t serialTxTaskHandle;

bool putString(const char* str, uint32_t strLen){
    uint32_t remainingSpace = TX_BUFFER_LEN - ring_buffer_avail(&txBuffer);
    // if(true){
    if(remainingSpace >= strLen){
        for(uint32_t i = 0; i < strLen; i++){
            ring_buffer_put(&txBuffer, (uint8_t)str[i]);
        }
        return true;
    }
    else
        return false;
}

void serialTxTask(void* pvParameters) {
    (void)pvParameters;
    ring_buffer_init(&txBuffer);

    for(;;) {
        char msg[DBG_MESSAGE_MAX_LEN] = "";
        char c = 0;
        RC_t err = RC_ERROR;
        uint32_t i = 0;
        // get bytes from ring buffer until a full string was extracted,
        // as determined by the null terminator
        while(RC_SUCCESS == ring_buffer_get(&txBuffer, (uint8_t*)&c)){
            Serial.print(c);
            // msg[i++] = c;
            // if(c == '\0'){
            //     // print the string
            //     SERIAL_PORT.print(msg);
            //     break;
            // }
        }
        // This is a low priority task that should run in the background when
        // there is nothing else to be done. As such, no sleep interval
        // is specified.
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}