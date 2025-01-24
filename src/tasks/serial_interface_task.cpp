#include "serial_interface_task.h"
#include <ulog.h>
#include "prj_config.h"
#include "serial_commands.h"

#include <cstdlib>

#define SIGNAL_LOCK_TIMEOUT_MS 25

#if SERIAL_CMD_INPUT_TIMEOUT_MS < SERIAL_INTERFACE_TASK_INTERVAL_MS
    #error "SERIAL_CMD_INPUT_TIMEOUT_MS should be larger than SERIAL_INTERFACE_TASK_INTERVAL_MS"
#endif

TaskHandle_t serialInterfaceTaskHandle;
rpm_signal_t* targetRPMSignalPtr;
runtime_settings_signal_t* rtSettingsSignalPtr;

void serialInterfaceTask(void* pvParameters){
    ULOG_TRACE("Starting serial interface task");
    if(NULL == pvParameters) {
        ULOG_CRITICAL("Failed to retrieve serial interface task params");
        configASSERT(false);
    }
    rpm_signal_t& measuredRPMSignal = ((serialInterfaceTaskParams_t*)pvParameters)->measuredRPMSignal;
    runtime_settings_signal_t& rtSettingsSignal = ((serialInterfaceTaskParams_t*)pvParameters)->runtimeSettingsSignal;
    rtSettingsSignalPtr = &rtSettingsSignal;

    // Command parser setup
    char serialCommandBuffer[SERIAL_CMD_INPUT_BUFFER_SIZE];

    TickType_t lastWakeTime = xTaskGetTickCount();
    ULOG_TRACE("Starting serial interface task loop");
    char cmdInputBuffer[SERIAL_CMD_INPUT_BUFFER_SIZE];
    // Indicates the number of chars currently in cmdInputBuffer.
    uint32_t fillLevel = 0;
    // set whenever a new character is detected.
    // If older than the current time. If no char was received since
    // last buffer reset, this value should be 0
    uint32_t timestampLastCharDetected_ms = 0;
    for(;;){
        bool cmdDelimDetected = false;
        bool clearBufferFlag = false;

        // Handle incoming data
        while(Serial.available() > 0){
            char c = Serial.read();
            if(fillLevel < SERIAL_CMD_INPUT_BUFFER_SIZE){
                cmdInputBuffer[fillLevel++] = c;
            }
            if(c == CMD_END_DELIMITER){
                if(parseCommand(rtSettingsSignal, (uint8_t*)cmdInputBuffer, fillLevel)){
                    ULOG_INFO("Command executed successfully");
                }
                else{
                    ULOG_ERROR("Failed to parse command");
                }
                // clear buffer
                clearBufferFlag = true;
            }
        }

        // Command timeout check
        if(timestampLastCharDetected_ms != 0){
            uint32_t lastCharAge = xTaskGetTickCount() - timestampLastCharDetected_ms;
            if(lastCharAge > SERIAL_CMD_INPUT_TIMEOUT_MS){
                clearBufferFlag = true;
            }
        }

        // clear command buffer if required
        if(clearBufferFlag){
            memset(cmdInputBuffer, 0, fillLevel);
            fillLevel = 0;
            timestampLastCharDetected_ms = 0;
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SERIAL_INTERFACE_TASK_INTERVAL_MS));
    }
}