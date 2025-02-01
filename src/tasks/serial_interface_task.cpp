#include "serial_interface_task.h"

#include <ulog.h>

#include <cstdlib>

#include "global.h"
#include "prj_config.h"
#include "serial_commands.h"
#include "serial_print.h"

#if SERIAL_CMD_INPUT_TIMEOUT_MS < SERIAL_INTERFACE_TASK_INTERVAL_MS
    #error "SERIAL_CMD_INPUT_TIMEOUT_MS should be larger than SERIAL_INTERFACE_TASK_INTERVAL_MS"
#endif

#define TASK_LOG_NAME ("SerIntfTsk")

TaskHandle_t serialInterfaceTaskHandle;

void serialInterfaceTask(void* pvParameters) {
    ULOG_TRACE("%s: Starting", TASK_LOG_NAME);

    // Command parser setup
    char serialCommandBuffer[SERIAL_CMD_INPUT_BUFFER_SIZE];

    TickType_t lastWakeTime = xTaskGetTickCount();
    ULOG_TRACE("%s: Starting loop", TASK_LOG_NAME);
    char cmdInputBuffer[SERIAL_CMD_INPUT_BUFFER_SIZE];
    // Indicates the number of chars currently in cmdInputBuffer.
    uint32_t fillLevel = 0;
    // set whenever a new character is detected.
    // If older than the current time. If no char was received since
    // last buffer reset, this value should be 0
    uint32_t timestampLastCharDetected_ms = 0;
    // Timestamp when plotting data was last transmitted
    uint32_t timestampLastPlotDataTansmission_ms = 0;
    for(;;) {
        bool cmdDelimDetected = false;
        bool clearBufferFlag = false;
        uint32_t currentTime_ms = xTaskGetTickCount();

        // Handle incoming data
        while(SERIAL_PORT.available() > 0) {
            char c = SERIAL_PORT.read();
            if(fillLevel < SERIAL_CMD_INPUT_BUFFER_SIZE) {
                cmdInputBuffer[fillLevel++] = c;
            }
            if(c == CMD_END_DELIMITER) {
                if(parseCommand((uint8_t*)cmdInputBuffer, fillLevel)) {
                    ULOG_INFO("%s: Command executed successfully", TASK_LOG_NAME);
                } else {
                    ULOG_ERROR("%s: Failed to parse command", TASK_LOG_NAME);
                }
                // clear buffer
                memset(cmdInputBuffer, 0, fillLevel);
                fillLevel = 0;
                timestampLastCharDetected_ms = 0;
            }
        }

        // Command timeout check
        if(timestampLastCharDetected_ms != 0) {
            uint32_t lastCharAge = currentTime_ms - timestampLastCharDetected_ms;
            if(lastCharAge > SERIAL_CMD_INPUT_TIMEOUT_MS) {
                memset(cmdInputBuffer, 0, fillLevel);
                fillLevel = 0;
                timestampLastCharDetected_ms = 0;
            }
        }

        // Transmit runtime data for plotting at fixed interval
        if(currentTime_ms - timestampLastPlotDataTansmission_ms > SERIAL_INTERFACE_PLOT_OUTPUT_INTERVAL_MS) {
            serialPrintf(">%s:%lu\n", runtimeSettings.targetRPM.name, runtimeSettings.targetRPM.get());
            serialPrintf(">measuredRPM:%lu\n", status.measuredRPM.rpm);
            serialPrintf(">PWM:%lu\n", status.pwmOutputLevel / (PWM_FULL_VALUE / 100));

            // update timestamp of last transmission
            timestampLastPlotDataTansmission_ms = currentTime_ms;
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SERIAL_INTERFACE_TASK_INTERVAL_MS));
    }
}