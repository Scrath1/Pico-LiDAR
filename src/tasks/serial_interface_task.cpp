#include "serial_interface_task.h"

#include <ulog.h>

#include <cstdlib>

#include "global.h"
#include "prj_config.h"
#include "serial_commands.h"
#include "serial_print.h"
#include "macros.h"

#define TARGET_BYTES_START_INDEX (2)
#define VALUE_BYTES_START_INDEX (6)

#if SERIAL_CMD_INPUT_TIMEOUT_MS < SERIAL_INTERFACE_TASK_INTERVAL_MS
    #error "SERIAL_CMD_INPUT_TIMEOUT_MS should be larger than SERIAL_INTERFACE_TASK_INTERVAL_MS"
#endif

#define TASK_LOG_NAME ("SerIntfTsk")

TaskHandle_t serialInterfaceTaskHandle;

void serialInterfaceTask(void* pvParameters) {
    ULOG_TRACE("%s: Starting", TASK_LOG_NAME);
    TickType_t lastWakeTime = xTaskGetTickCount();
    ULOG_TRACE("%s: Starting loop", TASK_LOG_NAME);

    // Timestamp when plotting data was last transmitted
    uint32_t timestampLastPlotDataTansmission_ms = 0;

    typedef enum {
        RX_IDLE = 0,     // Next byte to be received should be the start delimiter
        RX_CMD = 1,      // Next char should be the command byte
        RX_ID = 2,       // Next 4 bytes should be the target ID
        RX_VALUE = 3,    // Next 4 bytes should be the value to set
        RX_COMPLETE = 4  // Rx completed. Execute command
    } state_t;
    state_t rxCmdState = RX_IDLE;
    cmd_instruction_t cmd = CMD_NONE;
    uint8_t idBytes[4] = {0};
    uint8_t valueBytes[4] = {0};
    // Counts the number of bytes received for the current command, excluding the
    // start delimiter byte.
    // => cmd byte is at count=0, target is from 1..4 and value from 5..8
    uint32_t byteCounter = 0;
    uint32_t cmdStartTimestamp_ms = 0;
    for(;;) {
        // Handle incoming data
        while(SERIAL_PORT.available() > 0) {
            char c = SERIAL_PORT.read();
            if(rxCmdState != RX_IDLE)
                byteCounter++;
            else
                byteCounter = 0;
            // ULOG_TRACE("%s: b=%u, bcnt=%lu, rxState=%u", TASK_LOG_NAME, c, byteCounter, rxCmdState);

            switch(rxCmdState) {
                default:
                case RX_IDLE:
                    if(COMMAND_START_BYTE == (uint8_t)c){
                        rxCmdState = RX_CMD;
                        cmdStartTimestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount());
                        ULOG_DEBUG("%s: Cmd start byte detected", TASK_LOG_NAME);
                        // increment once to account for the byte being reset earlier
                    }
                    else{
                        ULOG_TRACE("%s: Unknown byte: %u", (uint8_t)c);
                    }
                    break;
                case RX_CMD:
                    cmd = (cmd_instruction_t)c;
                    rxCmdState = RX_ID;
                    break;
                case RX_ID:
                    // put bytes into the idBytes array and check byteCounter
                    // until 4 bytes were received, then transition to value state
                    if(byteCounter >= TARGET_BYTES_START_INDEX && byteCounter < VALUE_BYTES_START_INDEX) {
                        idBytes[byteCounter - TARGET_BYTES_START_INDEX] = (uint8_t)c;
                        if(byteCounter == VALUE_BYTES_START_INDEX - 1){
                            rxCmdState = RX_VALUE;
                            // ULOG_TRACE("Switched to RX_VALUE");
                        }
                    } else {
                        ULOG_ERROR("%s: Failed to parse ID bytes. Invalid byte counter", TASK_LOG_NAME);
                        rxCmdState = RX_IDLE;
                        memset(idBytes, 0, ARRAY_SIZE(idBytes));
                        memset(valueBytes, 0, ARRAY_SIZE(valueBytes));
                        cmdStartTimestamp_ms = 0;
                    }
                    break;
                case RX_VALUE:
                    if(byteCounter >= VALUE_BYTES_START_INDEX && byteCounter < VALUE_BYTES_START_INDEX + ARRAY_SIZE(valueBytes)) {
                        valueBytes[byteCounter - VALUE_BYTES_START_INDEX] = (uint8_t)c;
                        if(byteCounter == VALUE_BYTES_START_INDEX + ARRAY_SIZE(valueBytes) - 1){
                            rxCmdState = RX_COMPLETE;
                            // ULOG_TRACE("Switched to RX_COMPLETE");
                        }
                    } else {
                        ULOG_ERROR("%s: Failed to parse value bytes. Invalid byte counter", TASK_LOG_NAME);
                        rxCmdState = RX_IDLE;
                        memset(idBytes, 0, ARRAY_SIZE(idBytes));
                        memset(valueBytes, 0, ARRAY_SIZE(valueBytes));
                        cmdStartTimestamp_ms = 0;
                    }
                    break;
            }

            if(rxCmdState == RX_COMPLETE){
                if(parseCommand(cmd, idBytes, valueBytes)) {
                    ULOG_INFO("%s: Command executed successfully", TASK_LOG_NAME);
                } else {
                    ULOG_ERROR("%s: Failed to parse command", TASK_LOG_NAME);
                }
                memset(idBytes, 0, ARRAY_SIZE(idBytes));
                memset(valueBytes, 0, ARRAY_SIZE(valueBytes));
                rxCmdState = RX_IDLE;
                cmdStartTimestamp_ms = 0;
            }
        }

        uint32_t currentTime_ms = pdTICKS_TO_MS(xTaskGetTickCount());

        // Command timeout check
        if(cmdStartTimestamp_ms != 0 && currentTime_ms - cmdStartTimestamp_ms > SERIAL_CMD_INPUT_TIMEOUT_MS){
            memset(idBytes, 0, ARRAY_SIZE(idBytes));
            memset(valueBytes, 0, ARRAY_SIZE(valueBytes));
            rxCmdState = RX_IDLE;
            cmdStartTimestamp_ms = 0;
            ULOG_DEBUG("%s: Command timeout detected", TASK_LOG_NAME);
        }

        // Transmit runtime data for plotting at fixed interval
        if(currentTime_ms - timestampLastPlotDataTansmission_ms > SERIAL_INTERFACE_PLOT_OUTPUT_INTERVAL_MS) {
            serialPrintf(">%s:%lu\n", runtimeSettings.targetRPM.name, runtimeSettings.targetRPM.get());
            serialPrintf(">measuredRPM:%lu\n", status.measuredRPM.rpm);
            serialPrintf(">PWM:%0.1f\n", (float)status.pwmOutputLevel / (PWM_FULL_VALUE / 100));

            // update timestamp of last transmission
            timestampLastPlotDataTansmission_ms = currentTime_ms;
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SERIAL_INTERFACE_TASK_INTERVAL_MS));
    }
}