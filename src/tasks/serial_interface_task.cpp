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
    // Timestamp when plotting data was last transmitted
    uint32_t timestampLastPlotDataTansmission_ms = 0;
    for(;;){
        bool cmdDelimDetected = false;
        bool clearBufferFlag = false;
        uint32_t currentTime_ms = xTaskGetTickCount();

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
                memset(cmdInputBuffer, 0, fillLevel);
                fillLevel = 0;
                timestampLastCharDetected_ms = 0;
            }
        }

        // Command timeout check
        if(timestampLastCharDetected_ms != 0){
            uint32_t lastCharAge = currentTime_ms - timestampLastCharDetected_ms;
            if(lastCharAge > SERIAL_CMD_INPUT_TIMEOUT_MS){
                memset(cmdInputBuffer, 0, fillLevel);
                fillLevel = 0;
                timestampLastCharDetected_ms = 0;
            }
        }

        // Transmit runtime data for plotting at fixed interval
        if(currentTime_ms - timestampLastPlotDataTansmission_ms > SERIAL_INTERFACE_PLOT_OUTPUT_INTERVAL_MS){
            bool successRtsRead = false;
            bool successMeasuredRPMRead = false;
            runtime_settings_t rts = rtSettingsSignal.read(successRtsRead, SIGNAL_LOCK_TIMEOUT_MS);
            rpm_data_t measuredRPM = measuredRPMSignal.read(successMeasuredRPMRead, SIGNAL_LOCK_TIMEOUT_MS);
            if(successRtsRead && successMeasuredRPMRead){
                Serial.print(">targetRPM:");
                Serial.println(rts.pid_controller.targetRPM);
                Serial.print(">measuredRPM:");
                Serial.println(measuredRPM.rpm);
                // Serial.print(">PWM:");
                // Serial.println(pwm);
                Serial.print(">wErr:");
                Serial.println(measuredRPMSignal.writeErrCnt);
                Serial.print(">rErr:");
                Serial.println(measuredRPMSignal.readErrCnt);
                // update timestamp of last transmission
                timestampLastPlotDataTansmission_ms = currentTime_ms;
            }
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SERIAL_INTERFACE_TASK_INTERVAL_MS));
    }
}