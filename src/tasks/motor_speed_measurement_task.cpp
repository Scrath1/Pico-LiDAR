#include "motor_speed_measurement_task.h"

#include <ulog.h>

#include "averaging_filter.h"
#include "prj_config.h"
#include "globals.h"

#define QUEUE_TIMEOUT_MS (0)
#define SIGNAL_LOCK_TIMEOUT (5)

// ToDo: Remove
#define TARGET_RPM_MIN_THRESHOLD (60)
#define TARGET_RPM_FILTER_SIZE (16)

TaskHandle_t motorSpeedMeasurementTaskHandle;
QueueHandle_t hallSensorIntervalTimeQueue;

void motorSpeedMeasurementTask(void* pvParameters) {
    (void)pvParameters;

    // ToDo: Remove targetRPM stuff. This is just for testing
    AVERAGING_FILTER_DEF(targetRPMFilter, TARGET_RPM_FILTER_SIZE);
    AVERAGING_FILTER_DEF(rpmFilter, RPM_AVERAGING_FILTER_SIZE);
    averaging_filter_init(&rpmFilter);
    // IMPORTANT: The pulse times here are not related to the FreeRTOS tick counter
    // but are measured using a separate hardware timer

    uint32_t lastInterval_us = 0;
    TickType_t lastWakeTime = xTaskGetTickCount();
    for(;;) {
        // indicates whether any data was received during this task loop
        bool receivedDataFlag = false;
        // Process all intervals received since last task execution
        uint32_t interval_us = 0;
        while(pdTRUE ==
              xQueueReceive(hallSensorIntervalTimeQueue, &interval_us, pdMS_TO_TICKS(QUEUE_TIMEOUT_MS))) {
            // from interval, determine the expected rotations per minute
            const uint32_t rpm = (1 / ((float)interval_us / 1E6)) * 60 * PULSES_PER_REV;
            // and add the calculated rpm to a filter
            averaging_filter_put(&rpmFilter, rpm);

            // For the decay check
            lastInterval_us = interval_us;
            receivedDataFlag = true;
        }

        if(receivedDataFlag) {
            // If data was received this loop
            // Get average RPM from filter and apply it to the measured RPM signal
            const uint32_t avgRPM = averaging_filter_get_avg(&rpmFilter);
            measuredRPMSignal.write({.rpm = avgRPM}, SIGNAL_LOCK_TIMEOUT);
        } else {
            // No data was received. Check age of last rpm measurement to decide
            // on whether to slowly reduce the measured rpm value
            // The threshold for slowing down is a full expected rotation period
            // (based from the last measured interval) passing without new data
            const uint32_t currentOSTime_ms = pdTICKS_TO_MS(xTaskGetTickCount());
            const uint32_t signalAge_ms = currentOSTime_ms - measuredRPMSignal.updateTimestamp_ms;
            if((lastInterval_us / 1E6) * PULSES_PER_REV < signalAge_ms) {
                // apply decay to signal
                rpm_data_t d = {.rpm = static_cast<uint32_t>(measuredRPMSignal.data.rpm * RPM_DECAY_PERCENTAGE)};
                measuredRPMSignal.write(d, SIGNAL_LOCK_TIMEOUT);
            }
        }

        // ToDo: Remove. This is just for testing
        volatile uint16_t pot = analogRead(PIN_POTENTIOMETER);
        // averaging_filter_put(&targetRPMFilter, map(pot, 0, 1023, 0, MAX_TARGET_RPM));
        targetRPMSignal.write({.rpm = (uint32_t)map(pot, 0, 1023, 0, MAX_TARGET_RPM)}, SIGNAL_LOCK_TIMEOUT);
        // bool ignore;
        // Serial.print(">msmTargetRPM:");
        // Serial.println(targetRPMSignal.read(ignore, 0).rpm);
        // targetRPM = averaging_filter_get_avg(&targetRPMFilter);
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(MOTOR_SPEED_MEASUREMENT_TASK_INTERVAL_MS));
    }
}