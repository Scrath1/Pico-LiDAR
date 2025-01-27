#include "sensor_task.h"
#include <ulog.h>

#include <Wire.h>
#include <VL53L0X.h>
#include "prj_config.h"

#define VL53L0X_TIMEOUT_MS (50)
#define TEST_TASK_INTERVAL_TODO_CHANGE (30)

TaskHandle_t sensorTaskHandle;

void sensorTask(void* pvParameters){
    ULOG_TRACE("Starting sensor task");
    if(NULL == pvParameters){
        ULOG_CRITICAL("Failed to retrieve sensor task params");
        configASSERT(false);
    }
    rpm_signal_t& measuredRPMSignal = ((sensorTaskParams_t*)pvParameters)->measuredRPMSignal;
    runtime_settings_signal_t& rtSettingsSignal = ((sensorTaskParams_t*)pvParameters)->runtimeSettingsSignal;
    
    VL53L0X vl53l0x;
    Wire.begin();
    vl53l0x.setTimeout(VL53L0X_TIMEOUT_MS);
    while(!vl53l0x.init()){
        ULOG_ERROR("Failed to initialize VL53L0X sensor. Retrying in 500ms");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // undecided whether to use single shot or continuous mode
    // vl53l0x.startContinuous(measurementPeriod)

    // Ranging profiles for VL53L0X
    // +---------------+---------------+---------------------+----------------------------------------+
    // |     Mode      | Timing Budget |  Typical Max Range  |           Typical applicaton           |
    // +---------------+---------------+---------------------+----------------------------------------+
    // | Default       | 30ms          | 1.2m (white target) | standard                               |
    // | High accuracy | 200ms         | 1.2m (white target) | precise measurement                    |
    // | Long range    | 33ms          | 2m (white target)   | long ranging, only for dark conditions |
    // | High Speed    | 20ms          | 1.2m (white target) | high speed, accuracy is no priority    |
    // +---------------+---------------+---------------------+----------------------------------------+
    vl53l0x.setMeasurementTimingBudget(20000); // 20ms -> High Speed profile
    // Operating modes
    // 1. Single ranging: Only one measurement is taken
    // 2. Continuous: Back to back measuring as fast as possible
    // 3. Timed: Back to back measuring with a fixed delay between measurements

    // ToDo:
    // 1. Determine maximum number of measurement points per rotation based on
    //  rotation speed and time required for measuring
    // 2. Calculate task trigger intervals
    TickType_t lastWakeTime = xTaskGetTickCount();
    ULOG_TRACE("Starting sensor task loop");
    for(;;){
        uint16_t range_mm = vl53l0x.readRangeSingleMillimeters();
        if(vl53l0x.timeoutOccurred()){
            ULOG_WARNING("VL53L0X read timeout");
        }
        else{
                SERIAL_PORT.print(">VL53L0X:");
                SERIAL_PORT.println(range_mm);
                SERIAL_PORT.print(">VL53L0X_angle:");
                SERIAL_PORT.println(currentAngle);
        }

        if(pdFALSE == xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TEST_TASK_INTERVAL_TODO_CHANGE))){
            ULOG_WARNING("sensor task timing violation");
        }
    }
}