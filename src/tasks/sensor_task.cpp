#include "sensor_task.h"
#include <ulog.h>

#include <Wire.h>
#include <VL53L0X.h>
#include "prj_config.h"

#define VL53L0X_TIMEOUT_MS (50)
#define TEST_TASK_INTERVAL_TODO_CHANGE (30)

#define SIGNAL_LOCK_TIME_RELAXED (50)
#define SIGNAL_LOCK_TIME_CRITICAL (2)

TaskHandle_t sensorTaskHandle;

void sensorTask(void* pvParameters){
    ULOG_TRACE("Starting sensor task");
    if(NULL == pvParameters){
        ULOG_CRITICAL("Failed to retrieve sensor task params");
        configASSERT(false);
    }
    rpm_signal_t& measuredRPMSignal = ((sensorTaskParams_t*)pvParameters)->measuredRPMSignal;
    runtime_settings_signal_t& rtSettingsSignal = ((sensorTaskParams_t*)pvParameters)->runtimeSettingsSignal;
    dome_angle_signal_t& domeAngleSignal = ((sensorTaskParams_t*)pvParameters)->domeAngleSignal;
    VL53L0X vl53l0x;
    Wire.begin();
    vl53l0x.setTimeout(VL53L0X_TIMEOUT_MS);
    while(!vl53l0x.init()){
        ULOG_ERROR("Failed to initialize VL53L0X sensor. Retrying in 1s");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    bool rtsSuccess = false;
    runtime_settings_t rts;
    uint32_t lastRtsUpdate_ms = 0;
    while(!rtsSuccess){
        rts = rtSettingsSignal.read(rtsSuccess, SIGNAL_LOCK_TIME_RELAXED);
        lastRtsUpdate_ms = rtSettingsSignal.getLastUpdateTime_ms();
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
        if(lastRtsUpdate_ms != rtSettingsSignal.getLastUpdateTime_ms()){
            rts = rtSettingsSignal.read(rtsSuccess, SIGNAL_LOCK_TIME_CRITICAL);
        }

        uint16_t range_mm = vl53l0x.readRangeSingleMillimeters();
        // get angle of measurement
        bool success = false;
        angle_position_t anglePos = domeAngleSignal.read(success, SIGNAL_LOCK_TIME_CRITICAL);
        int16_t currentAngle = -1;
        if(rts.stableTargetRPM){
            currentAngle = anglePos.calculateCurrentAngle(rts.pid_controller.targetRPM);
        }
        if(success){
            if(vl53l0x.timeoutOccurred()){
                ULOG_WARNING("VL53L0X read timeout");
            }
            else{
                SERIAL_PORT.print(">VL53L0X:");
                SERIAL_PORT.print(currentAngle);
                SERIAL_PORT.print(":");
                SERIAL_PORT.print(range_mm);
                SERIAL_PORT.println("|np"); // flag for teleplot to not plot this value
            }
        }

        if(pdFALSE == xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TEST_TASK_INTERVAL_TODO_CHANGE))){
            ULOG_WARNING("sensor task timing violation");
        }
    }
}