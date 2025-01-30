#include "sensor_task.h"

#include <VL53L0X.h>
#include <Wire.h>
#include <ulog.h>

#include "global.h"
#include "prj_config.h"
#include "serial_print.h"

#define VL53L0X_TIMEOUT_MS 50

TaskHandle_t sensorTaskHandle;

// local setting copies
static setting<uint32_t> targetRPM;
static setting<uint32_t> vl53l0xTimingBudget_us;
static setting<uint16_t> dataPointsPerRev;

uint32_t rpmToTimePerRev_ms(uint16_t rpm){
    return (1000/(rpm/60));
}

uint16_t scantimeInterval_ms(uint32_t timePerRev_ms, uint16_t scanpointsPerRev){
    return timePerRev_ms / scanpointsPerRev;
}

uint16_t checkMaxScanpointsPerRev(uint16_t rpm, uint32_t scantimeBudget_ms, uint16_t scanpointsPerRev){
    uint32_t timePerRotation_ms = rpmToTimePerRev_ms(rpm);
    uint32_t timePerScan_ms = 0;
    while(true){
        if(scanpointsPerRev == 0) return 0;
        timePerScan_ms = scantimeInterval_ms(timePerRotation_ms, scanpointsPerRev);
        if(timePerScan_ms < scantimeBudget_ms){
            scanpointsPerRev--;
        }
        else return scanpointsPerRev;
    }
}

void sensorTask(void* pvParameters) {
    ULOG_TRACE("Starting sensor task");

    VL53L0X vl53l0x;
    Wire.begin();
    vl53l0x.setTimeout(VL53L0X_TIMEOUT_MS);
    while(!vl53l0x.init()) {
        ULOG_ERROR("Failed to initialize VL53L0X sensor. Retrying in 1s");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Ranging profiles for VL53L0X
    // +---------------+---------------+---------------------+----------------------------------------+
    // |     Mode      | Timing Budget |  Typical Max Range  |           Typical applicaton           |
    // +---------------+---------------+---------------------+----------------------------------------+
    // | Default       | 30ms          | 1.2m (white target) | standard                               |
    // | High accuracy | 200ms         | 1.2m (white target) | precise measurement                    |
    // | Long range    | 33ms          | 2m (white target)   | long ranging, only for dark conditions |
    // | High Speed    | 20ms          | 1.2m (white target) | high speed, accuracy is no priority    |
    // +---------------+---------------+---------------------+----------------------------------------+
    vl53l0x.setMeasurementTimingBudget(vl53l0xTimingBudget_us.get());  // 20ms -> High Speed profile
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
    for(;;) {
        // Check for setting updates. The local copies are used so that
        // changes in other tasks to the global objects are less likely to mess
        // up some calculations here
        if(targetRPM != runtimeSettings.targetRPM) {
            targetRPM = runtimeSettings.targetRPM;
        }
        if(vl53l0xTimingBudget_us != runtimeSettings.vl53l0xMeasurementTimingBudget_us) {
            vl53l0xTimingBudget_us = runtimeSettings.vl53l0xMeasurementTimingBudget_us;
            vl53l0x.setMeasurementTimingBudget(vl53l0xTimingBudget_us.get());
        }
        if(dataPointsPerRev != runtimeSettings.dataPointsPerRev) {
            dataPointsPerRev = runtimeSettings.dataPointsPerRev;
        }

        // ToDo: Maybe outsource this step to another point
        uint16_t checkedMaxScanpoints = checkMaxScanpointsPerRev(targetRPM.get(), vl53l0xTimingBudget_us.get() / 1000, dataPointsPerRev.get());
        if(checkedMaxScanpoints < dataPointsPerRev.get()){
            runtimeSettings.dataPointsPerRev.set(checkedMaxScanpoints);
            uint32_t rotationPeriod_ms = rpmToTimePerRev_ms(targetRPM.get());
            uint32_t taskInterval_ms = scantimeInterval_ms(rotationPeriod_ms, checkedMaxScanpoints);
            status.sensorTaskInterval_ms = taskInterval_ms;
            ULOG_DEBUG("rpm:%lu, datapoints:%u => intv:%lu", targetRPM.get(), dataPointsPerRev.get(), taskInterval_ms);
        }

        // Actual range reading
        uint16_t range_mm = vl53l0x.readRangeSingleMillimeters();
        // get angle of measurement
        int16_t currentAngle = -1;
        if(status.stableTargetRPM) {
            currentAngle = status.domeAngle.calculateCurrentAngle(targetRPM.get());
        }
        if(vl53l0x.timeoutOccurred()) {
            ULOG_WARNING("VL53L0X read timeout");
        } else {
            serialPrintf(">VL53L0X:%i:%lu|np\n", currentAngle, range_mm);
        }

        ULOG_ALWAYS("sensIntv:%lu", status.sensorTaskInterval_ms);
        if(pdFALSE == xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(status.sensorTaskInterval_ms))) {
            ULOG_WARNING("sensor task timing violation");
        }
    }
}