#include "sensor_task.h"

#include <VL53L0X.h>
#include <Wire.h>
#include <queue.h>
#include <ulog.h>

#include "global.h"
#include "prj_config.h"
#include "serial_print.h"

#define VL53L0X_TIMEOUT_MS 50

#define SPEED_OF_SOUND_MPS (343)
#define HC_SR04_TRIG_PULSE_DURATION_US (10)

#define TASK_LOG_NAME ("SensorTsk")

TaskHandle_t sensorTaskHandle;

// local setting copies
static setting<uint32_t> targetRPM;
static setting<uint32_t> vl53l0xTimingBudget_us;
static setting<uint16_t> dataPointsPerRev;
static setting<int16_t> angleOffset;

// Queue for sending measured pulse duration of HC-SR04 sensor in us
// to the actual sensor task
QueueHandle_t hc_sr04_pulseDurationQueue;

uint32_t rpmToTimePerRev_ms(uint16_t rpm) { return (1000 / (rpm / 60)); }

// Calculates the minimum time to set aside for sampling at each datapoint
// Based on the timing budget of the sensors + some configurable tolerance
// (SENSOR_SCAN_EXTRA_TIME_BUDGET_MS)
uint32_t getMinSampleTime_ms(uint32_t vl53l0x_budget_ms) {
    return vl53l0x_budget_ms + SENSOR_SCAN_EXTRA_TIME_BUDGET_MS + HC_SR04_TIME_BUDGET_MS;
}

// calculate the maximum time that can ideally be spent for each sample point.
// Based on the current target RPM and the number of sample points
uint32_t getMaxSampleTime_ms(uint16_t rpm, uint32_t samplePoints) { return (rpmToTimePerRev_ms(rpm)) / samplePoints; }

uint16_t checkMaxScanpointsPerRev(uint16_t rpm, uint32_t samplePoints, uint32_t vl53l0x_budget_ms) {
    // reduce the number of sample points until the minimum time required to sample
    // is below the maximum time available for each range sample
    while(true) {
        if(samplePoints == 0) return 0;
        uint32_t t_min = getMinSampleTime_ms(vl53l0x_budget_ms);
        uint32_t t_max = getMaxSampleTime_ms(rpm, samplePoints);
        if(t_min > t_max)
            samplePoints--;
        else
            break;
    }
    return samplePoints;
}

void hc_sr04_isr(BaseType_t& xHigherPriorityTaskWoken, bool risingEdge) {
    static uint32_t timeOfRisingEdge_us = 0;
    if(risingEdge) {
        timeOfRisingEdge_us = time_us_32();
    } else {
        if(timeOfRisingEdge_us != 0) {
            uint32_t currentTime_us = time_us_32();
            uint32_t duration = currentTime_us - timeOfRisingEdge_us;
            xQueueSendFromISR(hc_sr04_pulseDurationQueue, &duration, &xHigherPriorityTaskWoken);
        }
    }
}

int64_t reset_trig_pin(alarm_id_t id, void* user_data) {
    (void)user_data;
    gpio_put(PIN_TRIG, false);
    return 1;
}

void sensorTask(void* pvParameters) {
    ULOG_TRACE("%s: Starting", TASK_LOG_NAME);

    VL53L0X vl53l0x;
    Wire.begin();
    vl53l0x.setTimeout(VL53L0X_TIMEOUT_MS);
    bool vl53l0xInitialized = false;
    for(uint32_t i = 0; i < VL53L0X_MAX_CONNECTION_ATTEMPTS; i++) {
        if(vl53l0x.init()) {
            ULOG_INFO("%s: VL53L0X initialized", TASK_LOG_NAME);
            vl53l0xInitialized = true;
            break;
        } else {
            ULOG_ERROR("%s: Attempt %lu failed to initialize VL53L0X sensor. Retrying in 1s", TASK_LOG_NAME, i + 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if(!vl53l0xInitialized){
        ULOG_WARNING("%s: Could not initialize VL53L0X. Ignoring that sensor", TASK_LOG_NAME);
    }

    hc_sr04_pulseDurationQueue = xQueueCreate(4, sizeof(uint32_t));
    if(NULL == hc_sr04_pulseDurationQueue) {
        ULOG_CRITICAL("%s: Failed to create HC-SR04 pulse duration queue", TASK_LOG_NAME);
        configASSERT(false);
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
    // Operating modes
    // 1. Single ranging: Only one measurement is taken
    // 2. Continuous: Back to back measuring as fast as possible
    // 3. Timed: Back to back measuring with a fixed delay between measurements

    TickType_t lastWakeTime = xTaskGetTickCount();
    ULOG_TRACE("%s: Starting loop", TASK_LOG_NAME);
    for(;;) {
        // Check for setting updates. The local copies are used so that
        // changes in other tasks to the global objects are less likely to mess
        // up some calculations here
        if(targetRPM != runtimeSettings.targetRPM) {
            targetRPM = runtimeSettings.targetRPM;
        }
        if(vl53l0xTimingBudget_us != runtimeSettings.vl53l0xMeasurementTimingBudget_us) {
            vl53l0xTimingBudget_us = runtimeSettings.vl53l0xMeasurementTimingBudget_us;
            if(vl53l0xInitialized) {
                vl53l0x.setMeasurementTimingBudget(vl53l0xTimingBudget_us.get());
            }
        }
        if(dataPointsPerRev != runtimeSettings.dataPointsPerRev) {
            dataPointsPerRev = runtimeSettings.dataPointsPerRev;
        }
        if(angleOffset != runtimeSettings.angleOffset) {
            angleOffset = runtimeSettings.angleOffset;
        }

        // ToDo: Maybe outsource this step to another point
        uint16_t checkedMaxScanpoints =
            checkMaxScanpointsPerRev(targetRPM.get(), dataPointsPerRev.get(), vl53l0xTimingBudget_us.get() / 1000);
        if(checkedMaxScanpoints < dataPointsPerRev.get()) {
            runtimeSettings.dataPointsPerRev.set(checkedMaxScanpoints);
            uint32_t rotationPeriod_ms = rpmToTimePerRev_ms(targetRPM.get());
            uint32_t taskInterval_ms = rotationPeriod_ms / checkedMaxScanpoints;
            status.sensorTaskInterval_ms = taskInterval_ms;
            ULOG_WARNING("%s: Sample rate unattainable. Reduced scanpoints to %u", TASK_LOG_NAME, checkedMaxScanpoints);
        }

        // Actual range reading
        // 1. Trigger the HC-SR04 ultrasonic sensor
        gpio_put(PIN_TRIG, true);
        // 2. Set callback alarm to reset trigger after its trigger interval
        sleep_us(HC_SR04_TRIG_PULSE_DURATION_US);
        gpio_put(PIN_TRIG, false);
        // add_alarm_in_us(HC_SR04_TRIG_PULSE_DURATION_US, reset_trig_pin, NULL, true);

        // 2. Read the VL53L0X laser sensor
        uint16_t vl53l0x_range_mm = 0;
        if(vl53l0xInitialized) vl53l0x_range_mm = vl53l0x.readRangeSingleMillimeters();

        // 3. Wait for result of HC-SR04
        uint32_t hc_sr04_duration_us = 0;
        uint32_t hc_sr04_range_mm = 0;
        BaseType_t hc_sr04Success = xQueueReceive(hc_sr04_pulseDurationQueue, &hc_sr04_duration_us, pdMS_TO_TICKS(HC_SR04_TIME_BUDGET_MS));
        if(pdTRUE == hc_sr04Success) {
            // The received duration of the pulse is equal to the time between sending out a ultrasonic pulse and
            // receiving its echo in microseconds. As such, the formula to calculate the distance from this value is d =
            // speedOfSound(cm/us) * time(us) / 2 Technically the speed of sound also depends on temperature but we
            // don't have a temperature sensor to compensate for that
            const float v_sound_cm_us = 0.0001 * SPEED_OF_SOUND_MPS;
            float d_cm = v_sound_cm_us * hc_sr04_duration_us / 2;
            hc_sr04_range_mm = static_cast<uint32_t>(d_cm * 10);
        }
        // get angle of measurement
        int32_t currentAngle = -1;
        if(status.stableTargetRPM) {
            uint16_t cangle = status.domeAngle.calculateCurrentAngle(targetRPM.get(), angleOffset.get());
            currentAngle = static_cast<int32_t>(cangle);
            // serialPrintf(">Angle: %u\n", cangle);
        }
        if(vl53l0xInitialized) {
            if(vl53l0x.timeoutOccurred()) {
                ULOG_WARNING("%s: VL53L0X read timeout", TASK_LOG_NAME);
            } else {
                serialPrintf(">VL53L0X:%i:%lu|np\n", currentAngle, vl53l0x_range_mm);
            }
        }
        if(pdTRUE == hc_sr04Success){
            serialPrintf(">HC-SR04:%i:%lu|np\n", currentAngle, hc_sr04_range_mm);
        }

        // ULOG_ALWAYS("sensIntv:%lu", status.sensorTaskInterval_ms);
        if(pdFALSE == xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(status.sensorTaskInterval_ms))) {
            ULOG_WARNING("%s: Timing violation", TASK_LOG_NAME);
        }
    }
}