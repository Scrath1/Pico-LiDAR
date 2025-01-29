#include "signal_age_check_task.h"
#include <ulog.h>

#include "prj_config.h"
#include "global.h"

#define SIGNAL_LOCK_TIMEOUT 5

TaskHandle_t signalAgeCheckTaskHandle;

void signalAgeCheckTask(void* pvParameters){
    ULOG_TRACE("Starting signal age check task");
    TickType_t lastWakeTime = xTaskGetTickCount();
    ULOG_TRACE("Starting signal age check task loop");
    for(;;){
        if(status.measuredRPM.getAge_ms() > SIGNAL_MEASURED_RPM_AGE_THRESHOLD_MS){
            // measuredRPM signal is too old. Reset to 0
            status.measuredRPM.set(0);
            // also reset the dome angle signal since it is directly
            // dependent on the motor spinning
            status.domeAngle = dome_angle_t{.angleBase = 0, . timeOfAngleIncrement_us = time_us_32()};
        }

        xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SIGNAL_AGE_CHECK_INTERVAL_MS));
    }
}