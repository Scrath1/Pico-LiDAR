#include "signal_age_check_task.h"
#include <ulog.h>

#include "prj_config.h"

#define SIGNAL_LOCK_TIMEOUT 5

TaskHandle_t signalAgeCheckTaskHandle;

void signalAgeCheckTask(void* pvParameters){
    ULOG_TRACE("Starting signal age check task");
    if(NULL == pvParameters){
        ULOG_CRITICAL("Failed to retrieve signal age check task params");
        assert(false);
    }
    rpm_signal_t& measuredRPMSignal = ((signalAgeCheckTaskParams_t*)pvParameters)->measuredRPMSignal;
    
    TickType_t lastWakeTime = xTaskGetTickCount();
    ULOG_TRACE("Starting signal age check task loop");
    for(;;){
        bool measuredRPMSuccess = false;
        uint32_t measuredRPMAge_ms = 0;
        measuredRPMSignal.read(measuredRPMSuccess, SIGNAL_LOCK_TIMEOUT, measuredRPMAge_ms);
        if(measuredRPMSuccess){
            if(measuredRPMAge_ms > SIGNAL_MEASURED_RPM_AGE_THRESHOLD_MS){
                // measuredRPM signal is too old. Reset to 0
                measuredRPMSignal.write({.rpm = 0}, SIGNAL_LOCK_TIMEOUT);
            }
        }

        xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SIGNAL_AGE_CHECK_INTERVAL_MS));
    }
}