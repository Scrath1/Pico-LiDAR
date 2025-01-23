#include "motor_control_task.h"

#include <ulog.h>

#include "prj_config.h"
#include "spid.h"

#define SIGNAL_LOCK_TIMEOUT (10)

TaskHandle_t motorCtrlTaskHandle;

void motorControlTask(void* pvParameters) {
    ULOG_TRACE("Starting motor control task");
    if(NULL == pvParameters) {
        ULOG_CRITICAL("Failed to retrieve motor control task params");
        configASSERT(false);
    }
    rpm_signal_t& targetRPMSignal = ((motorControlTaskParams_t*)pvParameters)->targetRPMSignal;
    rpm_signal_t& measuredRPMSignal = ((motorControlTaskParams_t*)pvParameters)->measuredRPMSignal;
    runtime_settings_signal_t& rtSettingsSignal = ((motorControlTaskParams_t*)pvParameters)->runtimeSettingsSignal;

    bool rtSettingsSuccess = false;
    uint32_t lastSettingsUpdate = 0;

    taskENTER_CRITICAL();
    runtime_settings_t rtSettings = rtSettingsSignal.read(rtSettingsSuccess, SIGNAL_LOCK_TIMEOUT);
    if(rtSettingsSuccess) {
        lastSettingsUpdate = rtSettingsSignal.getLastUpdateTime_ms();
    }
    taskEXIT_CRITICAL();
    if(!rtSettingsSuccess) {
        ULOG_CRITICAL("Failed to obtain settings in motor control task");
        configASSERT(false);
    }

    // Initialize PID controller
    spid_t pid;
    float kp = rtSettings.pid_controller.kp;
    float ki = rtSettings.pid_controller.ki;
    float kd = rtSettings.pid_controller.kd;
    if(SPID_SUCCESS != spid_init(&pid, kp, ki, kd, PID_MIN_OUT, PID_MAX_OUT, PID_INTERVAL_MS)) {
        ULOG_CRITICAL("Failed to initialize PID controller");
        configASSERT(false);
    }

    TickType_t lastWakeTime = xTaskGetTickCount();
    ULOG_TRACE("Starting motor control task loop");
    for(;;) {
        // check if motor enable switch is on
        uint8_t pwm = 0;
        rpm_data_t measuredRPM = {.rpm = 0};
        rpm_data_t targetRPM = {.rpm = 0};

        // Check if there was an update to the PID settings
        uint32_t updateTime = rtSettingsSignal.getLastUpdateTime_ms();
        if(lastSettingsUpdate != updateTime) {
            ULOG_DEBUG("PID settings have changed. Updating controller");
            bool settingsSuccess = false;
            rtSettings = rtSettingsSignal.read(settingsSuccess, SIGNAL_LOCK_TIMEOUT);
            if(settingsSuccess) {
                spid_set_kp(&pid, rtSettings.pid_controller.kp);
                spid_set_ki(&pid, rtSettings.pid_controller.ki);
                spid_set_kd(&pid, rtSettings.pid_controller.kd);
                lastSettingsUpdate = updateTime;
                ULOG_INFO("Updated PID settings: Kp=%0.3f, Ki=%0.3f, Kd=%0.3f", rtSettings.pid_controller.kp,
                          rtSettings.pid_controller.ki, rtSettings.pid_controller.kd);
            } else {
                ULOG_ERROR("Failed to update PID settings");
            }
        }

        // get current target and measured RPM values
        bool targetRPMSuccess = false;
        targetRPM = targetRPMSignal.read(targetRPMSuccess, SIGNAL_LOCK_TIMEOUT);
        bool measuredRPMSuccess = false;
        uint32_t measuredRPMAge_ms;
        measuredRPM = measuredRPMSignal.read(measuredRPMSuccess, SIGNAL_LOCK_TIMEOUT, measuredRPMAge_ms);
        if(measuredRPMAge_ms > PID_INTERVAL_MS) {
            // if signal is too old, set targetRPM to 0 for safe state
            targetRPM.rpm = 0;
        }

        if(digitalRead(PIN_SWITCH_LEFT) && measuredRPMSuccess && targetRPMSuccess) {
            pwm = (uint8_t)spid_process(&pid, (float)targetRPM.rpm, measuredRPM.rpm);
        }
        // ToDo: Switch to Hardware based PWM
        // PWM_instance.setPWM(PIN_MOTOR_PWM, pwm_frequency, pwm);
        analogWrite(PIN_MOTOR_PWM, pwm);
        Serial.print(">measuredRPM:");
        Serial.println(measuredRPM.rpm);
        Serial.print(">targetRPM:");
        Serial.println(targetRPM.rpm);
        Serial.print(">PWM:");
        Serial.println(pwm);
        Serial.print(">wErr:");
        Serial.println(measuredRPMSignal.writeErrCnt);
        Serial.print(">rErr:");
        Serial.println(measuredRPMSignal.readErrCnt);
        // Preempt task until next loop iteration time
        xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(PID_INTERVAL_MS));
    }
}