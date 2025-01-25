#include "motor_control_task.h"

#include <ulog.h>

#include "prj_config.h"
#include "spid.h"
#include <hardware/pwm.h>

#define SIGNAL_LOCK_TIMEOUT (10)

TaskHandle_t motorCtrlTaskHandle;

void motorControlTask(void* pvParameters) {
    ULOG_TRACE("Starting motor control task");
    if(NULL == pvParameters) {
        ULOG_CRITICAL("Failed to retrieve motor control task params");
        configASSERT(false);
    }
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

    // get channel and slice number of the PWM slice used for the motor pin
    const uint8_t pwmChannelNum = pwm_gpio_to_channel(PIN_MOTOR_PWM);
    const uint8_t pwmSliceNum = pwm_gpio_to_slice_num(PIN_MOTOR_PWM);

    TickType_t lastWakeTime = xTaskGetTickCount();
    ULOG_TRACE("Starting motor control task loop");
    for(;;) {
        // check if motor enable switch is on
        uint8_t pwm = 0;
        rpm_data_t measuredRPM = {.rpm = 0};
        rpm_data_t targetRPM = {.rpm = 0};

        // Check if there was an update to any settings
        uint32_t updateTime = rtSettingsSignal.getLastUpdateTime_ms();
        if(lastSettingsUpdate != updateTime) {
            bool settingsSuccess = false;
            rtSettings = rtSettingsSignal.read(settingsSuccess, SIGNAL_LOCK_TIMEOUT);
            if(settingsSuccess) {
                spid_set_kp(&pid, rtSettings.pid_controller.kp);
                spid_set_ki(&pid, rtSettings.pid_controller.ki);
                spid_set_kd(&pid, rtSettings.pid_controller.kd);
                lastSettingsUpdate = updateTime;
                ULOG_INFO("Motor task: Received settings update");
            } else {
                ULOG_ERROR("Motor task: Failed to update settings");
            }
        }

        // get measured RPM values
        bool measuredRPMSuccess = false;
        uint32_t measuredRPMAge_ms;
        measuredRPM = measuredRPMSignal.read(measuredRPMSuccess, SIGNAL_LOCK_TIMEOUT, measuredRPMAge_ms);
        if(measuredRPMAge_ms > PID_INTERVAL_MS) {
            // if signal is too old, set targetRPM to 0 for safe state
            targetRPM.rpm = 0;
        }

        if(digitalRead(PIN_SWITCH_LEFT) && rtSettings.enableMotor && measuredRPMSuccess) {
            pwm = (uint8_t)spid_process(&pid, (float)rtSettings.pid_controller.targetRPM, measuredRPM.rpm);
        }
        pwm_set_chan_level(pwmSliceNum, pwmChannelNum, pwm);

        // Serial.print(">measuredRPM:");
        // Serial.println(measuredRPM.rpm);
        // Serial.print(">targetRPM:");
        // Serial.println(targetRPM.rpm);
        // Serial.print(">PWM:");
        // Serial.println(pwm);
        // Serial.print(">wErr:");
        // Serial.println(measuredRPMSignal.writeErrCnt);
        // Serial.print(">rErr:");
        // Serial.println(measuredRPMSignal.readErrCnt);
        // Preempt task until next loop iteration time
        xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(PID_INTERVAL_MS));
    }
}