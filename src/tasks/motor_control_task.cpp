#include "motor_control_task.h"

#include <ulog.h>

#include "prj_config.h"
#include "spid.h"
#include "globals.h"

#define SIGNAL_LOCK_TIMEOUT (10)

TaskHandle_t motorCtrlTaskHandle;

void motorControlTask(void* pvParameters) {
    (void)pvParameters;

    // Initialize PID controller
    spid_t pid;
    if(SPID_SUCCESS != spid_init(&pid, K_P, K_I, K_D, PID_MIN_OUT, PID_MAX_OUT, PID_INTERVAL_MS)) {
        ULOG_CRITICAL("Failed to initialize PID controller");
        configASSERT(false);
    }

    TickType_t lastWakeTime = xTaskGetTickCount();
    for(;;) {
        // check if motor enable switch is on
        uint8_t pwm = 0;
        rpm_data_t measuredRPM = {.rpm = 0};
        rpm_data_t targetRPM = {.rpm = 0};
        if(digitalRead(PIN_SWITCH_LEFT)) {
            bool measuredRPMSuccess = false;
            measuredRPM = measuredRPMSignal.read(measuredRPMSuccess, SIGNAL_LOCK_TIMEOUT);
            bool targetRPMSuccess = false;
            targetRPM = targetRPMSignal.read(targetRPMSuccess, SIGNAL_LOCK_TIMEOUT);
            if(measuredRPMSuccess && targetRPMSuccess){
                pwm = (uint8_t)spid_process(&pid, (float)targetRPM.rpm, measuredRPM.rpm);
            }
        }
        // PWM_instance.setPWM(PIN_MOTOR_PWM, pwm_frequency, pwm);
        analogWrite(PIN_MOTOR_PWM, pwm);
        // Serial.print(">measuredRPM:");
        // Serial.println(measuredRPM.rpm);
        Serial.print(">targetRPM:");
        Serial.println(targetRPM.rpm);
        // Serial.print(">PWM:");
        // Serial.println(pwm);

        // Preempt task until next loop iteration time
        xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(PID_INTERVAL_MS));
    }
}