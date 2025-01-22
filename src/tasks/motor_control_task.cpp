#include "motor_control_task.h"

#include <ulog.h>

#include "global.h"
#include "prj_config.h"
#include "spid.h"

#define SIGNAL_LOCK_TIMEOUT (10)

TaskHandle_t motorCtrlTaskHandle;

void motorControlTask(void* pvParameters) {
    if(NULL == pvParameters){
        ULOG_CRITICAL("Failed to retrieve motor control task params");
        assert(false);
    }
    rpm_signal_t& measuredRPMSignal = ((motorControlTaskParams_t*)pvParameters)->measuredRPMSignal;

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
        if(digitalRead(PIN_SWITCH_LEFT)) {
            bool success = false;
            measuredRPM = measuredRPMSignal.read(success, SIGNAL_LOCK_TIMEOUT);
            pwm = (uint8_t)spid_process(&pid, (float)targetRPM, measuredRPM.rpm);
        }
        // PWM_instance.setPWM(PIN_MOTOR_PWM, pwm_frequency, pwm);
        analogWrite(PIN_MOTOR_PWM, pwm);
        Serial.print(">measuredRPM:");
        Serial.println(measuredRPM.rpm);
        Serial.print(">targetRPM:");
        Serial.println(targetRPM);
        Serial.print(">PWM:");
        Serial.println(pwm);
        

        // Preempt task until next loop iteration time
        xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(PID_INTERVAL_MS));
    }
}