#include "motor_control_task.h"

#include <ulog.h>

#include "global.h"
#include "prj_config.h"
#include "spid.h"

TaskHandle_t motorCtrlTaskHandle;

void motorControlTask(void* pvParameters) {
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
        if(digitalRead(PIN_SWITCH_LEFT)) {
            pwm = (uint8_t)spid_process(&pid, targetRPM, measuredRPM);
        }
        // PWM_instance.setPWM(PIN_MOTOR_PWM, pwm_frequency, pwm);
        analogWrite(PIN_MOTOR_PWM, pwm);
        Serial.print(">PWM:");
        Serial.println(pwm);

        // Preempt task until next loop iteration time
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(lastWakeTime));
    }
}