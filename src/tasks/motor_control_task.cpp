#include "motor_control_task.h"

#include <hardware/pwm.h>
#include <ulog.h>

#include "prj_config.h"
#include "spid.h"

TaskHandle_t motorCtrlTaskHandle;

// Copies of settings objects
static setting<float> pid_kp;
static setting<float> pid_ki;
static setting<float> pid_kd;
static setting<uint32_t> targetRPM;
static setting<bool> enableMotor;

void rpmToleranceCheck() {
    const uint32_t tgt = targetRPM.get();
    const uint32_t minMeasuredRPM = tgt - tgt * MEASURED_RPM_TOLERANCE_PERCENT;
    const uint32_t maxMeasuredRPM = tgt + tgt * MEASURED_RPM_TOLERANCE_PERCENT;
    if(minMeasuredRPM <= status.measuredRPM.rpm && status.measuredRPM.rpm <= maxMeasuredRPM) {
        status.stableTargetRPMCount++;
    } else {
        status.stableTargetRPMCount = 0;
    }
    if(status.stableTargetRPMCount >= MEASURED_RPM_STABILITY_INTERVAL_COUNT && !status.stableTargetRPM) {
        status.stableTargetRPM = true;
        ULOG_DEBUG("measured RPM turned stable");
        // rpmWithinToleranceIntervalsCnt may eventually overflow but with
        // it being a uint32_t that should be a long long long time
    } else if(status.stableTargetRPMCount < MEASURED_RPM_STABILITY_INTERVAL_COUNT && status.stableTargetRPM) {
        status.stableTargetRPM = false;
        ULOG_DEBUG("measured RPM turned unstable");
    }
}

void motorControlTask(void* pvParameters) {
    ULOG_TRACE("Starting motor control task");
    // Copies of settings objects
    pid_kp = runtimeSettings.pid_kp;
    pid_ki = runtimeSettings.pid_ki;
    pid_kd = runtimeSettings.pid_kd;
    targetRPM = runtimeSettings.targetRPM;
    enableMotor = runtimeSettings.enableMotor;

    // Initialize PID controller
    spid_t pid;
    if(SPID_SUCCESS != spid_init(&pid, pid_kp.get(), pid_ki.get(), pid_kd.get(), PID_MIN_OUT, PID_MAX_OUT, PID_INTERVAL_MS)) {
        ULOG_CRITICAL("Failed to initialize PID controller");
        configASSERT(false);
    }

    // get channel and slice number of the PWM slice used for the motor pin
    const uint8_t pwmChannelNum = pwm_gpio_to_channel(PIN_MOTOR_PWM);
    const uint8_t pwmSliceNum = pwm_gpio_to_slice_num(PIN_MOTOR_PWM);

    TickType_t lastWakeTime = xTaskGetTickCount();
    ULOG_TRACE("Starting motor control task loop");
    for(;;) {
        // First check for setting updates
        if(pid_kp != runtimeSettings.pid_kp){
            ULOG_INFO("%s changed: %0.3f -> %0.3f", pid_kp.name, pid.k_p, runtimeSettings.pid_kp);
            pid_kp = runtimeSettings.pid_kp;
            spid_set_kp(&pid, pid_kp.get());
        }
        if(pid_ki != runtimeSettings.pid_ki){
            ULOG_INFO("%s changed: %0.3f -> %0.3f", pid_ki.name, pid.k_i, runtimeSettings.pid_ki);
            pid_ki = runtimeSettings.pid_ki;
            spid_set_ki(&pid, pid_ki.get());
        }
        if(pid_kd != runtimeSettings.pid_kd){
            ULOG_INFO("%s changed: %0.3f -> %0.3f", pid_kd.name, pid.k_d, runtimeSettings.pid_kd);
            pid_kd = runtimeSettings.pid_ki;
            spid_set_kd(&pid, pid_kd.get());
        }
        if(enableMotor != runtimeSettings.enableMotor){
            enableMotor = runtimeSettings.enableMotor;
        }

        // measured rpm stability check
        rpmToleranceCheck();
        uint16_t pwm;
        uint32_t pidRPMTarget;
        if(enableMotor.get()){ // if motor enable is on
            pwm = (uint16_t)spid_process(&pid, (float)targetRPM.get(), status.measuredRPM.rpm);
        }
        else{
            spid_process(&pid, (float)0, status.measuredRPM.rpm);
            pwm = 0;
        }
        status.pwmOutputLevel = pwm;
        pwm_set_chan_level(pwmSliceNum, pwmChannelNum, pwm);

        // Preempt task until next loop iteration time
        xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(PID_INTERVAL_MS));
    }
}