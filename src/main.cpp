#include <Arduino.h>
#include <FreeRTOS.h>
#include <RP2040_PWM.h>
#include <hardware/gpio.h>
#include <pico/stdlib.h>
#include <queue.h>
#include <task.h>
#include <ulog.h>

#include "macros.h"
#include "prj_config.h"
#include "signals/signal_types.h"
#include "spid.h"
#include "tasks/motor_control_task.h"
#include "tasks/signal_age_check_task.h"
#include "averaging_filter.h"

#define SIGNAL_LOCK_TIMEOUT 50
#define TARGET_RPM_MIN_THRESHOLD (60)
#define TARGET_RPM_FILTER_SIZE (16)

AVERAGING_FILTER_DEF(hallIntervalFilter, 4);
AVERAGING_FILTER_DEF(measuredRPMFilter, RPM_AVERAGING_FILTER_SIZE);
AVERAGING_FILTER_DEF(targetRPMFilter, TARGET_RPM_FILTER_SIZE);

rpm_signal_t targetRPMSignal, measuredRPMSignal;
runtime_settings_signal_t rtSettingsSignal;

void consoleLogger(ulog_level_t severity, char* msg) {
    char fmsg[256];
    uint32_t len = snprintf(fmsg, sizeof(fmsg), "%10lu [%s]: %s\n", pdTICKS_TO_MS(xTaskGetTickCount()),
                            ulog_level_name(severity), msg);
    Serial.print(fmsg);
}

void hallSensorISR(uint32_t events, BaseType_t& xHigherPriorityTaskWoken){
    if(events & GPIO_IRQ_EDGE_RISE){
        // signal detection visualization using LED
        digitalWrite(PIN_LED_USER, LED_OFF);
    }
    else if(events & GPIO_IRQ_EDGE_FALL){
        static uint32_t lastPulseTime_us = 0;
        static uint32_t pulseIntervals[PULSES_PER_REV] = {0};
        const uint32_t pulseIntervalsArrSize = sizeof(pulseIntervals)/sizeof(pulseIntervals[0]);
        // points to next index to write to
        static uint32_t pulseIntervalsNextIdx = 0;

        // signal detection using LED
        digitalWrite(PIN_LED_USER, 0);

        // initialize variable before measuring
        if(lastPulseTime_us == 0){
            lastPulseTime_us = time_us_32();
            return;
        }
        // calculate time between the last 2 magnetic pulses
        const uint32_t currentTime_us = time_us_32();
        const uint32_t interval_us = currentTime_us - lastPulseTime_us;
        lastPulseTime_us = currentTime_us;

        // add it to array of pulse times for averaging
        averaging_filter_put(&hallIntervalFilter, interval_us);
        uint32_t averageInterval_us = averaging_filter_get_avg(&hallIntervalFilter);

        // calculate actual axle rotation speed
        // Note: For some reason trying to replace this manual filtering
        // with the averaging filter used above results in extreme spikes in measurements
        // ToDo: Find reason and fix
        static uint32_t rpmMeasurements[RPM_AVERAGING_FILTER_SIZE];
        static const uint32_t rpmMeasurementsSize = sizeof(rpmMeasurements)/sizeof(rpmMeasurements[0]);
        static uint32_t rpmMeasurementsNextIdx = 0;
        uint32_t currentRPMMeasurement = (1/((float)averageInterval_us / 1E6)) * 60 * PULSES_PER_REV;
        rpmMeasurements[rpmMeasurementsNextIdx] = currentRPMMeasurement;
        rpmMeasurementsNextIdx = (rpmMeasurementsNextIdx+1) % rpmMeasurementsNextIdx;

        uint32_t avgRPM = 0;
        for(uint32_t i = 0; i < rpmMeasurementsSize; i++){
            avgRPM += rpmMeasurements[i];
        }
        avgRPM /= rpmMeasurementsSize;

        // and finally update the signal for the measured RPM
        if(!measuredRPMSignal.writeFromISR({.rpm = avgRPM}, &xHigherPriorityTaskWoken)){
            // retry once on failure
            measuredRPMSignal.writeFromISR({.rpm = avgRPM}, &xHigherPriorityTaskWoken);
        }
    }
}

void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(gpio == PIN_HALL_SENSOR) hallSensorISR(events, xHigherPriorityTaskWoken);

    if(xHigherPriorityTaskWoken){
        portYIELD_FROM_ISR(pdTRUE);
    }
}

void setup() {
    // put your setup code here, to run once:

    // Initialize the logger
    ULOG_INIT();
    ULOG_SUBSCRIBE(consoleLogger, ULOG_TRACE_LEVEL);

    // Set up serial
    Serial.setTX(0);
    Serial.setRX(1);
    Serial.begin(115200);

    averaging_filter_init(&hallIntervalFilter);
    averaging_filter_init(&measuredRPMFilter);
    averaging_filter_init(&targetRPMFilter);

    // Pin configuration
    ULOG_TRACE("Configuring pins");
    pinMode(PIN_MOTOR_PWM, OUTPUT);
    pinMode(PIN_POTENTIOMETER, INPUT);
    pinMode(PIN_SWITCH_LEFT, INPUT);
    pinMode(PIN_LED_USER, OUTPUT);
    digitalWrite(PIN_LED_USER, 1);

    gpio_init(PIN_HALL_SENSOR);
    gpio_set_dir(PIN_HALL_SENSOR, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_HALL_SENSOR, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_callback);

    // signals for communicating measured RPM and target RPM between tasks
    ULOG_TRACE("Initializing communication signals");
    measuredRPMSignal.init();
    measuredRPMSignal.write({.rpm = 0}, 0);
    targetRPMSignal.init();
    targetRPMSignal.write({.rpm = 0}, 0);
    rtSettingsSignal.init();
    rtSettingsSignal.write({
        .pid_controller = {
            .kp = K_P,
            .ki = K_I,
            .kd = K_D
        }
    }, 0);

    // create tasks
    ULOG_TRACE("Creating tasks");
    motorControlTaskParams_t mCtrlTaskParams = {
        .targetRPMSignal = targetRPMSignal,
        .measuredRPMSignal = measuredRPMSignal,
        .runtimeSettingsSignal = rtSettingsSignal
    };
    if(pdPASS != xTaskCreate(motorControlTask, MOTOR_CONTROL_TASK_NAME, MOTOR_CONTROL_TASK_STACK_SIZE,
                             (void*)&mCtrlTaskParams, MOTOR_CONTROL_TASK_PRIORITY, &motorCtrlTaskHandle)) {
        ULOG_CRITICAL("Failed to create motor control task");
        assert(false);
    }
    vTaskCoreAffinitySet(motorCtrlTaskHandle, (1<<0));
    signalAgeCheckTaskParams_t sigAgeChkTskParams = {
        .measuredRPMSignal = measuredRPMSignal
    };
    if(pdPASS != xTaskCreate(signalAgeCheckTask, SIGNAL_AGE_CHECK_TASK_NAME,
                             SIGNAL_AGE_CHECK_TASK_STACK_SIZE, (void*)&sigAgeChkTskParams,
                             SIGNAL_AGE_CHECK_TASK_PRIORITY, &signalAgeCheckTaskHandle)) {
        ULOG_CRITICAL("Failed to create signal age check task");
        assert(false);
    }
    vTaskCoreAffinitySet(signalAgeCheckTaskHandle, (1<<0));

    // vTaskDelete(NULL);
    // taskYIELD();
    // This is done in the background by the arduino-pico core already
    // vTaskStartScheduler();
    ULOG_TRACE("Setup finished");
}

void loop() {
    // ToDo: Remove after testing
    volatile uint16_t pot = analogRead(PIN_POTENTIOMETER);
    averaging_filter_put(&targetRPMFilter, map(pot, 0, 1023, 0, MAX_TARGET_RPM));
    uint32_t targetRPM = averaging_filter_get_avg(&targetRPMFilter);
    targetRPMSignal.write({.rpm = targetRPM}, SIGNAL_LOCK_TIMEOUT);
    vTaskDelay(100);
}