#include <Arduino.h>
#include <FreeRTOS.h>
#include <RP2040_PWM.h>
#include <hardware/gpio.h>
#include <pico/stdlib.h>
#include <task.h>
#include <ulog.h>

#include "global.h"
#include "macros.h"
#include "prj_config.h"
#include "spid.h"
#include "tasks/motor_control_task.h"

#define TARGET_RPM_MIN_THRESHOLD (60)
#define TARGET_RPM_FILTER_SIZE (16)

struct repeating_timer rpmDecayTimer;

void consoleLogger(ulog_level_t severity, char* msg) {
    char fmsg[256];
    uint32_t len = snprintf(fmsg, sizeof(fmsg), "%10lu [%s]: %s\n", pdTICKS_TO_MS(xTaskGetTickCount()),
                            ulog_level_name(severity), msg);
    Serial.print(fmsg);
}

bool rpmDecayLoop(struct repeating_timer* t) {
    uint32_t currentTime_us = time_us_32();
    uint32_t timeSinceUpdate_us = currentTime_us - lastRPMUpdateTime_us;
    if(timeSinceUpdate_us > expectedRPMUpdateInterval_us * 4) {
        measuredRPM *= RPM_DECAY_PERCENTAGE;
    }
    return true;
}

void gpio_callback(uint gpio, uint32_t events) {
    if(gpio == PIN_HALL_SENSOR && events & GPIO_IRQ_EDGE_RISE) {
        // signal detection using LED
        digitalWrite(PIN_LED_USER, 1);
    } else if(gpio == PIN_HALL_SENSOR && events & GPIO_IRQ_EDGE_FALL) {
        static uint32_t lastPulseTime_us = 0;
        static uint32_t pulseIntervals[PULSES_PER_REV] = {0};
        const uint32_t pulseIntervalsArrSize = sizeof(pulseIntervals) / sizeof(pulseIntervals[0]);
        // points to next index to write to
        static uint32_t pulseIntervalsNextIdx = 0;

        // signal detection using LED
        digitalWrite(PIN_LED_USER, 0);

        // initialize variable before measuring
        if(lastPulseTime_us == 0) {
            lastPulseTime_us = time_us_32();
            return;
        }
        // calculate time between the last 2 magnetic pulses
        const uint32_t currentTime_us = time_us_32();
        const uint32_t interval_us = currentTime_us - lastPulseTime_us;
        lastPulseTime_us = currentTime_us;

        // add it to array of pulse times for averaging
        pulseIntervals[pulseIntervalsNextIdx] = interval_us;
        pulseIntervalsNextIdx = (pulseIntervalsNextIdx + 1) % pulseIntervalsArrSize;

        // average pulse times
        uint32_t averageInterval_us = 0;
        for(uint32_t i = 0; i < pulseIntervalsArrSize; i++) {
            averageInterval_us += pulseIntervals[i];
        }
        averageInterval_us /= pulseIntervalsArrSize;

        // calculate actual axle rotation speed
        static uint32_t rpmMeasurements[RPM_AVERAGING_FILTER_SIZE];
        static const uint32_t rpmMeasurementsSize = sizeof(rpmMeasurements) / sizeof(rpmMeasurements[0]);
        static uint32_t rpmMeasurementsNextIdx = 0;
        uint32_t currentRPMMeasurement = (1 / ((float)averageInterval_us / 1E6)) * 60 * PULSES_PER_REV;
        rpmMeasurements[rpmMeasurementsNextIdx] = currentRPMMeasurement;
        rpmMeasurementsNextIdx = (rpmMeasurementsNextIdx + 1) % rpmMeasurementsNextIdx;

        uint32_t averagedRPM = 0;
        for(uint32_t i = 0; i < rpmMeasurementsSize; i++) {
            averagedRPM += rpmMeasurements[i];
        }
        averagedRPM /= rpmMeasurementsSize;
        measuredRPM = averagedRPM;
        lastRPMUpdateTime_us = currentTime_us;
        expectedRPMUpdateInterval_us = averageInterval_us;
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
    delay(2000);

    // Pin configuration
    pinMode(PIN_MOTOR_PWM, OUTPUT);
    pinMode(PIN_POTENTIOMETER, INPUT);
    pinMode(PIN_SWITCH_LEFT, INPUT);
    pinMode(PIN_LED_USER, OUTPUT);
    digitalWrite(PIN_LED_USER, 1);

    gpio_init(PIN_HALL_SENSOR);
    gpio_set_dir(PIN_HALL_SENSOR, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_HALL_SENSOR, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_callback);

    if(!add_repeating_timer_ms(RPM_DECAY_CHECK_INTERVAL_S * 1000, rpmDecayLoop, NULL, &rpmDecayTimer)) {
        Serial.println("Failed to create rpm decay loop timer");
        while(1);
    }

    // create tasks
    if(pdTRUE != xTaskCreate(motorControlTask, MOTOR_CONTROL_TASK_NAME, MOTOR_CONTROL_TASK_STACK_SIZE, NULL,
                             MOTOR_CONTROL_TASK_PRIORITY, &motorCtrlTaskHandle)) {
        ULOG_CRITICAL("Failed to create motor control task");
    }

    ULOG_DEBUG("Starting scheduler");
    vTaskStartScheduler();
    // Function should never return
    ULOG_CRITICAL("Scheduler crashed");
    for(;;);
}

void loop() {
    // put your main code here, to run repeatedly:
    uint16_t pot = analogRead(PIN_POTENTIOMETER);
    static uint16_t targetRPMFilter[TARGET_RPM_FILTER_SIZE];
    static uint32_t targetRPMFilterNextIdx = 0;
    targetRPMFilter[targetRPMFilterNextIdx] = map(pot, 0, 1023, 0, 1000);
    targetRPMFilterNextIdx = (targetRPMFilterNextIdx + 1) % ARRAY_SIZE(targetRPMFilter);
    uint32_t averageTargetRPM = 0;
    for(uint32_t i = 0; i < ARRAY_SIZE(targetRPMFilter); i++) {
        averageTargetRPM += targetRPMFilter[i];
    }
    averageTargetRPM /= ARRAY_SIZE(targetRPMFilter);
    if(averageTargetRPM < TARGET_RPM_MIN_THRESHOLD) {
        targetRPM = 0;
    } else {
        targetRPM = averageTargetRPM;
    }

    sleep_ms(100);

    Serial.print(">targetRPM:");
    Serial.println(targetRPM);
    Serial.print(">measuredRPM:");
    Serial.println(measuredRPM);
}