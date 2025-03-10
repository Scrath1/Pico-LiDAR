#include <Arduino.h>
#include <FreeRTOS.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>
#include <task.h>
#include <ulog.h>

#include "averaging_filter.h"
#include "global.h"
#include "macros.h"
#include "prj_config.h"
#include "serial_print.h"
#include "spid.h"
#include "tasks/motor_control_task.h"
#include "tasks/sensor_task.h"
#include "tasks/serial_interface_task.h"
#include "tasks/serial_tx_task.h"
#include "tasks/signal_age_check_task.h"

AVERAGING_FILTER_DEF(measuredRPMFilter, RPM_AVERAGING_FILTER_SIZE);

void consoleLogger(ulog_level_t severity, char* msg) {
    char fmsg[DBG_MESSAGE_MAX_LEN];
    uint32_t len = snprintf(fmsg, sizeof(fmsg), "%10lu [%s]: %s\n", pdTICKS_TO_MS(xTaskGetTickCount()),
                            ulog_level_name(severity), msg);
    serialPrint(fmsg, len + 1);
}

void hallSensorISR(uint32_t events) {
    if(events & GPIO_IRQ_EDGE_FALL) {
        static uint32_t lastPulseTime_us = 0;
        static uint32_t pulseIntervals[PULSES_PER_REV] = {0};
        const uint32_t pulseIntervalsArrSize = sizeof(pulseIntervals) / sizeof(pulseIntervals[0]);
        // points to next index to write to
        static uint32_t pulseIntervalsNextIdx = 0;

        // immediately acquire timestamp of pulse
        const uint32_t currentTime_us = time_us_32();

        // read
        dome_angle_t da = status.domeAngle;
        // modify
        uint16_t newAngleBase;
        if(!gpio_get(PIN_ZERO_HALL_SENSOR)) {
            // Zero angle detected
            newAngleBase = 0;
        } else {
            newAngleBase = (da.angleBase + (360 / PULSES_PER_REV)) % 360;
        }
        dome_angle_t newAngPos = {.angleBase = newAngleBase, .timeOfAngleIncrement_us = currentTime_us};
        // write back
        status.domeAngle = newAngPos;

        // initialize variable before measuring
        if(lastPulseTime_us == 0) {
            lastPulseTime_us = time_us_32();
            return;
        }
        // calculate time between the last 2 magnetic pulses
        const uint32_t interval_us = currentTime_us - lastPulseTime_us;
        lastPulseTime_us = currentTime_us;

        // calculate actual axle rotation speed
        static uint32_t rpmMeasurements[RPM_AVERAGING_FILTER_SIZE];
        static const uint32_t rpmMeasurementsSize = sizeof(rpmMeasurements) / sizeof(rpmMeasurements[0]);
        static uint32_t rpmMeasurementsNextIdx = 0;
        // Step 1. Calculate pulse interval in seconds instead of microsends
        float calc = static_cast<float>(interval_us) / 1E6;
        // Step 2. Multiply with amount of magnets per full rotation
        calc *= PULSES_PER_REV;
        // Step 3. Calculate frequency of rotation (Rotations per second)
        calc = 1 / calc;
        // Step 4. Calculate RPM from rotation frequency
        const uint32_t currentRPMMeasurement = calc * 60;

        // Average RPM and apply the averaged value to the measuredRPM status signal
        averaging_filter_put(&measuredRPMFilter, currentRPMMeasurement);
        status.measuredRPM.setFromISR(averaging_filter_get_avg(&measuredRPMFilter));
    }
}

void pushBtnLeftISR() {
    static uint32_t lastTriggerTime_ms = 0;
    const uint32_t currentTime_ms = pdTICKS_TO_MS(xTaskGetTickCountFromISR());
    if(currentTime_ms - lastTriggerTime_ms > BTN_DEBOUNCE_MS) {
        // button isn't bouncing (anymore)
        runtimeSettings.enableMotor.set(!runtimeSettings.enableMotor.get());
    }
    lastTriggerTime_ms = currentTime_ms;
}

void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(gpio == PIN_SPEED_HALL_SENSOR)
        hallSensorISR(events);
    else if(gpio == PIN_PUSHBTN)
        pushBtnLeftISR();
    else if(gpio == PIN_ECHO){
        if(events & GPIO_IRQ_EDGE_FALL) hc_sr04_isr(xHigherPriorityTaskWoken, false);
        else hc_sr04_isr(xHigherPriorityTaskWoken, true);
    }

    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR(pdTRUE);
    }
}

void configurePins() {
    gpio_init(PIN_LED_USER);
    gpio_set_dir(PIN_LED_USER, GPIO_OUT);
    gpio_set_drive_strength(PIN_LED_USER, GPIO_DRIVE_STRENGTH_12MA);
    gpio_put(PIN_LED_USER, LED_OFF);

    gpio_init(PIN_SPEED_HALL_SENSOR);
    gpio_set_dir(PIN_SPEED_HALL_SENSOR, GPIO_IN);
    // Hall effect sensors already have a pull-up resistor soldered to them
    gpio_set_irq_enabled_with_callback(PIN_SPEED_HALL_SENSOR, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true,
                                       gpio_callback);

    gpio_init(PIN_ZERO_HALL_SENSOR);
    gpio_set_dir(PIN_ZERO_HALL_SENSOR, GPIO_IN);
    // Hall effect sensors already have a pull-up resistor soldered to them

    gpio_init(PIN_PUSHBTN);
    gpio_set_pulls(PIN_PUSHBTN, true, false);
    gpio_set_dir(PIN_PUSHBTN, GPIO_IN);
    // only first gpio irq to be configured has to use gpio_set_irq_enabled_with_callback
    gpio_set_irq_enabled(PIN_PUSHBTN, GPIO_IRQ_EDGE_FALL, true);

    gpio_init(PIN_TRIG);
    gpio_set_pulls(PIN_TRIG, false, true);
    gpio_set_dir(PIN_TRIG, GPIO_OUT);

    gpio_init(PIN_ECHO);
    gpio_set_pulls(PIN_ECHO, false, true);
    gpio_set_dir(PIN_ECHO, GPIO_IN);
    gpio_set_irq_enabled(PIN_ECHO, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);

    gpio_set_function(PIN_MOTOR_PWM, GPIO_FUNC_PWM);
    uint8_t pwmSlice = pwm_gpio_to_slice_num(PIN_MOTOR_PWM);
    uint8_t pwmChan = pwm_gpio_to_channel(PIN_MOTOR_PWM);
    pwm_set_wrap(pwmSlice, PWM_FULL_VALUE);
    pwm_set_chan_level(pwmSlice, pwmChan, 0);
    pwm_set_enabled(pwmSlice, true);
}

void setup() {
    // put your setup code here, to run once:

    // Initialize the logger
    ULOG_INIT();
    ULOG_SUBSCRIBE(consoleLogger, ULOG_TRACE_LEVEL);

    // Set up serial
    SERIAL_PORT.setTX(0);
    SERIAL_PORT.setRX(1);
    SERIAL_PORT.begin(115200);

    averaging_filter_init(&measuredRPMFilter);

    // Pin configuration
    ULOG_TRACE("Setup: Configuring pins");
    configurePins();

    // create tasks
    ULOG_TRACE("Setup: Creating tasks");
    ULOG_TRACE("Setup: Switching to Tx task for serial output");
    volatile bool txTaskReady = false;
    if(pdPASS != xTaskCreateAffinitySet(serialTxTask, SERIAL_TX_TASK_NAME, SERIAL_TX_TASK_STACK_SIZE,
                                        (void*)(&txTaskReady), SERIAL_TX_TASK_PRIORITY, SERIAL_TX_TASK_CORE_MASK,
                                        &serialTxTaskHandle)) {
        ULOG_CRITICAL("Setup: Failed to create serial Tx task");
        configASSERT(false);
    }
    while(!txTaskReady) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    useTxTask = true;
    ULOG_TRACE("Setup: Switch to Tx task done");

    if(pdPASS != xTaskCreate(motorControlTask, MOTOR_CONTROL_TASK_NAME, MOTOR_CONTROL_TASK_STACK_SIZE, NULL,
                             MOTOR_CONTROL_TASK_PRIORITY, &motorCtrlTaskHandle)) {
        ULOG_CRITICAL("Setup: Failed to create motor control task");
        configASSERT(false);
    }
    vTaskCoreAffinitySet(motorCtrlTaskHandle, (1 << 0));

    if(pdPASS != xTaskCreate(signalAgeCheckTask, SIGNAL_AGE_CHECK_TASK_NAME, SIGNAL_AGE_CHECK_TASK_STACK_SIZE, NULL,
                             SIGNAL_AGE_CHECK_TASK_PRIORITY, &signalAgeCheckTaskHandle)) {
        ULOG_CRITICAL("Setup: Failed to create signal age check task");
        configASSERT(false);
    }
    vTaskCoreAffinitySet(signalAgeCheckTaskHandle, (1 << 0));
    if(pdPASS != xTaskCreate(serialInterfaceTask, SERIAL_INTERFACE_TASK_NAME, SERIAL_INTERFACE_TASK_STACK_SIZE, NULL,
                             SERIAL_INTERFACE_TASK_PRIORITY, &serialInterfaceTaskHandle)) {
        ULOG_CRITICAL("Setup: Failed to create serial interface task");
        configASSERT(false);
    }
    vTaskCoreAffinitySet(signalAgeCheckTaskHandle, (1 << 0));
    if(pdPASS != xTaskCreate(sensorTask, SENSOR_TASK_NAME,
                             SENSOR_TASK_STACK_SIZE, NULL,
                             SENSOR_TASK_PRIORITY, &sensorTaskHandle)) {
        ULOG_CRITICAL("Setup: Failed to create sensor task");
        configASSERT(false);
    }
    vTaskCoreAffinitySet(sensorTaskHandle, (1<<0));

    ULOG_TRACE("Setup: finished");
    vTaskDelete(NULL);
    taskYIELD();
    // This is done in the background by the arduino-pico core already
    // vTaskStartScheduler();
}

void loop() {}