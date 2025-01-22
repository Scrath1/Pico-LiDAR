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
#include "tasks/motor_speed_measurement_task.h"
#include "averaging_filter.h"

#define SIGNAL_LOCK_TIMEOUT 50
#define TARGET_RPM_MIN_THRESHOLD (60)
#define TARGET_RPM_FILTER_SIZE (16)
AVERAGING_FILTER_DEF(targetRPMFilter, TARGET_RPM_FILTER_SIZE);

QueueHandle_t hallSensorIntervalTimeQueue;
rpm_signal_t targetRPMSignal, measuredRPMSignal;

void consoleLogger(ulog_level_t severity, char* msg) {
    char fmsg[256];
    uint32_t len = snprintf(fmsg, sizeof(fmsg), "%10lu [%s]: %s\n", pdTICKS_TO_MS(xTaskGetTickCount()),
                            ulog_level_name(severity), msg);
    Serial.print(fmsg);
}

void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(gpio == PIN_HALL_SENSOR && events & GPIO_IRQ_EDGE_RISE) {
        // signal detection using LED
        digitalWrite(PIN_LED_USER, LED_OFF);
    } else if(gpio == PIN_HALL_SENSOR && events & GPIO_IRQ_EDGE_FALL) {
        if(NULL != hallSensorIntervalTimeQueue){
            static uint32_t lastTriggerTime_us = 0;
            if(lastTriggerTime_us == 0){
                lastTriggerTime_us = time_us_32();
                return;
            }
            else{
                uint32_t currentTime_us = time_us_32();
                uint32_t interval_us = currentTime_us - lastTriggerTime_us;
                xQueueSendFromISR(hallSensorIntervalTimeQueue, &interval_us, &xHigherPriorityTaskWoken);
                digitalWrite(PIN_LED_USER, LED_ON);
                lastTriggerTime_us = currentTime_us;
            }
        }

    }
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

    // Pin configuration
    pinMode(PIN_MOTOR_PWM, OUTPUT);
    pinMode(PIN_POTENTIOMETER, INPUT);
    pinMode(PIN_SWITCH_LEFT, INPUT);
    pinMode(PIN_LED_USER, OUTPUT);
    digitalWrite(PIN_LED_USER, 1);

    gpio_init(PIN_HALL_SENSOR);
    gpio_set_dir(PIN_HALL_SENSOR, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_HALL_SENSOR, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_callback);

    // Used to send the trigger time (in us) from the gpio interrupt to the
    // motor speed measurement task. Data type is uint32_t.
    hallSensorIntervalTimeQueue = xQueueCreate(HALL_TRIGGER_TIME_QUEUE_LEN, sizeof(uint32_t));
    if(NULL == hallSensorIntervalTimeQueue) {
        ULOG_CRITICAL("Failed to create hallSensorIntervalTimeQueue");
        configASSERT(false);
    }
    // signals for communicating measured RPM and target RPM between tasks
    measuredRPMSignal.init();
    measuredRPMSignal.write({.rpm = 0}, 0);
    targetRPMSignal.init();
    targetRPMSignal.write({.rpm = 0}, 0);

    // create tasks
    motorControlTaskParams_t mCtrlTaskParams = {
        .targetRPMSignal = targetRPMSignal,
        .measuredRPMSignal = measuredRPMSignal
    };

    if(pdPASS != xTaskCreate(motorControlTask, MOTOR_CONTROL_TASK_NAME, MOTOR_CONTROL_TASK_STACK_SIZE,
                             (void*)&mCtrlTaskParams, MOTOR_CONTROL_TASK_PRIORITY, &motorCtrlTaskHandle)) {
        ULOG_CRITICAL("Failed to create motor control task");
        assert(false);
    }
    motorSpeedMeasurementTaskParams_t mSpdMeasParams = {
        .targetRPMSignal = targetRPMSignal,
        .measuredRPMSignal = measuredRPMSignal,
        .hallSensorIntervalTimeQueue = hallSensorIntervalTimeQueue
    };

    if(pdPASS != xTaskCreate(motorSpeedMeasurementTask, MOTOR_SPEED_MEASUREMENT_TASK_NAME,
                             MOTOR_SPEED_MEASUREMENT_TASK_STACK_SIZE, (void*)&mSpdMeasParams,
                             MOTOR_SPEED_MEASUREMENT_TASK_PRIORITY, &motorSpeedMeasurementTaskHandle)) {
        ULOG_CRITICAL("Failed to create motor speed measurement task");
        assert(false);
    }

    // vTaskDelete(NULL);
    // taskYIELD();
    averaging_filter_init(&targetRPMFilter);
    // This is done in the background by the arduino-pico core already
    // vTaskStartScheduler();
}

void loop() {
    // ToDo: Remove after testing
    volatile uint16_t pot = analogRead(PIN_POTENTIOMETER);
    averaging_filter_put(&targetRPMFilter, map(pot, 0, 1023, 0, MAX_TARGET_RPM));
    uint32_t targetRPM = averaging_filter_get_avg(&targetRPMFilter);
    targetRPMSignal.write({.rpm = targetRPM}, SIGNAL_LOCK_TIMEOUT);
    vTaskDelay(100);
}