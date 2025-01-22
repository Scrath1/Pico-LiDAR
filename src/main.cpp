#include <Arduino.h>
#include <FreeRTOS.h>
#include <RP2040_PWM.h>
#include <hardware/gpio.h>
#include <pico/stdlib.h>
#include <queue.h>
#include <task.h>
#include <ulog.h>

#include "global.h"
#include "macros.h"
#include "prj_config.h"
#include "signals/signal_types.h"
#include "spid.h"
#include "tasks/motor_control_task.h"
#include "tasks/motor_speed_measurement_task.h"
#include "averaging_filter.h"

#define TARGET_RPM_MIN_THRESHOLD (60)
#define TARGET_RPM_FILTER_SIZE (16)

AVERAGING_FILTER_DEF(targetRPMFilter, TARGET_RPM_FILTER_SIZE);

void consoleLogger(ulog_level_t severity, char* msg) {
    char fmsg[256];
    uint32_t len = snprintf(fmsg, sizeof(fmsg), "%10lu [%s]: %s\n", pdTICKS_TO_MS(xTaskGetTickCount()),
                            ulog_level_name(severity), msg);
    Serial.print(fmsg);
}

QueueHandle_t hallSensorTriggerTimeQueue;

void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(gpio == PIN_HALL_SENSOR && events & GPIO_IRQ_EDGE_RISE) {
        // signal detection using LED
        digitalWrite(PIN_LED_USER, LED_OFF);
    } else if(gpio == PIN_HALL_SENSOR && events & GPIO_IRQ_EDGE_FALL) {
        if(NULL != hallSensorTriggerTimeQueue){
            uint32_t currentTime_us = time_us_32();
            xQueueSendFromISR(hallSensorTriggerTimeQueue, &currentTime_us, &xHigherPriorityTaskWoken);
            digitalWrite(PIN_LED_USER, LED_ON);
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

    // Used to send the trigger time (in us) from the gpio interrupt to the
    // motor speed measurement task. Data type is uint32_t.
    hallSensorTriggerTimeQueue = xQueueCreate(HALL_TRIGGER_TIME_QUEUE_LEN, sizeof(uint32_t));
    if(NULL == hallSensorTriggerTimeQueue) {
        ULOG_CRITICAL("Failed to create hallSensorTriggerTimeQueue");
        configASSERT(false);
    }
    // This buffer is used to send messages of the type rpm_message_t from the motor speed measuring task
    // to the motor control task. It is used as a LIFO buffer, so the motor control task
    // will only read the latest message and discard the rest. In case the buffer is full while trying
    // to send data, the oldest entry is deleted to make room for a new one
    rpm_signal_t measuredRPMSignal;
    measuredRPMSignal.init();
    measuredRPMSignal.write({.rpm = 0}, 0);

    // create tasks
    motorControlTaskParams_t mCtrlTaskParams = {.measuredRPMSignal = measuredRPMSignal};
    if(pdPASS != xTaskCreate(motorControlTask, MOTOR_CONTROL_TASK_NAME, MOTOR_CONTROL_TASK_STACK_SIZE,
                             (void*)&mCtrlTaskParams, MOTOR_CONTROL_TASK_PRIORITY, &motorCtrlTaskHandle)) {
        ULOG_CRITICAL("Failed to create motor control task");
        assert(false);
    }
    motorSpeedMeasurementTaskParams_t mSpdMeasParams = {
        .measuredRPMSignal = measuredRPMSignal,
        .hallSensorTriggerTimeQueue = hallSensorTriggerTimeQueue
    };
    if(pdPASS != xTaskCreate(motorSpeedMeasurementTask, MOTOR_SPEED_MEASUREMENT_TASK_NAME,
                             MOTOR_SPEED_MEASUREMENT_TASK_STACK_SIZE, (void*)&mSpdMeasParams,
                             MOTOR_SPEED_MEASUREMENT_TASK_PRIORITY, &motorSpeedMeasurementTaskHandle)) {
        ULOG_CRITICAL("Failed to create motor speed measurement task");
        assert(false);
    }
    targetRPM = 700;
    vTaskDelete(NULL);
    taskYIELD();
    averaging_filter_init(&targetRPMFilter);
    // This is done in the background by the arduino-pico core already
    // vTaskStartScheduler();
}



void loop() {
    // put your main code here, to run repeatedly:
    uint16_t pot = analogRead(PIN_POTENTIOMETER);
    averaging_filter_put(&targetRPMFilter, map(pot, 0, 1023, 0, MAX_TARGET_RPM));
    targetRPM = averaging_filter_get_avg(&targetRPMFilter);

    sleep_ms(100);
}