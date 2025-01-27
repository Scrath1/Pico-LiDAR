#include <Arduino.h>
#include <FreeRTOS.h>
#include <hardware/gpio.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <queue.h>
#include <task.h>
#include <ulog.h>

#include "macros.h"
#include "prj_config.h"
#include "signals/signal_types.h"
#include "spid.h"
#include "tasks/motor_control_task.h"
#include "tasks/signal_age_check_task.h"
#include "tasks/serial_interface_task.h"
#include "tasks/sensor_task.h"
#include "averaging_filter.h"

#define SIGNAL_LOCK_TIMEOUT 50
#define TARGET_RPM_MIN_THRESHOLD (60)
#define TARGET_RPM_FILTER_SIZE (16)

AVERAGING_FILTER_DEF(hallIntervalFilter, 4);
AVERAGING_FILTER_DEF(measuredRPMFilter, RPM_AVERAGING_FILTER_SIZE);
AVERAGING_FILTER_DEF(targetRPMFilter, TARGET_RPM_FILTER_SIZE);

rpm_signal_t measuredRPMSignal;
runtime_settings_signal_t rtSettingsSignal;
dome_angle_signal_t domeAngleSignal;

void consoleLogger(ulog_level_t severity, char* msg) {
    char fmsg[256];
    uint32_t len = snprintf(fmsg, sizeof(fmsg), "%10lu [%s]: %s\n", pdTICKS_TO_MS(xTaskGetTickCount()),
                            ulog_level_name(severity), msg);
    SERIAL_PORT.print(fmsg);
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

        // immediately acquire timestamp of pulse
        const uint32_t currentTime_us = time_us_32();

        // signal detection using LED
        digitalWrite(PIN_LED_USER, 0);

        // counter for pulses which previously couldn't be written back for some reason
        static uint32_t domeAngleRWFails = 0;
        // update base angle for angle position determination
        bool daSuccess = false;
        angle_position_t da = domeAngleSignal.readFromISR(daSuccess, &xHigherPriorityTaskWoken);
        if(daSuccess){
            const uint16_t anglePulseIncrement = 360 / PULSES_PER_REV;
            uint16_t newAngleBase = (da.angleBase + anglePulseIncrement) + (domeAngleRWFails * anglePulseIncrement);
            angle_position_t newAngPos = {.angleBase = newAngleBase, .timeOfAngleIncrement_us = currentTime_us};
            // write back new value
            if(domeAngleSignal.writeFromISR(newAngPos, &xHigherPriorityTaskWoken)){
                // reset missed pulse counter on successful writeback
                domeAngleRWFails = 0;
            }
        }
        else{
            domeAngleRWFails += 1;
        }
        

        // initialize variable before measuring
        if(lastPulseTime_us == 0){
            lastPulseTime_us = time_us_32();
            return;
        }
        // calculate time between the last 2 magnetic pulses
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

void pushBtnLeftISR(BaseType_t& xHigherPriorityTaskWoken){
    static uint32_t lastTriggerTime_ms = 0;
    const uint32_t currentTime_ms = xTaskGetTickCountFromISR();
    if(currentTime_ms - lastTriggerTime_ms > BTN_DEBOUNCE_MS){
        // button isn't bouncing
        bool readSuccess = false;
        runtime_settings_t rts = rtSettingsSignal.readFromISR(readSuccess, &xHigherPriorityTaskWoken);
        if(readSuccess){
            rts.enableMotor = !rts.enableMotor;
            rtSettingsSignal.writeFromISR(rts, &xHigherPriorityTaskWoken);
        }
    }
    lastTriggerTime_ms = currentTime_ms;
}

void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(gpio == PIN_HALL_SENSOR) hallSensorISR(events, xHigherPriorityTaskWoken);
    else if(gpio == PIN_PUSHBTN_LEFT) pushBtnLeftISR(xHigherPriorityTaskWoken);

    if(xHigherPriorityTaskWoken){
        portYIELD_FROM_ISR(pdTRUE);
    }
}

void configurePins(){
    pinMode(PIN_PUSHBTN_LEFT, INPUT);
    pinMode(PIN_LED_USER, OUTPUT);
    digitalWrite(PIN_LED_USER, 1);

    gpio_init(PIN_HALL_SENSOR);
    gpio_set_dir(PIN_HALL_SENSOR, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_HALL_SENSOR, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_callback);

    gpio_init(PIN_PUSHBTN_LEFT);
    gpio_set_dir(PIN_PUSHBTN_LEFT, GPIO_IN);
    // only first gpio irq to be configured has to use gpio_set_irq_enabled_with_callback
    gpio_set_irq_enabled(PIN_PUSHBTN_LEFT, GPIO_IRQ_EDGE_FALL, true);

    gpio_set_function(PIN_MOTOR_PWM, GPIO_FUNC_PWM);
    uint8_t pwmSlice = pwm_gpio_to_slice_num(PIN_MOTOR_PWM);
    uint8_t pwmChan = pwm_gpio_to_channel(PIN_MOTOR_PWM);
    pwm_set_wrap(pwmSlice, 255);
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

    averaging_filter_init(&hallIntervalFilter);
    averaging_filter_init(&measuredRPMFilter);
    averaging_filter_init(&targetRPMFilter);

    // Pin configuration
    ULOG_TRACE("Configuring pins");
    configurePins();

    // signals for communicating measured RPM and target RPM between tasks
    ULOG_TRACE("Initializing communication signals");
    measuredRPMSignal.init();
    measuredRPMSignal.write({.rpm = 0}, 0);
    rtSettingsSignal.init();
    rtSettingsSignal.write({
        .pid_controller = {
            .kp = K_P,
            .ki = K_I,
            .kd = K_D,
            .targetRPM = MOTOR_TARGET_SPEED,
        },
        .enableMotor = false,
        .stableTargetRPM = false
    }, 0);
    domeAngleSignal.init();
    domeAngleSignal.write({.angleBase = 0, .timeOfAngleIncrement_us = 0}, 0);

    // create tasks
    ULOG_TRACE("Creating tasks");
    motorControlTaskParams_t mCtrlTaskParams = {
        .measuredRPMSignal = measuredRPMSignal,
        .runtimeSettingsSignal = rtSettingsSignal
    };
    if(pdPASS != xTaskCreate(motorControlTask, MOTOR_CONTROL_TASK_NAME, MOTOR_CONTROL_TASK_STACK_SIZE,
                             (void*)&mCtrlTaskParams, MOTOR_CONTROL_TASK_PRIORITY, &motorCtrlTaskHandle)) {
        ULOG_CRITICAL("Failed to create motor control task");
        configASSERT(false);
    }
    vTaskCoreAffinitySet(motorCtrlTaskHandle, (1<<0));
    signalAgeCheckTaskParams_t sigAgeChkTskParams = {
        .measuredRPMSignal = measuredRPMSignal,
        .domeAngleSignal = domeAngleSignal
    };
    if(pdPASS != xTaskCreate(signalAgeCheckTask, SIGNAL_AGE_CHECK_TASK_NAME,
                             SIGNAL_AGE_CHECK_TASK_STACK_SIZE, (void*)&sigAgeChkTskParams,
                             SIGNAL_AGE_CHECK_TASK_PRIORITY, &signalAgeCheckTaskHandle)) {
        ULOG_CRITICAL("Failed to create signal age check task");
        configASSERT(false);
    }
    vTaskCoreAffinitySet(signalAgeCheckTaskHandle, (1<<0));
    serialInterfaceTaskParams_t serIntTskParams = {
        .measuredRPMSignal = measuredRPMSignal,
        .runtimeSettingsSignal = rtSettingsSignal
    };
    if(pdPASS != xTaskCreate(serialInterfaceTask, SERIAL_INTERFACE_TASK_NAME,
                             SERIAL_INTERFACE_TASK_STACK_SIZE, (void*)&serIntTskParams,
                             SERIAL_INTERFACE_TASK_PRIORITY, &serialInterfaceTaskHandle)) {
        ULOG_CRITICAL("Failed to create serial interface task");
        configASSERT(false);
    }
    vTaskCoreAffinitySet(signalAgeCheckTaskHandle, (1<<0));
    sensorTaskParams_t sensorTskParams = {
        .measuredRPMSignal = measuredRPMSignal,
        .runtimeSettingsSignal = rtSettingsSignal,
        .domeAngleSignal = domeAngleSignal
    };
    if(pdPASS != xTaskCreate(sensorTask, SENSOR_TASK_NAME,
                             SENSOR_TASK_STACK_SIZE, (void*)&sensorTskParams,
                             SENSOR_TASK_PRIORITY, &sensorTaskHandle)) {
        ULOG_CRITICAL("Failed to create sensor task");
        configASSERT(false);
    }
    vTaskCoreAffinitySet(sensorTaskHandle, (1<<0));

    ULOG_TRACE("Setup finished");
    vTaskDelete(NULL);
    taskYIELD();
    // This is done in the background by the arduino-pico core already
    // vTaskStartScheduler();
    
}

void loop() {}