#include <Arduino.h>
#include <RP2040_PWM.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include "spid.h"

#define Serial Serial1
#define PIN_MOTOR_PWM 16
#define PIN_HALL_SENSOR 26
#define PIN_POTENTIOMETER 27
#define PIN_SWITCH_LEFT 11
#define PIN_LED_USER 17

// Motor speed measurement stuff
#define PULSES_PER_REV 2
#define RPM_AVERAGING_FILTER_SIZE 8

#define K_P (0.1)
#define K_I (0.1)
#define K_D (0)
#define PID_INTERVAL_S (0.5) // 500ms
#define PID_MIN_OUT (0)
#define PID_MAX_OUT (100)
struct repeating_timer motorTimer, rpmDecayTimer;
spid_t pid;

#define MAX_TARGET_RPM (1600)
volatile uint16_t targetRPM = 0;
volatile uint16_t measuredRPM = 0;
volatile uint32_t lastRPMUpdateTime_us = 0;
volatile uint32_t expectedRPMUpdateInterval_us = 0;
#define RPM_DECAY_CHECK_INTERVAL_S (0.5)
#define RPM_DECAY_PERCENTAGE (0.2) // 20% per loop iteration

bool motorLoop(struct repeating_timer* t){
    // check if motor enable switch is on
    uint8_t pwm = 0;
    if(digitalRead(PIN_SWITCH_LEFT)){
        pwm = (uint8_t)spid_process(&pid, targetRPM, measuredRPM);
    }
    // PWM_instance.setPWM(PIN_MOTOR_PWM, pwm_frequency, pwm);
    analogWrite(PIN_MOTOR_PWM, pwm);
    Serial.print(">PWM:");
    Serial.println(pwm);
    return true;
}

bool rpmDecayLoop(struct repeating_timer* t){
    uint32_t currentTime_us = time_us_32();
    uint32_t timeSinceUpdate_us = currentTime_us - lastRPMUpdateTime_us;
    if(timeSinceUpdate_us > expectedRPMUpdateInterval_us * 4){
        measuredRPM *= RPM_DECAY_PERCENTAGE;
    }
    return true;
}

void gpio_callback(uint gpio, uint32_t events){
    if(gpio == PIN_HALL_SENSOR && events & GPIO_IRQ_EDGE_RISE){
        // signal detection using LED
        digitalWrite(PIN_LED_USER, 1);
    }
    else if(gpio == PIN_HALL_SENSOR && events & GPIO_IRQ_EDGE_FALL){
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
        pulseIntervals[pulseIntervalsNextIdx] = interval_us;
        pulseIntervalsNextIdx = (pulseIntervalsNextIdx+1) % pulseIntervalsArrSize;

        // average pulse times
        uint32_t averageInterval_us = 0;
        for(uint32_t i = 0; i < pulseIntervalsArrSize; i++){
            averageInterval_us += pulseIntervals[i];
        }
        averageInterval_us /= pulseIntervalsArrSize;
        // Serial.print(">intv:");
        // Serial.println(interval_us);
        // calculate actual axle rotation speed
        static uint32_t rpmMeasurements[RPM_AVERAGING_FILTER_SIZE];
        static const uint32_t rpmMeasurementsSize = sizeof(rpmMeasurements)/sizeof(rpmMeasurements[0]);
        static uint32_t rpmMeasurementsNextIdx = 0;
        uint32_t currentRPMMeasurement = (1/((float)averageInterval_us / 1E6)) * 60 * PULSES_PER_REV;
        rpmMeasurements[rpmMeasurementsNextIdx] = currentRPMMeasurement;
        rpmMeasurementsNextIdx = (rpmMeasurementsNextIdx+1) % rpmMeasurementsNextIdx;

        uint32_t averagedRPM = 0;
        for(uint32_t i = 0; i < rpmMeasurementsSize; i++){
            averagedRPM += rpmMeasurements[i];
        }
        averagedRPM /= rpmMeasurementsSize;
        measuredRPM = averagedRPM;
        lastRPMUpdateTime_us = currentTime_us;
        expectedRPMUpdateInterval_us = averageInterval_us;
    }
}

void setup()
{
    // put your setup code here, to run once:
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

    // PID controller initialization
    spid_init(&pid, K_P, K_I, K_D, PID_MIN_OUT, PID_MAX_OUT, PID_INTERVAL_S);
    // add callback for pid using hardware timer
    if(!add_repeating_timer_ms(PID_INTERVAL_S * 1000, motorLoop, NULL, &motorTimer)){
        Serial.println("Failed to create motor loop timer");
        while(1);
    }
    if(!add_repeating_timer_ms(RPM_DECAY_CHECK_INTERVAL_S * 1000, rpmDecayLoop, NULL, &rpmDecayTimer)){
        Serial.println("Failed to create rpm decay loop timer");
        while(1);
    }
}

void loop()
{
    // put your main code here, to run repeatedly:
    uint16_t pot = analogRead(PIN_POTENTIOMETER);
    targetRPM = map(pot, 0, 1023, 0, MAX_TARGET_RPM);
    sleep_ms(100);

    Serial.print(">targetRPM:");
    Serial.println(targetRPM);
    Serial.print(">measuredRPM:");
    Serial.println(measuredRPM);
}