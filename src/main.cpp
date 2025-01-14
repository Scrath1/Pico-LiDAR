#include <Arduino.h>
#include <RP2040_PWM.h>
#include <pico/stdlib.h>
#include "spid.h"


#define Serial Serial1
#define PIN_MOTOR_PWM 16
#define PIN_HALL_SENSOR 26
#define PIN_POTENTIOMETER 27
#define PIN_SWITCH_LEFT 11
#define PIN_LED_USER 17

#define HALL_SENSOR_DETECTION_THRESHOLD 1000
float pwm_frequency = 1000;
RP2040_PWM PWM_instance(PIN_MOTOR_PWM, pwm_frequency, 0);

#define K_P (0.1)
#define K_I (0.1)
#define K_D (0)
#define PID_INTERVAL_S (0.5) // 500ms
#define PID_MIN_OUT (0)
#define PID_MAX_OUT (255)
struct repeating_timer motorTimer, hallTimer;
spid_t pid;

#define MAX_TARGET_RPM (600)
volatile uint16_t targetRPM = 0;
volatile uint16_t measuredRPM = 0;

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

bool measurementLoop(struct repeating_timer* t){
    static uint32_t lastRotationCompleteTime_us = 0;
    static uint32_t lastRotationPeriod_us = 0;
    uint32_t currentTime = time_us_32();
    static bool risingEdge = false;
    uint16_t hall = analogRead(PIN_HALL_SENSOR);
    if(hall > HALL_SENSOR_DETECTION_THRESHOLD){
        if(!risingEdge){
            risingEdge = true;
            lastRotationPeriod_us = currentTime - lastRotationCompleteTime_us;
            lastRotationCompleteTime_us = currentTime;
            measuredRPM = (1/((float)lastRotationPeriod_us / 1E6)) * 60 / 2;
            digitalWrite(PIN_LED_USER, 0);
        }
    }
    else{
        digitalWrite(PIN_LED_USER, 1);
        risingEdge = false;
    }
    // decay rpm if after a double the last rotation period
    // no new completed rotation has been detected
    // uint32_t tsinceFullRev_us = currentTime - lastRotationCompleteTime_us;
    // if(tsinceFullRev_us > lastRotationPeriod_us){
    //     measuredRPM = measuredRPM * 0.85;
    // }

    return true;
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
    pinMode(PIN_HALL_SENSOR, INPUT);
    pinMode(PIN_POTENTIOMETER, INPUT);
    pinMode(PIN_SWITCH_LEFT, INPUT);
    pinMode(PIN_LED_USER, OUTPUT);

    // PID controller initialization
    spid_init(&pid, K_P, K_I, K_D, PID_MIN_OUT, PID_MAX_OUT, PID_INTERVAL_S);
    // add callback for pid using hardware timer
    if(!add_repeating_timer_ms(PID_INTERVAL_S * 1000, motorLoop, NULL, &motorTimer)){
        Serial.println("Failed to create motor loop timer");
        while(1);
    }
    if(!add_repeating_timer_ms(10, measurementLoop, NULL, &hallTimer)){
        Serial.println("Failed to create sensor feedback loop timer");
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