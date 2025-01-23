#ifndef PRJ_CONFIG_H
#define PRJ_CONFIG_H

// PIN CONFIGURATIONS
// ==============================================================
#define PIN_MOTOR_PWM 16
#define PIN_HALL_SENSOR 26
#define PIN_POTENTIOMETER 27
#define PIN_SWITCH_LEFT 11
#define PIN_LED_USER 17
#define LED_ON (0)
#define LED_OFF (1)

// COMMUNICATION
// ==============================================================
#define Serial Serial1

// MOTOR CONTROL CONFIGURATIONS
// ==============================================================
// Maximum target speed for the motor
#define MAX_TARGET_RPM (1000)
// PID control P constant
#define K_P (0.1)
// PID control I constant
#define K_I (0.1)
// PID control D constant
#define K_D (0)
// Interval with which the PID control loop is run in milliseconds
#define PID_INTERVAL_MS (500)
// Minimum PID controller output value
#define PID_MIN_OUT (0)
// Maximum PID controller output value
#define PID_MAX_OUT (255)

// MOTOR SPEED MEASUREMENT CONFIGURATIONS
// ==============================================================
// Number of magnet pulses for a full rotation
#define PULSES_PER_REV 4
// Filter size for averaging of measured RPM
#define RPM_AVERAGING_FILTER_SIZE 8

// SIGNAL AGE
// ==============================================================
// How often the task checking the age of signals is executed
#define SIGNAL_AGE_CHECK_INTERVAL_MS (25)
// How old the measuredRPM signal is allowed to be before it is invalidated
#define SIGNAL_MEASURED_RPM_AGE_THRESHOLD_MS (PID_INTERVAL_MS / PULSES_PER_REV * 2)

#endif // PRJ_CONFIG_H