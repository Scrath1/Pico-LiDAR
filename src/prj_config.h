#ifndef PRJ_CONFIG_H
#define PRJ_CONFIG_H

// PIN CONFIGURATIONS
// ==============================================================
#define PIN_MOTOR_PWM 16
#define PIN_HALL_SENSOR 26
#define PIN_POTENTIOMETER 27
#define PIN_SWITCH_LEFT 11
#define PIN_LED_USER 17

// COMMUNICATION
// ==============================================================
#define Serial Serial1

// MOTOR CONTROL CONFIGURATIONS
// ==============================================================
// Maximum target speed for the motor
#define MAX_TARGET_RPM (1600)
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
// Interval with which the recentness of the RPM measurement is checked.
// If the measurement gets too old, begin decaying the RPM value
#define RPM_DECAY_CHECK_INTERVAL_S (0.5)
// How many percent of the RPM measurement get lost with each
// decay interval
#define RPM_DECAY_PERCENTAGE (0.2) // 20% per loop iteration

#endif // PRJ_CONFIG_H