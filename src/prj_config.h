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
// Determines the interval with which the motor speed measurement task
// calculates the measured intervals between magnetic pulses to determine
// the current rotation speed. Pulses are still being recorded without the task
// being active. This also determines how often the age of the last pulse
// is checked to determine whether the motor has stalled
#define MOTOR_SPEED_MEASUREMENT_TASK_INTERVAL_MS (PID_INTERVAL_MS)
// How many percent of the RPM measurement get lost with each
// decay interval
#define RPM_DECAY_PERCENTAGE (0.2) // 20% per loop iteration

// TASK COMMUNICATION
// ==============================================================
// Size of the queue for sending the trigger times of the hall effect sensor
// to the speed measuring task
#define HALL_TRIGGER_TIME_QUEUE_LEN (4096)

#endif // PRJ_CONFIG_H