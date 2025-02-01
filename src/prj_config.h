#ifndef PRJ_CONFIG_H
#define PRJ_CONFIG_H

// PIN CONFIGURATIONS
// ==============================================================
#define PIN_MOTOR_PWM 16
#define PIN_SPEED_HALL_SENSOR 18
#define PIN_ZERO_HALL_SENSOR 19
#define PIN_PUSHBTN_LEFT 13
#define PIN_LED_USER 17
#define LED_ON (0)
#define LED_OFF (1)
// debouncing time for buttons
#define BTN_DEBOUNCE_MS (50)

// COMMUNICATION
// ==============================================================
#define SERIAL_PORT Serial1

// MOTOR CONTROL CONFIGURATIONS
// ==============================================================
// Maximum target speed for the motor
#define MAX_TARGET_RPM (1000)
// Default motor rotation target speed
#define MOTOR_TARGET_SPEED (300)
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
// Maximum PID constant value for error checking purposes
#define PID_CONSTANT_MAX (10)
// Number of PID intervals over which the measured RPM has to be relatively
// constant before being declared stable
#define MEASURED_RPM_STABILITY_INTERVAL_COUNT (5)
// the +- range by which the measured RPM may differ from the
// target RPM over MEASURED_RPM_STABILITY_INTERVAL_COUNT PID control
// intervals for the speed to be considered stable
#define MEASURED_RPM_TOLERANCE_PERCENT (0.025)

// MOTOR SPEED MEASUREMENT CONFIGURATIONS
// ==============================================================
// Number of magnet pulses for a full rotation
#define PULSES_PER_REV 4
// Filter size for averaging of measured RPM
#define RPM_AVERAGING_FILTER_SIZE 8

// SERIAL INTERFACE CONFIGURATIONS
// ==============================================================
// Specifies the interval with which the serial task is run
#define SERIAL_INTERFACE_TASK_INTERVAL_MS (100)
// Specifies the interval with which runtime data such as the
// current speed and pwm value are printed to the console for plotting
#define SERIAL_INTERFACE_PLOT_OUTPUT_INTERVAL_MS (100)
// Determines the maximum length of a serial command in bytes
#define SERIAL_CMD_INPUT_BUFFER_SIZE (64)
// Time after which received serial data for commands is discarded if no
// new data was received and the command is incomplete.
#define SERIAL_CMD_INPUT_TIMEOUT_MS (500)
// Maximum length of a debug message
#define DBG_MESSAGE_MAX_LEN (256)
#define TX_BUFFER_LEN (10 * DBG_MESSAGE_MAX_LEN)

// SENSOR CONFIGURATION
// ==============================================================
// Determines how many different data points are acquired per full
// revolution of the dome. If this value is too high it may not be possible to reach
#define DEFAULT_SCANPOINTS_PER_REV (16)
// Maximum time in ms that is allowed per range measurement.
#define RANGE_ACQUISITION_TIME_BUDGET_MS (33)

// SIGNAL AGE
// ==============================================================
// How often the task checking the age of signals is executed
#define SIGNAL_AGE_CHECK_INTERVAL_MS (25)
// How old the measuredRPM signal is allowed to be before it is invalidated
#define SIGNAL_MEASURED_RPM_AGE_THRESHOLD_MS (PID_INTERVAL_MS / PULSES_PER_REV * 2)

#endif  // PRJ_CONFIG_H