#ifndef PRJ_CONFIG_H
#define PRJ_CONFIG_H

// PIN CONFIGURATIONS
// ==============================================================
#define PIN_MOTOR_PWM (18)
#define PIN_SPEED_HALL_SENSOR (20)
#define PIN_ZERO_HALL_SENSOR (21)
#define PIN_PUSHBTN (3)
#define PIN_LED_USER (2)
#define PIN_ECHO (27)
#define PIN_TRIG (26)
#define LED_ON (1)
#define LED_OFF (0)
// debouncing time for buttons
#define BTN_DEBOUNCE_MS (100)

// COMMUNICATION
// ==============================================================
#define SERIAL_PORT Serial1

// MOTOR CONTROL CONFIGURATIONS
// ==============================================================
// Maximum target speed for the motor
#define MAX_TARGET_RPM (1000)
// Default motor rotation target speed
#define MOTOR_TARGET_SPEED (120)
// Defines the value which represents a PWM duty cycle of 100%
#define PWM_FULL_VALUE (16383)
// PID control P constant
#define K_P (1.6)
// PID control I constant
#define K_I (1.6)
// PID control D constant
#define K_D (0)
// Interval with which the PID control loop is run in milliseconds
#define PID_INTERVAL_MS (100)
// Minimum PID controller output value
#define PID_MIN_OUT (0)
// Maximum PID controller output value
#define PID_MAX_OUT (PWM_FULL_VALUE)
// Maximum PID constant value for very basic error checking purposes
#define PID_CONSTANT_MAX (10)
// Number of PID intervals over which the measured RPM has to be relatively
// constant before being declared stable
#define MEASURED_RPM_STABILITY_INTERVAL_COUNT (25)
// the +- range by which the measured RPM may differ from the
// target RPM over MEASURED_RPM_STABILITY_INTERVAL_COUNT PID control
// intervals for the speed to be considered stable
#define MEASURED_RPM_TOLERANCE (10)

// MOTOR SPEED MEASUREMENT CONFIGURATIONS
// ==============================================================
// Number of magnet pulses for a full rotation
#define PULSES_PER_REV 4
// Filter size for averaging of measured RPM
#define RPM_AVERAGING_FILTER_SIZE 4

// SERIAL INTERFACE CONFIGURATIONS
// ==============================================================
// Specifies the interval with which the serial task is run
#define SERIAL_INTERFACE_TASK_INTERVAL_MS (100)
// Specifies the interval with which runtime data such as the
// current speed and pwm value are printed to the console for plotting.\
// This value should be greater or equal to SERIAL_INTERFACE_TASK_INTERVAL_MS
#define SERIAL_INTERFACE_PLOT_OUTPUT_INTERVAL_MS (100)
// Determines the maximum length of a serial command in bytes
#define SERIAL_CMD_INPUT_BUFFER_SIZE (64)
// Time after which received serial data for commands is discarded if no
// new data was received and the command is incomplete.
#define SERIAL_CMD_INPUT_TIMEOUT_MS (105)
// Maximum length of a debug message
#define DBG_MESSAGE_MAX_LEN (256)
#define TX_BUFFER_LEN (10 * DBG_MESSAGE_MAX_LEN)

// SENSOR CONFIGURATION
// ==============================================================
// Determines how many different data points are acquired per full
// revolution of the dome. If this value is too high it may not be possible to reach
#define DEFAULT_SCANPOINTS_PER_REV (16)
// Maximum time in ms for the VL53L0X to measure a distance
#define VL53L0X_TIME_BUDGET_MS (20)
// How often the MCU tries to connect to the VL53L0X before ignoring it
// and only using the HC-SR04
#define VL53L0X_MAX_CONNECTION_ATTEMPTS (5)
// Maximum time to wait for the result of the HC-SR04 sensor.
// This time is in addition to the time budget of the VL53L0X sensor
// as the HC-SR04 is triggered first and read later
#define HC_SR04_TIME_BUDGET_MS (10)
// Extra time budget for each scan above the bare minimum
// required for getting the sensor readings. Increase if you get
// warnings about timing violations
#define SENSOR_SCAN_EXTRA_TIME_BUDGET_MS (12)
// Default offset by which to rotate any angle value
#define DEFAULT_ANGLE_OFFSET (-45)
// Distance of the HC-SR04 sensor from the the rotation center
#define HC_SR04_CENTER_OFFSET_MM (35)
// Distance of the VL53L0X sensor from the the rotation center
#define VL53L0X_CENTER_OFFSET_MM (44)

// SIGNAL AGE
// ==============================================================
// How often the task checking the age of signals is executed
#define SIGNAL_AGE_CHECK_INTERVAL_MS (25)
// How old the measuredRPM signal is allowed to be before it is invalidated
#define SIGNAL_MEASURED_RPM_AGE_THRESHOLD_MS (300)

#endif  // PRJ_CONFIG_H