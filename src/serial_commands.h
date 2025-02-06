#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include <cstdint>

/**
 * Commands begin with a single instruction byte (see cmd_instruction_t)
 * followed by 4 bytes for the ID of the variable to set or get
 * followed by another 4 bytes for the value to set. In case of a get command
 * these last 4 bytes are all 0.
 * Bytes are ordered MSB to LSB
 *
 * Table generated using https://ozh.github.io/ascii-tables/
 * +-------------+-------------+-------------+
 * | Instruction | Variable ID | Value       |
 * +-------------+-------------+-------------+
 * | 8bit        | 32bit       | 32bit       |
 * +-------------+-------------+-------------+
 */

#define COMMAND_FRAME_SIZE (1 + 4 + 4 + 1)

typedef enum { CMD_NONE = 0, CMD_SET = 1, CMD_GET = 2, CMD_RESET = 3} cmd_instruction_t;

typedef enum {
    ID_NONE = 0, // placeholder
    ID_KP = 1, // PID K_p parameter
    ID_KI = 2, // PID K_i parameter
    ID_KD = 3, // PID K_d parameter
    ID_TARGET_RPM = 4, // target speed for sensor dome
    ID_ENABLE_MOTOR = 5, // enable/disable motor
    ID_VL53L0X_TIME_BUDGET = 6,
    ID_DATAPOINTS_PER_REV = 7, // Number of datapoints per dome revolution
    ID_ANGLE_OFFSET = 8, // Value by which to shift all calculated angles
    ID_SERIAL_ERROR_COUNTER = 9, // Number of errors when receiving serial commands
} parameter_id_t;

typedef struct {
    cmd_instruction_t instruction;
    uint32_t numOfParams;
} serial_command_t;

/**
 * Takes a byte array which should match the instruction written above
 * and tries to parse it. Depending on the command found, the frame is
 * passed to the corresponding function
 */
bool parseCommand(cmd_instruction_t cmd, const uint8_t targetIdBytes[4], const uint8_t valueBytes[4]);

#endif  // SERIAL_COMMANDS_H