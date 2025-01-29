#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include <cstdint>

#define CMD_END_DELIMITER ('\n')

/**
 * Commands begin with a single instruction byte (see cmd_instruction_t)
 * followed by however many parameters this specific command needs
 * and finished using a delimiter byte containing the ASCII code for '\n'
 * Parameter bytes are ordered MSB to LSB
 * 
 * Table generated using https://ozh.github.io/ascii-tables/
 * +-------------+-------------+-------+-------------+-----------+
 * | Instruction | Parameter 1 |  ...  | Parameter n | Delimiter |
 * +-------------+-------------+-------+-------------+-----------+
 * | 8bit        | 32bit       | 32bit | 32bit       | 8bit (\n) |
 * +-------------+-------------+-------+-------------+-----------+
 */

typedef enum{
    CMD_NONE = 0,
    CMD_SET = 1,
    CMD_GET = 2
} cmd_instruction_t;
    
typedef enum{
    ID_NONE = 0,
    ID_KP = 1,
    ID_KI = 2,
    ID_KD = 3,
    ID_TARGET_RPM = 4,
    ID_ENABLE_MOTOR = 5
} parameter_id_t;

typedef struct{
    cmd_instruction_t instruction;
    uint32_t numOfParams;
} serial_command_t;

/**
 * Takes a byte array which should match the instruction written above
 * and tries to parse it. Depending on the command found, the frame is
 * passed to the corresponding function
 */
bool parseCommand(const uint8_t* frame, uint8_t frameSize);

bool cmd_set(const uint8_t* frame, const uint32_t frameSize);

bool cmd_get(const uint8_t* frame, const uint32_t frameSize);

#endif // SERIAL_COMMANDS_H