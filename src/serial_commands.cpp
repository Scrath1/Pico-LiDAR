#include "serial_commands.h"

#include "global.h"
#include "prj_config.h"
#include "serial_print.h"
#include "ulog.h"

#define LOG_LOCATION_NAME ("SerCmds")

bool parseCommand(const uint8_t* frame, uint8_t frameSize) {
    if(frame == NULL) {
        ULOG_DEBUG("%s: frame is NULL", LOG_LOCATION_NAME);
        return false;
    }

    // print received frame
    // SERIAL.print("\n");
    // for(uint32_t i = 0; i < frameSize; i++){
    //     SERIAL.printf("%.2X", frame[i]);
    // }

    // we should have n*4+2 bytes in the frame.
    // 1 byte for the command, 4 byte per parameter and 1 byte
    // for the final delimiter
    if((frameSize % 4) != 2) {
        ULOG_DEBUG("%s: Invalid command framesize: %u", LOG_LOCATION_NAME, frameSize);
        return false;
    }

    cmd_instruction_t cmd = (cmd_instruction_t)(frame[0]);
    switch(cmd) {
        default:
        case CMD_NONE:
            ULOG_ERROR("%s: Command code is NONE", LOG_LOCATION_NAME);
            return false;
        case CMD_SET:
            return cmd_set(frame, frameSize);
        case CMD_GET:
            return cmd_get(frame, frameSize);
    }
    return true;
}

/**
 * Converts 4 bytes to a 32bit word
 * byte[0] is the MSB
 * byte[3] is the LSB
 */
uint32_t bytesToWord32(const uint8_t bytes[4]) {
    return (static_cast<uint32_t>(bytes[0]) << 24) | (static_cast<uint32_t>(bytes[1]) << 16) |
           (static_cast<uint32_t>(bytes[2]) << 8) | (static_cast<uint32_t>(bytes[3]) << 0);
}

bool cmd_set(const uint8_t* frame, const uint32_t frameSize) {
    if(frame == NULL) {
        ULOG_DEBUG("%s: frame must not be NULL", LOG_LOCATION_NAME);
        return false;
    }
    // the set command expects 2 parameters, so the expected frameSize is
    // 1 + 2*4 + 1
    if((frameSize != 10 || frameSize == 0)) {
        ULOG_DEBUG("%s: Invalid framesize for cmd_set: %u", LOG_LOCATION_NAME, frameSize);
        return false;
    }

    // sanity check that the function was called for the correct command
    if(frame[0] != CMD_SET) {
        ULOG_DEBUG("%s: cmd_set invalid cmd code: %u", LOG_LOCATION_NAME, frame[0]);
        return false;
    }

    parameter_id_t tgtVar = (parameter_id_t)bytesToWord32(&(frame[1]));

    uint32_t parameterValue = bytesToWord32(&(frame[5]));
    switch(tgtVar) {
        default:
            ULOG_ERROR("%s: Invalid parameter ID: %lu", LOG_LOCATION_NAME, tgtVar);
            return false;
        case ID_NONE:
            ULOG_ERROR("%s: Variable ID NONE received", LOG_LOCATION_NAME);
            return false;
        case ID_KP: {
            float kp = *(reinterpret_cast<float*>(&parameterValue));
            if(kp > PID_CONSTANT_MAX) {
                ULOG_ERROR("%s: K_P value too large: %0.3f", LOG_LOCATION_NAME, kp);
                return false;
            } else {
                ULOG_INFO("%s: %s changed: %0.3f -> %0.3f", LOG_LOCATION_NAME, runtimeSettings.pid_kp.name,
                          runtimeSettings.pid_kp.get(), kp);
                runtimeSettings.pid_kp.set(kp);
            }
        } break;
        case ID_KI: {
            float ki = *(reinterpret_cast<float*>(&parameterValue));
            if(ki > PID_CONSTANT_MAX) {
                ULOG_ERROR("%s: K_I value too large: %0.3f", LOG_LOCATION_NAME, ki);
                return false;
            } else {
                ULOG_INFO("%s: %s changed: %0.3f -> %0.3f", LOG_LOCATION_NAME, runtimeSettings.pid_ki.name,
                          runtimeSettings.pid_ki.get(), ki);
                runtimeSettings.pid_ki.set(ki);
            }
        } break;
        case ID_KD: {
            float kd = *(reinterpret_cast<float*>(&parameterValue));
            if(kd > PID_CONSTANT_MAX) {
                ULOG_ERROR("%s: K_D value too large: %0.3f", LOG_LOCATION_NAME, kd);
                return false;
            } else {
                ULOG_INFO("%s: %s changed: %0.3f -> %0.3f", LOG_LOCATION_NAME, runtimeSettings.pid_kd.name,
                          runtimeSettings.pid_kd.get(), kd);
                runtimeSettings.pid_kd.set(kd);
            }
        } break;
        case ID_TARGET_RPM: {
            uint32_t tgtRpm = parameterValue;
            if(tgtRpm > MAX_TARGET_RPM) {
                ULOG_ERROR("%s: Target RPM too large: %lu", LOG_LOCATION_NAME, parameterValue);
                return false;
            } else {
                ULOG_INFO("%s: %s changed: %lu -> %lu", LOG_LOCATION_NAME, runtimeSettings.targetRPM.name,
                          runtimeSettings.targetRPM.get(), tgtRpm);
                runtimeSettings.targetRPM.set(tgtRpm);
            }
        } break;
        case ID_ENABLE_MOTOR: {
            bool en = (bool)parameterValue;
            if(en != runtimeSettings.enableMotor.get()) {
                ULOG_INFO("%s: %s changed: %u -> %u", LOG_LOCATION_NAME, runtimeSettings.enableMotor.name,
                          runtimeSettings.enableMotor.get(), en);
                runtimeSettings.enableMotor.set(en);
            }
        } break;
    }
    return true;
}

bool cmd_get(const uint8_t* frame, const uint32_t frameSize) {
    if(frame == NULL) {
        ULOG_DEBUG("%s: frame must not be NULL", LOG_LOCATION_NAME);
        return false;
    }
    // The get command expects a frame length of 6
    // 1 + 4 + 1
    if((frameSize != 6 || frameSize == 0)) {
        ULOG_DEBUG("%s: Invalid framesize for cmd_get: %u", LOG_LOCATION_NAME, frameSize);
        return false;
    }

    // sanity check that the function was called for the correct command
    if(frame[0] != CMD_GET) {
        ULOG_DEBUG("%s: cmd_get invalid cmd code: %u", LOG_LOCATION_NAME, frame[0]);
        return false;
    }

    parameter_id_t tgtVar = (parameter_id_t)bytesToWord32(&(frame[1]));

    switch(tgtVar) {
        default:
            ULOG_ERROR("%s: Invalid parameter ID: %lu", LOG_LOCATION_NAME, tgtVar);
            return false;
        case ID_NONE:
            ULOG_ERROR("%s: Variable ID NONE received", LOG_LOCATION_NAME);
            return false;
        case ID_KP:
            serialPrintf(">%s:%f\n", runtimeSettings.pid_kp.name, runtimeSettings.pid_kp.get());
            break;
        case ID_KI:
            serialPrintf(">%s:%f\n", runtimeSettings.pid_ki.name, runtimeSettings.pid_ki.get());
            break;
        case ID_KD:
            serialPrintf(">%s:%f\n", runtimeSettings.pid_kd.name, runtimeSettings.pid_kd.get());
            break;
        case ID_TARGET_RPM:
            serialPrintf(">%s:%lu\n", runtimeSettings.targetRPM.name, runtimeSettings.targetRPM.get());
            break;
        case ID_ENABLE_MOTOR:
            serialPrintf(">%s:%u\n", runtimeSettings.enableMotor.name, runtimeSettings.enableMotor.get());
            break;
    }
    return true;
}