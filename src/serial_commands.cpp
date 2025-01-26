#include "serial_commands.h"
#include <Arduino.h>
#include "prj_config.h"

#include "ulog.h"

#define SIGNAL_LOCK_TIMEOUT 10

bool parseCommand(runtime_settings_signal_t& settings, const uint8_t* frame, uint8_t frameSize){
    if(frame == NULL){
        ULOG_DEBUG("frame is NULL");
        return false;
    }

    // print received frame
    // Serial.print("\n");
    // for(uint32_t i = 0; i < frameSize; i++){
    //     Serial.printf("%.2X", frame[i]);
    // }

    // we should have n*4+2 bytes in the frame.
    // 1 byte for the command, 4 byte per parameter and 1 byte
    // for the final delimiter
    if((frameSize % 4) != 2){
        ULOG_DEBUG("Invalid command framesize: %u", frameSize);
        return false;
    }

    cmd_instruction_t cmd = (cmd_instruction_t)(frame[0]);
    switch(cmd){
        default:
        case CMD_NONE:
            ULOG_ERROR("Command code is NONE");
            return false;
        case CMD_SET:
            return cmd_set(settings, frame, frameSize);
        case CMD_GET:
            return cmd_get(settings, frame, frameSize);
    }
    return true;
}

/**
 * Converts 4 bytes to a 32bit word
 * byte[0] is the MSB
 * byte[3] is the LSB
 */
uint32_t bytesToWord32(const uint8_t bytes[4]){
    return (static_cast<uint32_t>(bytes[0]) << 24) |
        (static_cast<uint32_t>(bytes[1]) << 16) |
        (static_cast<uint32_t>(bytes[2]) << 8) |
        (static_cast<uint32_t>(bytes[3]) << 0);
}

bool cmd_set(runtime_settings_signal_t& settings, const uint8_t* frame, const uint32_t frameSize){
    if(frame == NULL){
        ULOG_DEBUG("frame must not be NULL");
        return false;
    }
    // the set command expects 2 parameters, so the expected frameSize is
    // 1 + 2*4 + 1
    if((frameSize != 10 || frameSize == 0)){
        ULOG_DEBUG("Invalid framesize for cmd_set: %u", frameSize);
        return false;
    }

    // sanity check that the function was called for the correct command
    if(frame[0] != CMD_SET){
        ULOG_DEBUG("cmd_set invalid cmd code: %u", frame[0]);
        return false;
    }

    parameter_id_t tgtVar = (parameter_id_t)bytesToWord32(&(frame[1]));
    
    bool readSuccess = false;
    runtime_settings_t rts = settings.read(readSuccess, SIGNAL_LOCK_TIMEOUT);
    if(!readSuccess){
        ULOG_DEBUG("cmd_set settings read failed");
        return false;
    }

    uint32_t parameterValue = bytesToWord32(&(frame[5]));
    switch(tgtVar){
        default:
            ULOG_ERROR("Invalid parameter ID: %lu", tgtVar);
            return false;
        case ID_NONE:
            ULOG_ERROR("Variable ID NONE received");
            return false;
        case ID_KP:
            rts.pid_controller.kp = *(reinterpret_cast<float*>(&parameterValue));
            if(rts.pid_controller.kp > PID_CONSTANT_MAX){
                // value wasn't written back to setting signal so
                // we can just return here
                ULOG_ERROR("K_P value too large: %0.3f", rts.pid_controller.kp);
                return false;
            }
            else ULOG_DEBUG("Updated to K_P: %0.3f", rts.pid_controller.kp);
            break;
        case ID_KI:
            rts.pid_controller.ki = *(reinterpret_cast<float*>(&parameterValue));
            if(rts.pid_controller.ki > PID_CONSTANT_MAX){
                // value wasn't written back to setting signal so
                // we can just return here
                ULOG_ERROR("K_I value too large: %0.3f", rts.pid_controller.ki);
                return false;
            }
            else ULOG_DEBUG("Updated to K_I: %0.3f", rts.pid_controller.ki);
            break;
        case ID_KD:
            rts.pid_controller.kd = *(reinterpret_cast<float*>(&parameterValue));
            if(rts.pid_controller.kd > PID_CONSTANT_MAX){
                // value wasn't written back to setting signal so
                // we can just return here
                ULOG_ERROR("K_D value too large: %0.3f", rts.pid_controller.kd);
                return false;
            }
            else ULOG_DEBUG("Updated to K_D: %0.3f", rts.pid_controller.kd);
            break;
        case ID_TARGET_RPM:
            rts.pid_controller.targetRPM = parameterValue;
            if(rts.pid_controller.targetRPM > MAX_TARGET_RPM){
                ULOG_ERROR("Target RPM too large: %lu", parameterValue);
                return false;
            }
            break;
        case ID_ENABLE_MOTOR:
            rts.enableMotor = parameterValue;
            break;
    }
    // write back settings object
    if(settings.write(rts, SIGNAL_LOCK_TIMEOUT))
        return true;
    else{
        ULOG_ERROR("Settings writeback failed");
        return false;
    }
}

bool cmd_get(runtime_settings_signal_t& settings, const uint8_t* frame, const uint32_t frameSize){
    if(frame == NULL){
        ULOG_DEBUG("frame must not be NULL");
        return false;
    }
    // The get command expects a frame length of 6
    // 1 + 4 + 1
    if((frameSize != 6 || frameSize == 0)){
        ULOG_DEBUG("Invalid framesize for cmd_get: %u", frameSize);
        return false;
    }

    // sanity check that the function was called for the correct command
    if(frame[0] != CMD_GET){
        ULOG_DEBUG("cmd_get invalid cmd code: %u", frame[0]);
        return false;
    }

    parameter_id_t tgtVar = (parameter_id_t)bytesToWord32(&(frame[1]));
    
    bool readSuccess = false;
    runtime_settings_t rts = settings.read(readSuccess, SIGNAL_LOCK_TIMEOUT);
    if(!readSuccess){
        ULOG_DEBUG("cmd_set settings read failed");
        return false;
    }

    switch(tgtVar){
        default:
            ULOG_ERROR("Invalid parameter ID: %lu", tgtVar);
            return false;
        case ID_NONE:
            ULOG_ERROR("Variable ID NONE received");
            return false;
        case ID_KP:
            Serial.printf(">K_p:%f\n", rts.pid_controller.kp);
            break;
        case ID_KI:
            Serial.printf(">K_i:%f\n", rts.pid_controller.ki);
            break;
        case ID_KD:
            Serial.printf(">K_d:%f\n", rts.pid_controller.kd);
            break;
        case ID_TARGET_RPM:
            Serial.printf(">targetRPM:%lu\n", rts.pid_controller.targetRPM);
            break;
        case ID_ENABLE_MOTOR:
            Serial.printf(">enableMotor:%u\n", rts.enableMotor);
            break;
    }
    return true;
}