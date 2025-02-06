#include "serial_commands.h"

#include "global.h"
#include "prj_config.h"
#include "serial_print.h"
#include "ulog.h"
#include "hardware/watchdog.h"

#define LOG_LOCATION_NAME ("SerCmds")

/**
 * Converts 4 bytes to a 32bit word
 * byte[0] is the MSB
 * byte[3] is the LSB
 */
uint32_t bytesToWord32(const uint8_t bytes[4]) {
    return (static_cast<uint32_t>(bytes[0]) << 24) | (static_cast<uint32_t>(bytes[1]) << 16) |
           (static_cast<uint32_t>(bytes[2]) << 8) | (static_cast<uint32_t>(bytes[3]) << 0);
}

bool cmd_set(parameter_id_t id, const uint8_t valueBytes[4]) {
    uint32_t parameterValue = bytesToWord32(valueBytes);
    switch(id) {
        default:
            ULOG_ERROR("%s: Invalid parameter ID: %lu", LOG_LOCATION_NAME, id);
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
        case ID_VL53L0X_TIME_BUDGET: {
            uint32_t budget = parameterValue;
            if(budget != runtimeSettings.vl53l0xMeasurementTimingBudget_us.get()){
                ULOG_INFO("%s: %s changed: %lu -> %lu", LOG_LOCATION_NAME, runtimeSettings.vl53l0xMeasurementTimingBudget_us.name,
                    runtimeSettings.vl53l0xMeasurementTimingBudget_us.get(), budget);
                runtimeSettings.vl53l0xMeasurementTimingBudget_us.set(budget);
            }
        } break;
        case ID_DATAPOINTS_PER_REV: {
            uint32_t dppr = parameterValue;
            if(dppr != runtimeSettings.dataPointsPerRev.get() && dppr > 0){
                ULOG_INFO("%s: %s changed: %lu -> %lu", LOG_LOCATION_NAME, runtimeSettings.dataPointsPerRev.name,
                    runtimeSettings.dataPointsPerRev.get(), dppr);
                runtimeSettings.dataPointsPerRev.set(dppr);
            }
        } break;
        case ID_ANGLE_OFFSET: {
            int32_t offset = (*(reinterpret_cast<int32_t*>(&parameterValue))) % 360;
            if(offset != runtimeSettings.angleOffset.get()) {
                ULOG_INFO("%s: %s changed: %i -> %i", LOG_LOCATION_NAME, runtimeSettings.angleOffset.name,
                          runtimeSettings.angleOffset.get(), offset);
                runtimeSettings.angleOffset.set((uint16_t)offset);
            }
        } break;
        case ID_SERIAL_ERROR_COUNTER:
            ULOG_ERROR("%s: serial error counter is read-only", LOG_LOCATION_NAME);
            break;
        case ID_RESET:
            if(parameterValue == 1){
                ULOG_INFO("%s: Resetting MCU", LOG_LOCATION_NAME);
                watchdog_enable(10, true);
            }
            else{
                ULOG_ERROR("%s: Parameter for reset command must be 1", LOG_LOCATION_NAME);
            }
            break;
    }
    return true;
}

bool cmd_get(parameter_id_t id) {
    switch(id) {
        default:
            ULOG_ERROR("%s: Invalid parameter ID: %lu", LOG_LOCATION_NAME, id);
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
        case ID_VL53L0X_TIME_BUDGET:
            serialPrintf(">%s:%lu\n", runtimeSettings.vl53l0xMeasurementTimingBudget_us.name, runtimeSettings.vl53l0xMeasurementTimingBudget_us.get());
            break;
        case ID_DATAPOINTS_PER_REV:
            serialPrintf(">%s:%i\n", runtimeSettings.angleOffset.name, runtimeSettings.angleOffset.get());
            break;
        case ID_ANGLE_OFFSET:
            serialPrintf(">%s:%i\n", runtimeSettings.angleOffset.name, runtimeSettings.angleOffset.get());
            break;
        case ID_SERIAL_ERROR_COUNTER:
            serialPrintf(">serialErrorCounter:%lu\n", status.serialCmdErrors);
            break;
        case ID_RESET:
            ULOG_ERROR("Reset is write only");
            break;
    }
    return true;
}

bool parseCommand(cmd_instruction_t cmd, const uint8_t targetIdBytes[4], const uint8_t valueBytes[4]) {
    if(targetIdBytes == NULL) {
        ULOG_DEBUG("%s: targetIdBytes is NULL", LOG_LOCATION_NAME);
        status.serialCmdErrors++;
        return false;
    }
    if(valueBytes == NULL){
        ULOG_DEBUG("%s: valueBytes is NULL", LOG_LOCATION_NAME);
        status.serialCmdErrors++;
        return false;
    }

    // print received frame
    // serialPrint("\n");
    // for(uint32_t i = 0; i < frameSize; i++){
    //     serialPrintf("%.2X", frame[i]);
    // }

    parameter_id_t id = static_cast<parameter_id_t>(bytesToWord32(targetIdBytes));
    // ULOG_TRACE("%s: Parsed ID:%u from bytes %x%x%x%x", LOG_LOCATION_NAME, id,targetIdBytes[0], targetIdBytes[1], targetIdBytes[2], targetIdBytes[3]);
    bool ret = false;
    switch(cmd) {
        default:
        case CMD_NONE:
            ULOG_ERROR("%s: Command code is NONE or unknown: %u", LOG_LOCATION_NAME, (uint8_t)cmd);
            ret = false;
            break;
        case CMD_SET:
            ret = cmd_set(id, valueBytes);
            break;
        case CMD_GET:
            ret = cmd_get(id);
            break;
    }
    if(!ret) status.serialCmdErrors++;
    return ret;
}