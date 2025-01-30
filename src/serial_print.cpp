#include "serial_print.h"
#include "tasks/serial_tx_task.h"
#include <cstdio>
#include "prj_config.h"
#include <Arduino.h>

bool useTxTask;

void serialPrint(const char* msg){
    if(useTxTask){
        uint32_t len = strlen(msg)+1; // +1 to include the null-terminator
        serialPrint(msg, len);
    }
    else{
        SERIAL_PORT.print(msg);
    }
}

void serialPrint(const char* msg, uint32_t msgLen){
    if(useTxTask){
        // putString(msg, msgLen);
        while(!putString(msg, msgLen));
    }
    else{
        SERIAL_PORT.print(msg);
    }
}

void serialPrintf(const char* fmt, ...){
    va_list args;
    va_start(args, fmt);

    char buf[DBG_MESSAGE_MAX_LEN];
    vsnprintf(buf, DBG_MESSAGE_MAX_LEN, fmt, args);
    va_end(args);
    serialPrint(buf, DBG_MESSAGE_MAX_LEN);
}