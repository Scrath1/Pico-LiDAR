#ifndef SERIAL_PRINT_H
#define SERIAL_PRINT_H
#include <cstdint>
#include <cstdarg>

extern bool useTxTask;

void serialPrint(const char* msg);
void serialPrint(const char* msg, uint32_t msgLen);
void serialPrintf(const char* fmt, ...);

#endif // SERIAL_PRINT_H