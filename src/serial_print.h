#ifndef SERIAL_PRINT_H
#define SERIAL_PRINT_H
#include <cstdarg>
#include <cstdint>

extern volatile bool useTxTask;

void serialPrint(const char* msg);
void serialPrint(const char* msg, uint32_t msgLen);
void serialPrintf(const char* fmt, ...);

#endif  // SERIAL_PRINT_H