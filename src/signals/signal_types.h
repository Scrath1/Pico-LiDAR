#ifndef SIGNAL_TYPES_H
#define SIGNAL_TYPES_H
#include "signal_template.h"

typedef struct{
    uint32_t rpm;
} rpm_data_t;

typedef signal<rpm_data_t> rpm_signal_t;

#endif // SIGNAL_TYPES_H