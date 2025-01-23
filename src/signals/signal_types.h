#ifndef SIGNAL_TYPES_H
#define SIGNAL_TYPES_H
#include "signal_template.h"
#include "runtime_settings.h"

typedef struct{
    uint32_t rpm;
} rpm_data_t;

typedef signal<rpm_data_t> rpm_signal_t;
typedef signal<runtime_settings_t> runtime_settings_signal_t;

#endif // SIGNAL_TYPES_H