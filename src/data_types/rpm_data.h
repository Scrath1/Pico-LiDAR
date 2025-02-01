#ifndef RPM_DATA_H
#define RPM_DATA_H
#include <FreeRTOS.h>
#include <task.h>

#include <cstdint>

typedef struct {
    uint32_t rpm;
    uint32_t timestampLastChanged_ms;

    void set(uint32_t rpm) {
        this->rpm = rpm;
        timestampLastChanged_ms = xTaskGetTickCount();
    }

    void setFromISR(uint32_t rpm) {
        this->rpm = rpm;
        timestampLastChanged_ms = xTaskGetTickCountFromISR();
    }

    uint32_t getAge_ms() { return xTaskGetTickCount() - timestampLastChanged_ms; }
} rpm_data_t;

#endif  // RPM_DATA_H