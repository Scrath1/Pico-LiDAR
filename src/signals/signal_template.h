#ifndef SIGNAL_H
#define SIGNAL_H

#include <task.h>

#include <cstdint>
#ifdef SIGNAL_USE_MUTEX
    #include "semphr.h"
#endif  // SIGNAL_USE_MUTEX

template <typename T>
struct signal {
    uint32_t updateTimestamp_ms;
    T data;
    SemaphoreHandle_t mutex;

    bool init() {
#ifdef SIGNAL_USE_MUTEX
        if(NULL == mutex) this->mutex = xSemaphoreCreateMutex();

        return mutex != NULL;
#else
        return true;
#endif
    }

    bool write(T d, uint32_t timeout_ms) {
#ifdef SIGNAL_USE_MUTEX
        if(NULL == mutex) return false;
#endif  // SIGNAL_USE_MUTEX

#ifdef SIGNAL_USE_MUTEX
        if(pdTRUE == xSemaphoreTake(this->mutex, timeout_ms)) {
#else
        if(true) {
#endif  // SIGNAL_USE_MUTEX
            this->updateTimestamp_ms = xTaskGetTickCount();
            this->data = d;
#ifdef SIGNAL_USE_MUTEX
            xSemaphoreGive(this->mutex);
#endif  // SIGNAL_USE_MUTEX
            return true;
        }
        return false;
    }

    T read(bool& successful, uint32_t timeout_ms) {
        T out;
#ifdef SIGNAL_USE_MUTEX
        if(NULL == mutex) {
            successful = false;
            return out;
        }
#endif  // SIGNAL_USE_MUTEX

#ifdef SIGNAL_USE_MUTEX
        if(pdTRUE == xSemaphoreTake(this->mutex, pdMS_TO_TICKS(timeout_ms))) {
#else
        if(true) {
#endif  // SIGNAL_USE_MUTEX
            out = this->data;
            successful = true;
#ifdef SIGNAL_USE_MUTEX
            xSemaphoreGive(this->mutex);
#endif  // SIGNAL_USE_MUTEX
        } else {
            successful = false;
        }
        return out;
    }
};
#endif  // SIGNAL_H