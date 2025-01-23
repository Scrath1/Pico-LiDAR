#ifndef SIGNAL_H
#define SIGNAL_H

#include <FreeRTOS.h>
#include <task.h>

#include <cstdint>
#ifdef SIGNAL_USE_MUTEX
    #include "semphr.h"
#endif  // SIGNAL_USE_MUTEX

template <typename T>
struct signal {
    // Stores when the signal was last updated. Based on OS timestamp
    uint32_t updateTimestamp_ms;
    // Counter for how often a write failed because the signal was locked
    // or not initialized
    uint32_t writeErrCnt;
    // Counter for how often a read failed because the signal was locked
    // or not initialized
    uint32_t readErrCnt;
    T data;
#ifdef SIGNAL_USE_MUTEX
    SemaphoreHandle_t mutex;
#endif  // SIGNAL_USE_MUTEX

    bool init() {
        updateTimestamp_ms = 0;
        writeErrCnt = 0;
        readErrCnt = 0;
        data = T();
#ifdef SIGNAL_USE_MUTEX
        if(NULL == mutex) this->mutex = xSemaphoreCreateMutex();

        return mutex != NULL;
#else
        return true;
#endif
    }

    bool write(T d, uint32_t timeout_ms) {
#ifdef SIGNAL_USE_MUTEX
        if(NULL == mutex) {
            writeErrCnt++;
            return false;
        }
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
        writeErrCnt++;
        return false;
    }

    bool writeFromISR(T d, BaseType_t* higherPriorityTaskWoken) {
#ifdef SIGNAL_USE_MUTEX
        if(NULL == mutex) return false;
        if(pdTRUE == xSemaphoreTakeFromISR(this->mutex, higherPriorityTaskWoken)) {
#else
        if(true) {
#endif  // SIGNAL_USE_MUTEX
            this->updateTimestamp_ms = xTaskGetTickCountFromISR();
            this->data = d;
#ifdef SIGNAL_USE_MUTEX
            xSemaphoreGiveFromISR(this->mutex, higherPriorityTaskWoken);
#endif  // SIGNAL_USE_MUTEX
            return true;
        }
        writeErrCnt++;
        return false;
    }

    T read(bool& successful, uint32_t timeout_ms) {
        T out;
#ifdef SIGNAL_USE_MUTEX
        if(NULL == mutex) {
            readErrCnt++;
            successful = false;
            return out;
        }
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
            readErrCnt++;
            successful = false;
        }
        return out;
    }

    T read(bool& successful, uint32_t timeout_ms, uint32_t& signalAge_ms){
        T out;
#ifdef SIGNAL_USE_MUTEX
        if(NULL == mutex) {
            readErrCnt++;
            successful = false;
            return out;
        }
        if(pdTRUE == xSemaphoreTake(this->mutex, pdMS_TO_TICKS(timeout_ms))) {
#else
        if(true) {
#endif  // SIGNAL_USE_MUTEX
            out = this->data;
            successful = true;
            signalAge_ms = xTaskGetTickCount() - this->updateTimestamp_ms;
#ifdef SIGNAL_USE_MUTEX
            xSemaphoreGive(this->mutex);
#endif  // SIGNAL_USE_MUTEX
        } else {
            readErrCnt++;
            successful = false;
        }
        return out;
    }
};
#endif  // SIGNAL_H