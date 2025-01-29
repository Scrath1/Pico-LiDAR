#ifndef SETTING_TEMPLATE_H
#define SETTING_TEMPLATE_H
#include <cstdint>
#include <cmath>
#include <FreeRTOS.h>
#include <task.h>

#define SETTING_NAME_MAX_LEN (20)
#define FLOAT_EQUALITY_THRESHOLD (0.000005)

template<typename T>
struct setting {
    T value;
    uint32_t lastChanged_ms;
    const char* name;

    setting() : name(""){}
    setting(T v, const char* n) : value(v), lastChanged_ms(0), name(n) {}

    void set(T val) {
        this->value = val;
        this->lastChanged_ms = xTaskGetTickCount();
    }

    void setFromISR(T val) {
        this->value = val;
        this->lastChanged_ms = xTaskGetTickCountFromISR();
    }

    T get() {
        return this->value;
    }

    setting& operator=(const setting& other) {
        if(this == &other) return *this;

        this->value = other.value;
        this->lastChanged_ms = other.lastChanged_ms;
        return *this;
    }

    setting& operator=(const T& val) {
        this.set(val);
        return *this;
    }
};

template<typename T>
static inline bool operator==(const T& lhs, const T& rhs) {
    return (lhs.value == rhs.value) &&
        (lhs.lastChanged_ms == rhs.lastChanged_ms);
}

template<typename T>
static inline bool operator!=(const T& lhs, const T& rhs) {
    return !(lhs == rhs);
}

static inline bool areEqualAbs(float a, float b, float epsilon) {
    return (std::fabs(a - b) <= epsilon);
}

static inline bool operator==(const setting<float>& lhs, const setting<float>& rhs) {
    return (areEqualAbs(lhs.value, rhs.value, FLOAT_EQUALITY_THRESHOLD)) &&
        (lhs.lastChanged_ms == rhs.lastChanged_ms);
}

#endif //SETTING_TEMPLATE_H
