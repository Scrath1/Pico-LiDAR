#ifndef AVERAGING_FILTER_H
#define AVERAGING_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#define AVERAGING_FILTER_DEF(name, len) \
    uint32_t name##_data[len]; \
    averaging_filter_t name = {\
        .buffer = name##_data, \
        .nextIdx = 0, \
        .size = len}

typedef struct averaging_filter {
    uint32_t *buffer;
    uint32_t nextIdx;
    const uint32_t size;
} averaging_filter_t;

static inline void averaging_filter_init(averaging_filter_t* filter){
    for(uint32_t i = 0; i < filter->size; i++){
        filter->buffer[i] = 0;
    }
}

static inline void averaging_filter_put(averaging_filter_t* filter, uint32_t val){
    filter->buffer[filter->nextIdx] = val;
    filter->nextIdx = (filter->nextIdx + 1) % filter->size;
}

static inline uint32_t averaging_filter_get_avg(const averaging_filter_t* filter){
    uint32_t sum = 0;
    for(uint32_t i = 0; i < filter->size; i++){
        sum += filter->buffer[i];
    }
    return sum / filter->size;
}

#ifdef __cplusplus
}
#endif
#endif // AVERAGING_FILTER_H