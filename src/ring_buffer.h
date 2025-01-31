#ifndef RING_BUFFER_H
#define RING_BUFFER_H
#ifndef __STATIC_INLINE
    #define __STATIC_INLINE static inline
#endif // __STATIC_INLINE

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <semphr.h>

#include "return_codes.h"

#define BUFFER_ITEM_TYPE char

/**
* Usage:
* 1. Call RING_BUFFER_DEF(ringbuf,ringbufSize)
* 2. Use ring_buffer_put(&ringbuf, input)
*/

/**
 * Used to allocate ringbuffer object.
 * @warning Actual buffer capacity is sz-1
 */
#define RING_BUFFER_DEF(x, sz) \
    BUFFER_ITEM_TYPE x##_data[sz];      \
    ring_buffer_t x = {        \
        .buffer = x##_data,    \
        .head = 0,             \
        .tail = 0,             \
        .len = sz}

typedef struct ring_buffer
{
    BUFFER_ITEM_TYPE *buffer;
    // Index where the next element to be added will be stored
    volatile uint32_t head;
    // Index of oldest element in ringbuffer. If there is no data
    // in the buffer, this points at the same idx as head.
    volatile uint32_t tail;
    // Total buffer array length. Actual ringbuffer capacity is len-1
    const uint32_t len;
    SemaphoreHandle_t mutex;
} ring_buffer_t;

/**
 * @brief Initializes ringbuffer struct by setting head and tail to 0.
 *  Can also be used to reset a ringbuffer object to 0. Does not actually
 *  clear the memory contents though.
 * @param rb [IN] Pointer to ringbuffer object
 * @return RC_SUCCESS on success,
 * @return RC_ERROR_NULL if mutex could not be initialized
 */
RC_t ring_buffer_init(ring_buffer_t *const rb);

/**
 * @brief Adds a single byte to the ringbuffer
 * @param rb [IN] Pointer to ringbuffer object
 * @param b [IN] Byte to add
 * @return RC_SUCCESS on success
 */
RC_t ring_buffer_put(ring_buffer_t *const rb, BUFFER_ITEM_TYPE b, uint32_t timeout_ms);

/**
 * @brief Removes a single byte from the ringbuffer and returns it to the user,
 *  if a byte is available in the buffer
 * @param rb [IN] Pointer to ringbuffer object
 * @param bPtr [IN] Pointer to variable where the removed byte should be stored
 * @param timeout_ms [IN] Maximum time for acquisition of mutex before returning error
 * @return RC_SUCCESS on success
 * @return RC_BUFFER_EMPTY if there is no data to retrieve
 * @return RC_ERROR_BUSY if the mutex lock could not be acquired
 */
RC_t ring_buffer_get(ring_buffer_t *const rb, BUFFER_ITEM_TYPE *const bPtr, uint32_t timeout_ms);

/**
 * @brief Checks whether the ringbuffer is empty
 * @param rb [IN] Pointer to ringbuffer object
 * @return true if buffer is empty
 */
static inline bool ring_buffer_is_empty(ring_buffer_t *const rb)
{
    if(NULL == rb->mutex) return true;
    xSemaphoreTakeRecursive(rb->mutex, portMAX_DELAY);
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    bool ret = (head == tail);
    xSemaphoreGiveRecursive(rb->mutex);
    return ret;
}

/**
 * @brief Checks whether the ringbuffer is full
 * @param rb [IN] Pointer to ringbuffer object
 * @return true if buffer is full
 */
static inline bool ring_buffer_is_full(ring_buffer_t *const rb)
{
    if(NULL == rb->mutex) return true;
    xSemaphoreTakeRecursive(rb->mutex, portMAX_DELAY);
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    bool ret = (((head + 1) % rb->len) == tail);
    xSemaphoreGiveRecursive(rb->mutex);
    return ret;
}

/**
 * @brief Returns number of bytes currently stored in ringbuffer
 * @param rb [IN] Pointer to ringbuffer object
 */
static inline uint32_t ring_buffer_avail(ring_buffer_t *const rb)
{
    if(NULL == rb->mutex) return 0;
    xSemaphoreTakeRecursive(rb->mutex, portMAX_DELAY);
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    bool ret = (head - tail) % rb->len;
    xSemaphoreGiveRecursive(rb->mutex);
    return ret;
}
#ifdef __cplusplus
}
#endif
#endif