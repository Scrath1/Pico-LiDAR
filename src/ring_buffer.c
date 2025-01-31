#include "ring_buffer.h"

RC_t ring_buffer_init(ring_buffer_t *const rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->mutex = xSemaphoreCreateRecursiveMutex();
    if(rb->mutex == NULL) return RC_ERROR_NULL;
    else return RC_SUCCESS;
}

RC_t ring_buffer_put(ring_buffer_t *const rb, BUFFER_ITEM_TYPE c, uint32_t timeout_ms)
{
    if(NULL == rb->mutex) return RC_ERROR_NULL;
    RC_t ret = RC_ERROR;
    if(pdPASS == xSemaphoreTakeRecursive(rb->mutex, timeout_ms)){
        if(ring_buffer_is_full(rb))
        {
            // increment tail position to second oldest
            // byte since the oldest will now be overwritten
            rb->tail = (rb->tail + 1) % rb->len;
        }
        rb->buffer[rb->head] = c;
        rb->head = (rb->head + 1) % rb->len;
        xSemaphoreGiveRecursive(rb->mutex);
        ret = RC_SUCCESS;
    }
    else ret = RC_ERROR_BUSY;
    return ret;
}

RC_t ring_buffer_get(ring_buffer_t *const rb, BUFFER_ITEM_TYPE *const c, uint32_t timeout_ms)
{
    if(NULL == rb->mutex) return RC_ERROR_NULL;
    RC_t ret = RC_ERROR;
    if(pdPASS == xSemaphoreTakeRecursive(rb->mutex, pdMS_TO_TICKS(timeout_ms))){
        if(ring_buffer_is_empty(rb))
        {
            ret = RC_ERROR_BUFFER_EMPTY;
        }
        else{
            *c = rb->buffer[rb->tail];
            rb->tail = (rb->tail + 1) % rb->len;
            ret = RC_SUCCESS;
        }
        xSemaphoreGiveRecursive(rb->mutex);
    }
    else{
        ret = RC_ERROR_BUSY;
    }
    return ret;
}