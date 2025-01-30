#include "ring_buffer.h"

void ring_buffer_init(ring_buffer_t *const rb)
{
    rb->head = 0;
    rb->tail = 0;
}

RC_t ring_buffer_put(ring_buffer_t *const rb, uint8_t c)
{
    if (ring_buffer_is_full(rb))
    {
        // increment tail position to second oldest
        // byte since the oldest will now be overwritten
        rb->tail = (rb->tail + 1) % rb->len;
    }

    rb->buffer[rb->head] = c;
    rb->head = (rb->head + 1) % rb->len;

    return RC_SUCCESS;
}

RC_t ring_buffer_get(ring_buffer_t *const rb, uint8_t *const c)
{
    if (ring_buffer_is_empty(rb))
    {
        return RC_ERROR_BUFFER_EMPTY;
    }

    *c = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->len;

    return RC_SUCCESS;
}