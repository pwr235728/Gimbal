#include "circ_buf.h"

void circ_buf_init(circ_buf_t *circ_buf, uint8_t * buf, uint32_t max_length)
{
	circ_buf->data = buf;
	circ_buf->maxlen = max_length;
	circ_buf->head = 0;
	circ_buf->tail = 0;
}

uint32_t circ_buf_write(circ_buf_t *buf, uint8_t data)
{
    uint32_t next;

    next = buf->head + 1;  // next is where head will point to after this write.
    if (next >= buf->maxlen)
        next = 0;

    if (next == buf->tail)  // if the head + 1 == tail, circular buffer is full
        return -1;

    buf->data[buf->head] = data;  // Load data and then move
    buf->head = next;             // head to next data offset.
    return 0;  // return success to indicate successful push.
}

uint32_t circ_buf_read(circ_buf_t *buf, uint8_t *data)
{
	 uint32_t next;

    if (buf->head == buf->tail)  // if the head == tail, we don't have any data
        return -1;

    next = buf->tail + 1;  // next is where tail will point to after this read.
    if(next >= buf->maxlen)
        next = 0;

    *data = buf->data[buf->tail];  // Read data and then move
    buf->tail = next;              // tail to next offset.
    return 0;  // return success to indicate successful push.
}

uint32_t circ_buf_get_bytes_available(circ_buf_t *buf)
{
    int32_t bytes_available;
    bytes_available = buf->tail - buf->head;
    if (bytes_available <= 0)
    	bytes_available += buf->maxlen;
    return bytes_available - 1; // -1 to account for the always-empty slot.
}

uint32_t circ_buf_bytes_in_buf(circ_buf_t *buf)
{
	 return buf->maxlen - 1 - circ_buf_get_bytes_available(rx_circ_buf);
}
