#ifndef CIRC_BUF_H_
#define CIRC_BUF_H_
#include <stdint.h>

typedef struct {
    uint8_t*  data;
    uint32_t maxlen;
    uint32_t head;
    uint32_t tail;
} circ_buf_t;

void circ_buf_init(circ_buf_t *circ_buf, uint8_t * buf, uint32_t max_length);

uint32_t circ_buf_write(circ_buf_t *buf, uint8_t data);

uint32_t circ_buf_read(circ_buf_t *buf, uint8_t *data);

uint32_t circ_buf_get_bytes_available(circ_buf_t *buf);
uint32_t circ_buf_bytes_in_buf(circ_buf_t *buf);

#endif /* CIRC_BUF_H_ */
