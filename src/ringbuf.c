/*
 * ringbuf.c
 *
 *  Created on: Aug 9, 2013
 *      Author: petera
 */

#include "ringbuf.h"
#include "miniutils.h"

void ringbuf_init(ringbuf *rb, u8_t *buffer, u16_t max_len) {
  rb->max_len = max_len;
  rb->buffer = buffer;
  rb->r_ix = 0;
  rb->w_ix = 0;
  rb->len = 0;
}

int ringbuf_getc(ringbuf *rb, u8_t *c) {
  if (rb->len == 0) {
    return RB_ERR_EMPTY;
  }
  *c = rb->buffer[rb->r_ix];
  if (rb->r_ix == rb->max_len-1) {
    rb->r_ix = 0;
  } else {
    rb->r_ix++;
  }
  rb->len--;
  return RB_OK;
}


int ringbuf_putc(ringbuf *rb, u8_t c) {
  if (rb->len == rb->max_len) {
    return RB_ERR_FULL;
  }
  rb->buffer[rb->w_ix] = c;
  if (rb->w_ix == rb->max_len-1) {
    rb->w_ix = 0;
  } else {
    rb->w_ix++;
  }
  rb->len++;
  return RB_OK;
}

int ringbuf_available(ringbuf *rb) {
  return rb->len;
}

int ringbuf_free(ringbuf *rb) {
  return rb->max_len - rb->len;
}

int ringbuf_put(ringbuf *rb, u8_t *buf, u16_t len) {
  int to_write;
  if (rb->len == rb->max_len) {
    return RB_ERR_FULL;
  }
  int free = ringbuf_free(rb);
  if (len > free) {
    len = free;
  }
  to_write = len;
  if (rb->w_ix + len >= rb->max_len) {
    memcpy(&rb->buffer[rb->w_ix], buf, rb->max_len - rb->w_ix);
    rb->len += rb->max_len - rb->w_ix;
    buf += rb->max_len - rb->w_ix;
    to_write -= rb->max_len - rb->w_ix;
    rb->w_ix = 0;
  }
  memcpy(&rb->buffer[rb->w_ix], buf, to_write);
  rb->len += to_write;
  rb->w_ix += to_write;
  if (rb->w_ix == rb->max_len-1) {
    rb->w_ix = 0;
  }

  return len;
}

int ringbuf_get(ringbuf *rb, u8_t *buf, u16_t len) {
  int to_read;
  if (rb->len == 0) {
    return RB_ERR_EMPTY;
  }
  int avail = ringbuf_available(rb);
  if (len > avail) {
    len = avail;
  }
  to_read = len;
  if (rb->r_ix + len >= rb->max_len) {
    memcpy(buf, &rb->buffer[rb->r_ix], rb->max_len - rb->r_ix);
    rb->len -= rb->max_len - rb->r_ix;
    buf += rb->max_len - rb->r_ix;
    to_read -= rb->max_len - rb->r_ix;
    rb->r_ix = 0;
  }
  memcpy(buf, &rb->buffer[rb->r_ix], to_read);
  rb->len += to_read;
  rb->r_ix += to_read;
  if (rb->r_ix == rb->max_len-1) {
    rb->r_ix = 0;
  }

  return len;
}

