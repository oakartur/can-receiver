#include "ringbuf.h"
#include <stdlib.h>
#include <string.h>

bool ringbuf_init(ringbuf_t *rb, size_t elem_size, size_t capacity) {
  if (!rb || elem_size == 0 || capacity == 0) return false;

  rb->buf = (uint8_t *)malloc(elem_size * capacity);
  if (!rb->buf) return false;

  rb->elem_size = elem_size;
  rb->capacity  = capacity;
  rb->head      = 0;
  rb->count     = 0;
  return true;
}

void ringbuf_free(ringbuf_t *rb) {
  if (!rb) return;
  free(rb->buf);
  rb->buf = NULL;
  rb->elem_size = rb->capacity = rb->head = rb->count = 0;
}

bool ringbuf_push(ringbuf_t *rb, const void *elem) {
  if (!rb || !rb->buf || !elem) return false;

  memcpy(rb->buf + (rb->head * rb->elem_size), elem, rb->elem_size);
  rb->head = (rb->head + 1) % rb->capacity;
  if (rb->count < rb->capacity) rb->count++;
  return true;
}

size_t ringbuf_count(const ringbuf_t *rb) {
  return rb ? rb->count : 0;
}

bool ringbuf_get_latest(const ringbuf_t *rb, void *out_elem) {
  if (!rb || !rb->buf || rb->count == 0 || !out_elem) return false;
  size_t idx = (rb->head == 0) ? (rb->capacity - 1) : (rb->head - 1);
  memcpy(out_elem, rb->buf + (idx * rb->elem_size), rb->elem_size);
  return true;
}
