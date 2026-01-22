#ifndef MAIN_RINGBUF_H
#define MAIN_RINGBUF_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint8_t *buf;
  size_t   elem_size;
  size_t   capacity;
  size_t   head;      // próxima escrita
  size_t   count;     // quantos válidos
} ringbuf_t;

bool ringbuf_init(ringbuf_t *rb, size_t elem_size, size_t capacity);
void ringbuf_free(ringbuf_t *rb);
bool ringbuf_push(ringbuf_t *rb, const void *elem);
size_t ringbuf_count(const ringbuf_t *rb);
bool ringbuf_get_latest(const ringbuf_t *rb, void *out_elem);

#endif  // MAIN_RINGBUF_H
