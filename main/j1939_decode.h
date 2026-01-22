#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "mcp2515.h"

typedef struct {
  float gen_total_kw;     // potência entregue (kW)
  bool  running;          // ligado/desligado derivado
  uint32_t last_update_ms;
  uint32_t frames_ok;
  uint32_t frames_drop;
} generator_state_t;

void j1939_decode_start(QueueHandle_t in_q, generator_state_t *state, SemaphoreHandle_t state_mtx);
