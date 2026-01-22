#pragma once
#include <stdint.h>
#include "freertos/semphr.h"
#include "j1939_decode.h"

typedef struct {
  uint16_t port;
  generator_state_t *state;
  SemaphoreHandle_t state_mtx;
} modbus_tcp_cfg_t;

void modbus_tcp_start(const modbus_tcp_cfg_t *cfg);
