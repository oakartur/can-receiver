#ifndef MAIN_CAN_WATCHDOG_H
#define MAIN_CAN_WATCHDOG_H
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "j1939_decode.h"

typedef struct {
  // Se não receber nenhum frame por este tempo -> recuperação
  uint32_t no_rx_timeout_ms;     // ex: 3000

  // Intervalo de checagem
  uint32_t poll_period_ms;       // ex: 500

  // Quantos resets seguidos antes de aumentar backoff
  uint32_t max_resets_before_backoff;  // ex: 3

  // Backoff base (multiplica por tentativas)
  uint32_t backoff_base_ms;      // ex: 2000

  // Estado (opcional, para expor no Modbus futuramente)
  generator_state_t *state;
  SemaphoreHandle_t state_mtx;
} can_watchdog_cfg_t;

void can_watchdog_start(const can_watchdog_cfg_t *cfg);

#endif  // MAIN_CAN_WATCHDOG_H
