#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* ===== Tipos ===== */

typedef struct {
    bool running;
    float gen_total_kw;
    uint32_t last_update_ms;
} generator_state_t;

typedef struct {
    uint16_t port;
    generator_state_t *state;
    SemaphoreHandle_t state_mtx;
} modbus_tcp_cfg_t;

/* ===== API ===== */

void modbus_tcp_start(const modbus_tcp_cfg_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_TCP_H */
