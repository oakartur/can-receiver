#include "j1939_decode.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "j1939";

// PGN 65029 = 0xFE05 (Generator Total AC Power)
// ID típico: 0x18FE052C (priority=6, PF=0xFE, PS=0x05, SA=0x2C)
// Vamos aceitar qualquer SA, mas exigir PF/PS.
#define J1939_PGN_FE05 0xFE05

static uint32_t j1939_extract_pgn(uint32_t can_id_29) {
  // 29-bit ID: [28:26]=prio, [25]=R, [24]=DP, [23:16]=PF, [15:8]=PS, [7:0]=SA
  uint32_t pf = (can_id_29 >> 16) & 0xFF;
  uint32_t ps = (can_id_29 >>  8) & 0xFF;
  uint32_t dp = (can_id_29 >> 24) & 0x01;

  // Para PF >= 240, PGN inclui PS (PDU2). Para PF < 240, PS é destino e PGN zera PS (PDU1).
  uint32_t pgn;
  if (pf < 240) {
    pgn = (dp << 16) | (pf << 8);
  } else {
    pgn = (dp << 16) | (pf << 8) | ps;
  }
  return pgn;
}

static int32_t le_i32(const uint8_t *p) {
  // little-endian signed 32
  return (int32_t)((uint32_t)p[0] |
                   ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) |
                   ((uint32_t)p[3] << 24));
}

typedef struct {
  QueueHandle_t in_q;
  generator_state_t *st;
  SemaphoreHandle_t mtx;
} ctx_t;

static void decode_task(void *arg) {
  ctx_t *ctx = (ctx_t *)arg;
  can_frame_t f;

  // running derivado por kW
  const float RUN_KW_THRESH = 0.5f;

  while (1) {
    if (xQueueReceive(ctx->in_q, &f, portMAX_DELAY) != pdTRUE) continue;

    if (!f.ext) continue; // J1939 é 29-bit

    uint32_t pgn = j1939_extract_pgn(f.id);
    if (pgn != J1939_PGN_FE05) continue;

    // Pelo manual, "Generator Total Real Power (SPN 2452)" em W.
    // Na prática, muitos mapas J1939 colocam potência em 4 bytes. Aqui assumimos i32 em W nos bytes 0..3.
    // Se seu documento indicar offset/byte diferente, ajusta aqui.
    if (f.dlc < 4) continue;

    int32_t watts = le_i32(&f.data[0]);
    float kw = (float)watts / 1000.0f;

    xSemaphoreTake(ctx->mtx, portMAX_DELAY);
    ctx->st->gen_total_kw = kw;
    ctx->st->running = (kw > RUN_KW_THRESH) || (kw < -RUN_KW_THRESH);
    ctx->st->last_update_ms = (uint32_t)esp_log_timestamp();
    ctx->st->frames_ok++;
    xSemaphoreGive(ctx->mtx);
  }
}

void j1939_decode_start(QueueHandle_t in_q, generator_state_t *state, SemaphoreHandle_t state_mtx) {
  static ctx_t s_ctx;
  memset(&s_ctx, 0, sizeof(s_ctx));
  s_ctx.in_q = in_q;
  s_ctx.st = state;
  s_ctx.mtx = state_mtx;

  xTaskCreatePinnedToCore(decode_task, "j1939_decode", 4096, &s_ctx, 8, NULL, 1);
  ESP_LOGI(TAG, "J1939 decode task started");
}
