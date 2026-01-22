#include "can_watchdog.h"
#include "mcp2515.h"
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "can_wd";

static void wd_task(void *arg) {
  can_watchdog_cfg_t cfg = *(can_watchdog_cfg_t *)arg;

  uint32_t resets = 0;
  uint32_t last_recover_ms = 0;

  // Inicializa timestamp base
  if (mcp2515_last_rx_ms() == 0) {
    // se não recebeu ainda, define agora
    // (mcp2515_recover_reinit também faz isso, mas evitamos já resetar)
  }

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(cfg.poll_period_ms));

    uint32_t now = (uint32_t)esp_log_timestamp();
    uint32_t last_rx = mcp2515_last_rx_ms();
    uint8_t eflg = mcp2515_read_eflg();

    bool bus_off = (eflg & 0x20) != 0;      // TXBO
    bool err_passive = (eflg & 0x10) != 0;  // TXEP
    bool rx_over = (eflg & 0xC0) != 0;      // RX0OVR/RX1OVR
    bool no_rx = (last_rx != 0) ? ((now - last_rx) > cfg.no_rx_timeout_ms)
                                : ((now > cfg.no_rx_timeout_ms)); // desde boot

    if (rx_over) {
      ESP_LOGW(TAG, "RX overflow detected (EFLG=0x%02X). Clearing overflow flags.", eflg);
      mcp2515_clear_rx_overflow();
      // overflow não é motivo automático para reset total; vamos só limpar
      continue;
    }

    if (bus_off || err_passive || no_rx) {
      // Evita bater em loop se acabou de tentar
      if ((now - last_recover_ms) < cfg.poll_period_ms) {
        continue;
      }

      ESP_LOGW(TAG,
               "CAN recovery trigger: bus_off=%d err_passive=%d no_rx=%d (EFLG=0x%02X last_rx=%ums ago)",
               bus_off, err_passive, no_rx, eflg, (last_rx == 0) ? 0 : (now - last_rx));

      bool ok = mcp2515_recover_reinit();
      last_recover_ms = now;

      if (ok) {
        resets++;
        ESP_LOGW(TAG, "MCP2515 reinit OK. resets=%u", (unsigned)resets);

        // Opcional: refletir no estado
        if (cfg.state && cfg.state_mtx) {
          xSemaphoreTake(cfg.state_mtx, portMAX_DELAY);
          // você pode criar campos como "can_resets" depois
          // cfg.state->frames_drop += 0; // placeholder
          xSemaphoreGive(cfg.state_mtx);
        }

        // Backoff progressivo se estiver resetando sem parar
        if (resets >= cfg.max_resets_before_backoff) {
          uint32_t backoff = cfg.backoff_base_ms * (resets - cfg.max_resets_before_backoff + 1);
          if (backoff > 15000) backoff = 15000; // teto
          ESP_LOGW(TAG, "Applying backoff: %ums", (unsigned)backoff);
          vTaskDelay(pdMS_TO_TICKS(backoff));
        }
      } else {
        ESP_LOGE(TAG, "MCP2515 reinit FAILED. Will retry.");
        // pequeno delay para não travar CPU
        vTaskDelay(pdMS_TO_TICKS(500));
      }
    } else {
      // tudo ok -> zera contador de resets progressivo após período estável
      // Regra simples: se recebeu frame recente, reduz resets gradualmente
      if (resets > 0 && (now - last_rx) < (cfg.no_rx_timeout_ms / 2)) {
        resets--;
      }
    }
  }
}

void can_watchdog_start(const can_watchdog_cfg_t *cfg) {
  static can_watchdog_cfg_t s_cfg;
  s_cfg = *cfg;
  xTaskCreatePinnedToCore(wd_task, "can_watchdog", 4096, &s_cfg, 7, NULL, 1);
}
