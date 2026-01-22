#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "wifi_mngr.h"
#include "ringbuf.h"
#include "can_rx_task.h"
#include "j1939_decode.h"
#include "modbus_tcp.h"
#include "mcp2515.h"
#include "can_watchdog.h"

static const char *TAG = "app_main";

// ====== Config do seu hardware ======
#define MCP_SPI_HOST        SPI2_HOST        // VSPI no ESP-IDF atual costuma ser SPI2_HOST
#define MCP_PIN_SCK         18
#define MCP_PIN_MOSI        23
#define MCP_PIN_MISO        19
#define MCP_PIN_CS          25
#define MCP_PIN_INT         26

// ====== Ajuste de filas/buffers ======
#define CAN_RX_QUEUE_LEN    64
#define STATE_RING_CAP      300   // 5 min @1Hz

typedef struct {
  uint32_t ts_ms;
  float    kw;
  bool     running;
} generator_sample_t;

static QueueHandle_t g_can_rx_q = NULL;
static ringbuf_t     g_state_rb;

static generator_state_t g_state;
static SemaphoreHandle_t g_state_mtx = NULL;

static void sampler_task(void *arg) {
  (void)arg;

  TickType_t last = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));

    generator_sample_t s;
    s.ts_ms = (uint32_t)(esp_log_timestamp());

    xSemaphoreTake(g_state_mtx, portMAX_DELAY);
    s.kw = g_state.gen_total_kw;
    s.running = g_state.running;
    xSemaphoreGive(g_state_mtx);

    ringbuf_push(&g_state_rb, &s);
  }
}

void app_main(void) {
  esp_err_t err;

  // NVS (Wi-Fi)
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  g_state_mtx = xSemaphoreCreateMutex();
  if (!g_state_mtx) {
    ESP_LOGE(TAG, "Failed to create state mutex");
    return;
  }

  // Ring buffer na heap
  if (!ringbuf_init(&g_state_rb, sizeof(generator_sample_t), STATE_RING_CAP)) {
    ESP_LOGE(TAG, "Failed to init ring buffer");
    return;
  }

  // Wi-Fi
  wifi_mgr_init();
  wifi_mgr_start_sta();

  // MCP2515 init (8MHz, 250kbps, filtro PGN 0xFE05)
  mcp2515_cfg_t mcfg = {
    .spi_host = MCP_SPI_HOST,
    .pin_sck  = MCP_PIN_SCK,
    .pin_mosi = MCP_PIN_MOSI,
    .pin_miso = MCP_PIN_MISO,
    .pin_cs   = MCP_PIN_CS,
    .pin_int  = MCP_PIN_INT,
    .osc_hz   = MCP_OSC_8MHZ,
    .bitrate  = MCP_BITRATE_250K,
    .rx_filter_pgn_fe05_only = true, // aceita só 0x18FE05xx
  };

  if (!mcp2515_init(&mcfg)) {
    ESP_LOGE(TAG, "MCP2515 init failed");
    return;
  }

  // Fila CAN
  g_can_rx_q = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(can_frame_t));
  if (!g_can_rx_q) {
    ESP_LOGE(TAG, "Failed to create CAN RX queue");
    return;
  }

  // Task CAN RX (ISR via GPIO INT)
  can_rx_task_start(&mcfg, g_can_rx_q);

  // Task decode J1939
  j1939_decode_start(g_can_rx_q, &g_state, g_state_mtx);

  // Sampler (1Hz) grava ring buffer
  xTaskCreatePinnedToCore(sampler_task, "sampler", 3072, NULL, 5, NULL, 1);
  
  can_watchdog_cfg_t wdcfg = {
    .no_rx_timeout_ms = 3000,          // 3s sem RX -> tenta recuperar
    .poll_period_ms = 500,             // checa 2x por segundo
    .max_resets_before_backoff = 3,    // depois de 3 resets seguidos, começa backoff
    .backoff_base_ms = 2000,           // 2s, 4s, 6s...
    .state = &g_state,
    .state_mtx = g_state_mtx,
  };
  can_watchdog_start(&wdcfg);

  // Modbus TCP
  modbus_tcp_cfg_t mbcfg = {
    .port = 502,
    .state = &g_state,
    .state_mtx = g_state_mtx,
  };
  modbus_tcp_start(&mbcfg);

  ESP_LOGI(TAG, "System started.");
}
