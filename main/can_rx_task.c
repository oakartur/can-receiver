#include "can_rx_task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/task.h"

static const char *TAG = "can_rx";

static TaskHandle_t s_can_task = NULL;
static QueueHandle_t s_out_q = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
  (void)arg;
  BaseType_t hpw = pdFALSE;
  if (s_can_task) {
    vTaskNotifyGiveFromISR(s_can_task, &hpw);
  }
  if (hpw) portYIELD_FROM_ISR();
}

static void can_rx_task(void *arg) {
  (void)arg;
  can_frame_t f;

  while (1) {
    // Acorda por interrupção (ou timeout, pra evitar travar em caso de bug)
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));

    // Enquanto houver frames pendentes, drena
    while (mcp2515_read_frame(&f)) {
      if (xQueueSend(s_out_q, &f, 0) != pdTRUE) {
        // fila cheia: drop
        ESP_LOGW(TAG, "RX queue full; dropping frame");
      }
    }
  }
}

void can_rx_task_start(const mcp2515_cfg_t *mcfg, QueueHandle_t out_q) {
  if (!mcfg || !out_q) return;

  s_out_q = out_q;

  // GPIO INT já configurado no mcp2515_init(), aqui só instala ISR
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  gpio_isr_handler_add(mcfg->pin_int, gpio_isr_handler, NULL);

  xTaskCreatePinnedToCore(can_rx_task, "can_rx_task", 4096, NULL, 10, &s_can_task, 1);
  ESP_LOGI(TAG, "CAN RX task started");
}
