#include "wifi_mgr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/ip_addr.h"

static const char *TAG = "wifi_mgr";

#ifndef CONFIG_WIFI_SSID
#define CONFIG_WIFI_SSID "SET_IN_MENUCONFIG"
#endif

#ifndef CONFIG_WIFI_PASSWORD
#define CONFIG_WIFI_PASSWORD "SET_IN_MENUCONFIG"
#endif

static EventGroupHandle_t s_wifi_evt;
static const int WIFI_CONNECTED_BIT = BIT0;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  (void)arg;
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGW(TAG, "Disconnected; reconnecting...");
    esp_wifi_connect();
    xEventGroupClearBits(s_wifi_evt, WIFI_CONNECTED_BIT);
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    xEventGroupSetBits(s_wifi_evt, WIFI_CONNECTED_BIT);
  }
}

void wifi_mgr_init(void) {
  s_wifi_evt = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
}

void wifi_mgr_start_sta(void) {
  wifi_config_t wifi_config = {0};
  snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s", CONFIG_WIFI_SSID);
  snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s", CONFIG_WIFI_PASSWORD);
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Wi-Fi STA start: ssid=%s", CONFIG_WIFI_SSID);
}
