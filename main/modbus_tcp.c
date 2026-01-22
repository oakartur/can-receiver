#include "modbus_tcp.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <string.h>
#include <errno.h>
#include <math.h>

static const char *TAG = "modbus_tcp";

// Mapa Modbus (exemplo):
// Coils:
//   00001: running (1=ligado)
// Holding Registers:
//   40001: gen_total_kw * 10 (signed int16)  -> 1 casa decimal
//   40002: last_update_ms low16
//   40003: last_update_ms high16

static uint16_t u16_be(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static void put_u16_be(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)(v & 0xFF); }

static int16_t clamp_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static void build_regs(const modbus_tcp_cfg_t *cfg, uint16_t addr_1based, uint16_t qty, uint16_t *out) {
  // addr_1based: 40001 => offset 0
  // Vamos trabalhar como holding offset 0..
  uint16_t off = addr_1based - 40001;

  generator_state_t st;
  xSemaphoreTake(cfg->state_mtx, portMAX_DELAY);
  st = *cfg->state;
  xSemaphoreGive(cfg->state_mtx);

  for (uint16_t i = 0; i < qty; i++) {
    uint16_t r = off + i;
    uint16_t val = 0;

    if (r == 0) {
      // kW * 10 em int16
      int32_t scaled = (int32_t)lroundf(st.gen_total_kw * 10.0f);
      val = (uint16_t)(int16_t)clamp_i16(scaled);
    } else if (r == 1) {
      val = (uint16_t)(st.last_update_ms & 0xFFFF);
    } else if (r == 2) {
      val = (uint16_t)((st.last_update_ms >> 16) & 0xFFFF);
    } else {
      val = 0;
    }
    out[i] = val;
  }
}

static uint8_t build_coils(const modbus_tcp_cfg_t *cfg, uint16_t coil_addr_1based, uint16_t qty, uint8_t *out_bytes) {
  // coil_addr_1based: 00001 => offset 0
  memset(out_bytes, 0, (qty + 7) / 8);

  generator_state_t st;
  xSemaphoreTake(cfg->state_mtx, portMAX_DELAY);
  st = *cfg->state;
  xSemaphoreGive(cfg->state_mtx);

  uint16_t off = coil_addr_1based - 1;

  for (uint16_t i = 0; i < qty; i++) {
    uint16_t c = off + i;
    bool bit = false;
    if (c == 0) bit = st.running;

    if (bit) {
      out_bytes[i / 8] |= (1u << (i % 8));
    }
  }
  return (uint8_t)((qty + 7) / 8);
}

static void modbus_tcp_task(void *arg) {
  modbus_tcp_cfg_t cfg = *(modbus_tcp_cfg_t *)arg;

  int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (listen_fd < 0) {
    ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
    vTaskDelete(NULL);
    return;
  }

  int yes = 1;
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(cfg.port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
    ESP_LOGE(TAG, "bind() failed: errno=%d", errno);
    close(listen_fd);
    vTaskDelete(NULL);
    return;
  }

  if (listen(listen_fd, 2) != 0) {
    ESP_LOGE(TAG, "listen() failed: errno=%d", errno);
    close(listen_fd);
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "Modbus TCP listening on port %u", cfg.port);

  while (1) {
    struct sockaddr_in6 source_addr;
    socklen_t socklen = sizeof(source_addr);
    int sock = accept(listen_fd, (struct sockaddr *)&source_addr, &socklen);
    if (sock < 0) {
      ESP_LOGE(TAG, "accept() failed: errno=%d", errno);
      continue;
    }

    ESP_LOGI(TAG, "Client connected");

    while (1) {
      uint8_t req[260];
      int rlen = recv(sock, req, sizeof(req), 0);
      if (rlen <= 0) break;

      // MBAP (7 bytes): TID(2) PID(2)=0 LEN(2) UID(1)
      if (rlen < 8) continue;

      uint16_t tid = u16_be(&req[0]);
      uint16_t pid = u16_be(&req[2]);
      uint16_t len = u16_be(&req[4]);
      uint8_t uid = req[6];
      uint8_t fcode = req[7];

      if (pid != 0) continue;
      // len inclui UID+PDU
      if (len < 2) continue;

      // PDU:
      // FC3: [7]=3, [8..9]=addr, [10..11]=qty
      // FC1: idem
      if (fcode != 3 && fcode != 1) {
        // exception: Illegal Function
        uint8_t resp[9];
        put_u16_be(&resp[0], tid);
        put_u16_be(&resp[2], 0);
        put_u16_be(&resp[4], 3); // UID + FC + EX
        resp[6] = uid;
        resp[7] = (uint8_t)(fcode | 0x80);
        resp[8] = 0x01;
        send(sock, resp, sizeof(resp), 0);
        continue;
      }

      if (rlen < 12) continue;

      uint16_t addr0 = u16_be(&req[8]);   // 0-based do Modbus
      uint16_t qty   = u16_be(&req[10]);

      // Endereçamento humano (1-based):
      // Holding: 40001 corresponde addr0=0
      // Coil: 00001 corresponde addr0=0
      if (qty == 0 || qty > 125) continue;

      uint8_t resp[260];
      memset(resp, 0, sizeof(resp));

      // Resposta base
      put_u16_be(&resp[0], tid);
      put_u16_be(&resp[2], 0);
      resp[6] = uid;
      resp[7] = fcode;

      if (fcode == 3) {
        // Holding regs
        // Se você preferir 40001, use addr_1based = 40001 + addr0
        uint16_t addr_1based = (uint16_t)(40001 + addr0);

        uint16_t regs[125];
        build_regs(&cfg, addr_1based, qty, regs);

        resp[8] = (uint8_t)(qty * 2); // byte count
        for (uint16_t i = 0; i < qty; i++) {
          put_u16_be(&resp[9 + i * 2], regs[i]);
        }

        uint16_t out_len = (uint16_t)(3 + qty * 2); // UID+FC+BC+data
        put_u16_be(&resp[4], out_len);
        send(sock, resp, (int)(7 + out_len), 0);

      } else if (fcode == 1) {
        // Coils
        uint16_t addr_1based = (uint16_t)(1 + addr0);
        uint8_t coil_bytes[32];
        uint8_t bc = build_coils(&cfg, addr_1based, qty, coil_bytes);

        resp[8] = bc;
        memcpy(&resp[9], coil_bytes, bc);

        uint16_t out_len = (uint16_t)(3 + bc); // UID+FC+BC + data
        put_u16_be(&resp[4], out_len);
        send(sock, resp, (int)(7 + out_len), 0);
      }
    }

    ESP_LOGI(TAG, "Client disconnected");
    close(sock);
  }
}

void modbus_tcp_start(const modbus_tcp_cfg_t *cfg) {
  static modbus_tcp_cfg_t s_cfg;
  s_cfg = *cfg;
  xTaskCreatePinnedToCore(modbus_tcp_task, "modbus_tcp", 6144, &s_cfg, 6, NULL, 0);
}
