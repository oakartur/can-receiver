#ifndef COMPONENTS_MCP2515_MCP2515_H
#define COMPONENTS_MCP2515_MCP2515_H
#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"

typedef enum {
  MCP_OSC_8MHZ = 8000000,
} mcp_osc_t;

typedef enum {
  MCP_BITRATE_250K,
} mcp_bitrate_t;

typedef struct {
  spi_host_device_t spi_host;
  int pin_sck;
  int pin_mosi;
  int pin_miso;
  int pin_cs;
  int pin_int;

  mcp_osc_t    osc_hz;
  mcp_bitrate_t bitrate;

  bool rx_filter_pgn_fe05_only; // aceita só 0x18FE05xx (ignora SA)
} mcp2515_cfg_t;

typedef struct {
  uint32_t id;       // 29-bit quando ext=true
  bool     ext;
  uint8_t  dlc;
  uint8_t  data[8];
} can_frame_t;

bool mcp2515_init(const mcp2515_cfg_t *cfg);
bool mcp2515_read_frame(can_frame_t *out);

// Diagnóstico/recuperação
uint8_t mcp2515_read_eflg(void);
uint8_t mcp2515_read_canintf(void);
void    mcp2515_clear_rx_overflow(void);
bool    mcp2515_recover_reinit(void);

// Timestamp de última recepção (ms desde boot)
uint32_t mcp2515_last_rx_ms(void);

#endif  // COMPONENTS_MCP2515_MCP2515_H
