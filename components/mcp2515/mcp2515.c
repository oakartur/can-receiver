#include "mcp2515.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "mcp2515";

// ===== MCP2515 SPI Commands =====
#define MCP_CMD_RESET        0xC0
#define MCP_CMD_READ         0x03
#define MCP_CMD_WRITE        0x02
#define MCP_CMD_BIT_MODIFY   0x05
#define MCP_CMD_READ_STATUS  0xA0
#define MCP_CMD_RX_STATUS    0xB0
#define MCP_CMD_RTS_TX0      0x81

// ===== Registers =====
#define MCP_CANSTAT          0x0E
#define MCP_CANCTRL          0x0F
#define MCP_CNF3             0x28
#define MCP_CNF2             0x29
#define MCP_CNF1             0x2A
#define MCP_CANINTE          0x2B
#define MCP_CANINTF          0x2C
#define MCP_EFLG             0x2D

#define MCP_TXB0CTRL         0x30
#define MCP_RXB0CTRL         0x60
#define MCP_RXB1CTRL         0x70

#define MCP_RXM0SIDH         0x20
#define MCP_RXM0SIDL         0x21
#define MCP_RXM0EID8         0x22
#define MCP_RXM0EID0         0x23

#define MCP_RXF0SIDH         0x00
#define MCP_RXF0SIDL         0x01
#define MCP_RXF0EID8         0x02
#define MCP_RXF0EID0         0x03

#define MCP_RXF1SIDH         0x04
#define MCP_RXF1SIDL         0x05
#define MCP_RXF1EID8         0x06
#define MCP_RXF1EID0         0x07

// RXB0 registers
#define MCP_RXB0SIDH         0x61
#define MCP_RXB0SIDL         0x62
#define MCP_RXB0EID8         0x63
#define MCP_RXB0EID0         0x64
#define MCP_RXB0DLC          0x65
#define MCP_RXB0D0           0x66

// RXB1 registers
#define MCP_RXB1SIDH         0x71
#define MCP_RXB1SIDL         0x72
#define MCP_RXB1EID8         0x73
#define MCP_RXB1EID0         0x74
#define MCP_RXB1DLC          0x75
#define MCP_RXB1D0           0x76

// ===== Bits =====
#define CANCTRL_REQOP_MASK   0xE0
#define CANCTRL_MODE_CONFIG  0x80
#define CANCTRL_MODE_NORMAL  0x00

#define CANINTE_RX0IE        0x01
#define CANINTE_RX1IE        0x02

#define CANINTF_RX0IF        0x01
#define CANINTF_RX1IF        0x02

#define RXBCTRL_RXM_MASK     0x60
#define RXBCTRL_RXM_ALL      0x60  // receive any

#define RXBCTRL_BUKT         0x04  // rollover

// ===== Static driver state =====
static spi_device_handle_t s_spi = NULL;
static mcp2515_cfg_t s_cfg;
static volatile uint32_t s_last_rx_ms = 0;


// ===== SPI helpers =====
static esp_err_t spi_txrx(const uint8_t *tx, uint8_t *rx, int len) {
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = len * 8;
  t.tx_buffer = tx;
  t.rx_buffer = rx;
  return spi_device_transmit(s_spi, &t);
}

static void mcp_reset(void) {
  uint8_t tx[1] = { MCP_CMD_RESET };
  spi_txrx(tx, NULL, 1);
}

static uint8_t mcp_read_reg(uint8_t addr) {
  uint8_t tx[3] = { MCP_CMD_READ, addr, 0x00 };
  uint8_t rx[3] = {0};
  spi_txrx(tx, rx, 3);
  return rx[2];
}

static void mcp_write_reg(uint8_t addr, uint8_t val) {
  uint8_t tx[3] = { MCP_CMD_WRITE, addr, val };
  spi_txrx(tx, NULL, 3);
}

static void mcp_write_regs(uint8_t addr, const uint8_t *vals, int n) {
  uint8_t tx[2 + 16];
  if (n > 16) n = 16;
  tx[0] = MCP_CMD_WRITE;
  tx[1] = addr;
  memcpy(&tx[2], vals, n);
  spi_txrx(tx, NULL, 2 + n);
}

static void mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t data) {
  uint8_t tx[4] = { MCP_CMD_BIT_MODIFY, addr, mask, data };
  spi_txrx(tx, NULL, 4);
}

static uint8_t mcp_read_status(void) {
  uint8_t tx[2] = { MCP_CMD_READ_STATUS, 0x00 };
  uint8_t rx[2] = {0};
  spi_txrx(tx, rx, 2);
  return rx[1];
}

// ===== CAN ID mapping for MCP2515 filters (extended 29-bit) =====
// MCP2515 stores: SIDH, SIDL, EID8, EID0
// For EXID:
// SID = bits 28..18 (11 bits)
// EID = bits 17..0 (18 bits)
// SIDL: bits 7..5 = SID[2..0], bit3=EXIDE, bits1..0=EID[17..16]
static void id29_to_regs(uint32_t id, uint8_t *sidh, uint8_t *sidl, uint8_t *eid8, uint8_t *eid0) {
  uint32_t sid = (id >> 18) & 0x7FF;
  uint32_t eid = id & 0x3FFFF;

  *sidh = (uint8_t)(sid >> 3);
  *sidl = (uint8_t)((sid & 0x07) << 5);
  *sidl |= 1 << 3; // EXIDE
  *sidl |= (uint8_t)((eid >> 16) & 0x03);
  *eid8 = (uint8_t)((eid >> 8) & 0xFF);
  *eid0 = (uint8_t)(eid & 0xFF);
}

static void set_mode(uint8_t mode) {
  mcp_bit_modify(MCP_CANCTRL, CANCTRL_REQOP_MASK, mode);
  // aguarda CANSTAT refletir o modo
  for (int i = 0; i < 50; i++) {
    uint8_t stat = mcp_read_reg(MCP_CANSTAT);
    if ((stat & CANCTRL_REQOP_MASK) == mode) return;
  }
}

// ===== Bitrate config: 8MHz, 250kbps =====
// Config conservadora (SJW=1, BRP=0, PropSeg/PS1/PS2 ajustados)
static bool set_bitrate_8mhz_250k(void) {
  // Uma configuração válida comum para 8MHz/250k:
  // TQ = 2*(BRP+1)/Fosc = 2/8MHz = 250ns (BRP=0)
  // Bit time 250kbps = 4us => 16 TQ
  // PropSeg=5, PS1=6, PS2=4 => 1+5+6+4 = 16
  // CNF1: SJW=1(00), BRP=0 => 0x00
  // CNF2: BTLMODE=1, SAM=0, PS1=6-1=5, PRSEG=5-1=4 => 0b1001 0100? => 0x94 com PS1=5, PRSEG=4
  // CNF3: PHSEG2=4-1=3, SOF=0, WAKFIL=0 => 0x03
  mcp_write_reg(MCP_CNF1, 0x00);
  mcp_write_reg(MCP_CNF2, 0x94);
  mcp_write_reg(MCP_CNF3, 0x03);
  return true;
}

static void setup_filters_fe05_only(void) {
  // Queremos aceitar 0x18FE05xx (ignorar SA).
  // Máscara: bits 28..8 = 1, bits 7..0 = 0
  // Como ID é 29 bits, máscara = 0x1FFFFF00
  uint32_t mask = 0x1FFFFF00;
  uint32_t filt = 0x18FE0500;

  uint8_t sidh, sidl, eid8, eid0;

  // RXM0
  id29_to_regs(mask, &sidh, &sidl, &eid8, &eid0);
  uint8_t m0[4] = {sidh, sidl, eid8, eid0};
  mcp_write_regs(MCP_RXM0SIDH, m0, 4);

  // RXF0
  id29_to_regs(filt, &sidh, &sidl, &eid8, &eid0);
  uint8_t f0[4] = {sidh, sidl, eid8, eid0};
  mcp_write_regs(MCP_RXF0SIDH, f0, 4);

  // RXF1 (mesmo filtro)
  mcp_write_regs(MCP_RXF1SIDH, f0, 4);

  // RXB0/RXB1: receber apenas msgs que passam filtros (RXM=00)
  // Se colocar RXM=11 recebe tudo. Então aqui não setamos RXM_ALL.
  // RXB0CTRL: BUKT habilita rollover
  mcp_write_reg(MCP_RXB0CTRL, RXBCTRL_BUKT);
  mcp_write_reg(MCP_RXB1CTRL, 0x00);
}

static void setup_receive_any(void) {
  // recebe qualquer frame
  mcp_write_reg(MCP_RXB0CTRL, RXBCTRL_RXM_ALL | RXBCTRL_BUKT);
  mcp_write_reg(MCP_RXB1CTRL, RXBCTRL_RXM_ALL);
}

bool mcp2515_init(const mcp2515_cfg_t *cfg) {
  if (!cfg) return false;
  s_cfg = *cfg;

  // SPI bus
  spi_bus_config_t buscfg = {
    .mosi_io_num = cfg->pin_mosi,
    .miso_io_num = cfg->pin_miso,
    .sclk_io_num = cfg->pin_sck,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 0
  };
  if (spi_bus_initialize(cfg->spi_host, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) {
    ESP_LOGE(TAG, "spi_bus_initialize failed");
    return false;
  }

  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 2 * 1000 * 1000, // 2 MHz (seguro)
    .mode = 0,
    .spics_io_num = cfg->pin_cs,
    .queue_size = 4,
  };

  if (spi_bus_add_device(cfg->spi_host, &devcfg, &s_spi) != ESP_OK) {
    ESP_LOGE(TAG, "spi_bus_add_device failed");
    return false;
  }

  // INT GPIO
  gpio_config_t io = {0};
  io.intr_type = GPIO_INTR_NEGEDGE;
  io.mode = GPIO_MODE_INPUT;
  io.pin_bit_mask = 1ULL << cfg->pin_int;
  io.pull_up_en = 1;
  gpio_config(&io);

  // Reset + modo config
  mcp_reset();
  // pequena espera
  for (volatile int i=0; i<200000; i++) { }

  set_mode(CANCTRL_MODE_CONFIG);

  // Bitrate
  if (cfg->osc_hz == MCP_OSC_8MHZ && cfg->bitrate == MCP_BITRATE_250K) {
    set_bitrate_8mhz_250k();
  } else {
    ESP_LOGE(TAG, "Unsupported osc/bitrate");
    return false;
  }

  // filtros
  if (cfg->rx_filter_pgn_fe05_only) {
    setup_filters_fe05_only();
  } else {
    setup_receive_any();
  }

  // Interrupts RX
  mcp_write_reg(MCP_CANINTE, CANINTE_RX0IE | CANINTE_RX1IE);

  // Clear flags
  mcp_write_reg(MCP_CANINTF, 0x00);

  // Normal mode
  set_mode(CANCTRL_MODE_NORMAL);

  ESP_LOGI(TAG, "MCP2515 init OK (8MHz/250k, filter=%s)",
           cfg->rx_filter_pgn_fe05_only ? "FE05-only" : "any");
  return true;
}

static bool read_rx_buf0(can_frame_t *out) {
  uint8_t sidh = mcp_read_reg(MCP_RXB0SIDH);
  uint8_t sidl = mcp_read_reg(MCP_RXB0SIDL);
  uint8_t eid8 = mcp_read_reg(MCP_RXB0EID8);
  uint8_t eid0 = mcp_read_reg(MCP_RXB0EID0);
  uint8_t dlc  = mcp_read_reg(MCP_RXB0DLC) & 0x0F;

  bool ext = (sidl & (1 << 3)) != 0;

  uint32_t id = 0;
  if (ext) {
    uint32_t sid = ((uint32_t)sidh << 3) | (sidl >> 5);
    uint32_t eid = ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
    id = (sid << 18) | eid;
  } else {
    // Standard 11-bit
    id = (((uint32_t)sidh << 3) | (sidl >> 5)) & 0x7FF;
  }

  out->id = id;
  out->ext = ext;
  out->dlc = (dlc > 8) ? 8 : dlc;

  for (int i = 0; i < out->dlc; i++) {
    out->data[i] = mcp_read_reg(MCP_RXB0D0 + i);
  }

  // clear flag
  mcp_bit_modify(MCP_CANINTF, CANINTF_RX0IF, 0x00);
  s_last_rx_ms = (uint32_t)esp_log_timestamp();
  return true;
}

static bool read_rx_buf1(can_frame_t *out) {
  uint8_t sidh = mcp_read_reg(MCP_RXB1SIDH);
  uint8_t sidl = mcp_read_reg(MCP_RXB1SIDL);
  uint8_t eid8 = mcp_read_reg(MCP_RXB1EID8);
  uint8_t eid0 = mcp_read_reg(MCP_RXB1EID0);
  uint8_t dlc  = mcp_read_reg(MCP_RXB1DLC) & 0x0F;

  bool ext = (sidl & (1 << 3)) != 0;

  uint32_t id = 0;
  if (ext) {
    uint32_t sid = ((uint32_t)sidh << 3) | (sidl >> 5);
    uint32_t eid = ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
    id = (sid << 18) | eid;
  } else {
    id = (((uint32_t)sidh << 3) | (sidl >> 5)) & 0x7FF;
  }

  out->id = id;
  out->ext = ext;
  out->dlc = (dlc > 8) ? 8 : dlc;

  for (int i = 0; i < out->dlc; i++) {
    out->data[i] = mcp_read_reg(MCP_RXB1D0 + i);
  }

  mcp_bit_modify(MCP_CANINTF, CANINTF_RX1IF, 0x00);
  s_last_rx_ms = (uint32_t)esp_log_timestamp();
  return true;
}

bool mcp2515_read_frame(can_frame_t *out) {
  if (!out) return false;

  uint8_t intf = mcp_read_reg(MCP_CANINTF);

  if (intf & CANINTF_RX0IF) {
    return read_rx_buf0(out);
  }
  if (intf & CANINTF_RX1IF) {
    return read_rx_buf1(out);
  }

  // fallback: RX status
  uint8_t st = mcp_read_status();
  (void)st;

  return false;
}

uint32_t mcp2515_last_rx_ms(void) {
  return s_last_rx_ms;
}

uint8_t mcp2515_read_eflg(void) {
  return mcp_read_reg(MCP_EFLG);
}

uint8_t mcp2515_read_canintf(void) {
  return mcp_read_reg(MCP_CANINTF);
}

void mcp2515_clear_rx_overflow(void) {
  // EFLG bits RX0OVR(6), RX1OVR(7)
  // Clearing EFLG: write 0 to those bits via bit modify
  mcp_bit_modify(MCP_EFLG, 0xC0, 0x00);
  // Também limpa CANINTF se ficou sujo
  mcp_bit_modify(MCP_CANINTF, 0x03, 0x00);
}

bool mcp2515_recover_reinit(void) {
  // Reaplica init completo com a configuração atual salva em s_cfg
  // Estratégia: modo config -> reset -> reinit
  // Observação: spi/gpio já estão prontos, então basta reprogramar MCP.
  mcp_reset();
  for (volatile int i=0; i<200000; i++) { }

  set_mode(CANCTRL_MODE_CONFIG);

  if (s_cfg.osc_hz == MCP_OSC_8MHZ && s_cfg.bitrate == MCP_BITRATE_250K) {
    set_bitrate_8mhz_250k();
  } else {
    return false;
  }

  if (s_cfg.rx_filter_pgn_fe05_only) {
    setup_filters_fe05_only();
  } else {
    setup_receive_any();
  }

  mcp_write_reg(MCP_CANINTE, CANINTE_RX0IE | CANINTE_RX1IE);
  mcp_write_reg(MCP_CANINTF, 0x00);
  mcp_write_reg(MCP_EFLG, 0x00);

  set_mode(CANCTRL_MODE_NORMAL);

  s_last_rx_ms = (uint32_t)esp_log_timestamp();
  return true;
}