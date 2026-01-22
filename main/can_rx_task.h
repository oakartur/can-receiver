#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "mcp2515.h"

void can_rx_task_start(const mcp2515_cfg_t *mcfg, QueueHandle_t out_q);
