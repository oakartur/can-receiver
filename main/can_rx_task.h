#ifndef MAIN_CAN_RX_TASK_H
#define MAIN_CAN_RX_TASK_H
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "mcp2515.h"

void can_rx_task_start(const mcp2515_cfg_t *mcfg, QueueHandle_t out_q);

#endif  // MAIN_CAN_RX_TASK_H
