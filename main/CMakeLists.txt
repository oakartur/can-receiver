idf_component_register(
                    SRCS 
                        "main.c"
                        "wifi_mngr.c"
                        "ringbuf.c"
                        "can_rx_task.c"
                        "j1939_decode.c"
                        "modbus_tcp.c"
                        "can_watchdog.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_wifi lwip mcp2515)
