idf_component_register(
    SRCS "main.c" "uart_comm.c" "wifi_ap.c" "web_server.c" "stm32_status.c" "web_commands.c"
    INCLUDE_DIRS "../include"
    PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_http_server
                  esp_driver_uart esp_driver_gpio esp_timer
    EMBED_FILES "../web/index.html" "../web/style.css" "../web/app.js"
)
