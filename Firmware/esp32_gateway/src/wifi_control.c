#include "wifi_control.h"

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "telemetry.h"
#include "uart_comm.h"

#define STATUS_JSON_SIZE 1024

static const char *TAG = "WIFI_CTRL";

/* Pagina del panel: web/index.html, embebida en el firmware */
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    // EMBED_TXTFILES agrega un '\0' al final que no forma parte de la pagina
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start - 1);
}

static esp_err_t status_handler(httpd_req_t *req) {
    static char json[STATUS_JSON_SIZE];   // El servidor atiende de a un pedido

    size_t length = Telemetry_ToJson(json, sizeof(json));
    if (length == 0) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Sin datos");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, length);
}

static esp_err_t start_handler(httpd_req_t *req) {
    Uart_SendLine("START");
    ESP_LOGI(TAG, "START enviado a la STM32");
    return httpd_resp_sendstr(req, "START enviado");
}

static esp_err_t stop_handler(httpd_req_t *req) {
    Uart_SendLine("STOP");
    ESP_LOGI(TAG, "STOP enviado a la STM32");
    return httpd_resp_sendstr(req, "STOP enviado");
}

static void start_web_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;   // Un celular que se fue no deja ocupadas las conexiones
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo iniciar el servidor web");
        return;
    }

    const httpd_uri_t index_uri = { .uri = "/",      .method = HTTP_GET,  .handler = index_handler };
    const httpd_uri_t start_uri = { .uri = "/start", .method = HTTP_POST, .handler = start_handler };
    const httpd_uri_t stop_uri  = { .uri = "/stop",  .method = HTTP_POST, .handler = stop_handler };
    const httpd_uri_t status_uri = { .uri = "/api/status", .method = HTTP_GET, .handler = status_handler };
    httpd_register_uri_handler(server, &status_uri);
    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &start_uri);
    httpd_register_uri_handler(server, &stop_uri);
}

void WifiControl_Start(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = WIFI_CONTROL_SSID,
            .ssid_len = sizeof(WIFI_CONTROL_SSID) - 1,
            .channel = WIFI_CONTROL_CHANNEL,
            .password = WIFI_CONTROL_PASSWORD,
            .max_connection = WIFI_CONTROL_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Red WiFi \"%s\" lista. Abrir http://192.168.4.1", WIFI_CONTROL_SSID);
    start_web_server();
}
