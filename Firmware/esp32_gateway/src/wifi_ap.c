#include "wifi_ap.h"

#include <stdbool.h>
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_defaults.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "nvs_flash.h"

/* Configuracion centralizada de laboratorio. No se ofrece modo abierto. */
#define WIFI_AP_SSID        "GenericRobot"
#define WIFI_AP_PASSWORD    "GAR-Lab-2026"
#define WIFI_AP_CHANNEL     1U
#define WIFI_AP_MAX_CLIENTS 2U

static const char *TAG = "WIFI_AP";
static bool s_attempted;
static esp_err_t s_result = ESP_ERR_INVALID_STATE;

static void WifiAp_ClientEvent(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base != WIFI_EVENT || data == NULL) return;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *event = data;
        ESP_LOGI(TAG, "Cliente conectado (AID=%u)", (unsigned)event->aid);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *event = data;
        ESP_LOGI(TAG, "Cliente desconectado (AID=%u)", (unsigned)event->aid);
    }
}

esp_err_t WifiAp_Start(void)
{
    if (s_attempted) return s_result;
    s_attempted = true;

    const size_t ssid_length = strlen(WIFI_AP_SSID);
    const size_t password_length = strlen(WIFI_AP_PASSWORD);
    if (ssid_length == 0U || ssid_length > 32U ||
        password_length < 8U || password_length > 63U) {
        ESP_LOGE(TAG, "SSID o longitud de clave WPA2 invalidos; AP no iniciado");
        return s_result = ESP_ERR_INVALID_ARG;
    }

    esp_netif_t *netif = NULL;
    esp_event_handler_instance_t client_handler = NULL;
    bool wifi_initialized = false;
    bool wifi_started = false;
    const char *step = "nvs_flash_init";
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) goto failed; /* No borrar NVS automaticamente. */

    step = "esp_netif_init";
    err = esp_netif_init();
    if (err != ESP_OK) goto failed;
    step = "esp_event_loop_create_default";
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) goto failed;

    /* Mismos defaults DHCP/IP del helper de ESP-IDF, sin su assert/abort. */
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_AP();
    step = "esp_netif_new";
    netif = esp_netif_new(&netif_config);
    if (netif == NULL) { err = ESP_ERR_NO_MEM; goto failed; }
    step = "esp_netif_attach_wifi_ap";
    err = esp_netif_attach_wifi_ap(netif);
    if (err != ESP_OK) goto failed;
    step = "esp_wifi_set_default_wifi_ap_handlers";
    err = esp_wifi_set_default_wifi_ap_handlers();
    if (err != ESP_OK) goto failed;

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    step = "esp_wifi_init";
    err = esp_wifi_init(&init_config);
    if (err != ESP_OK) goto failed;
    wifi_initialized = true;
    step = "esp_event_handler_instance_register";
    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             WifiAp_ClientEvent, NULL, &client_handler);
    if (err != ESP_OK) goto failed;

    wifi_config_t config = {0};
    memcpy(config.ap.ssid, WIFI_AP_SSID, ssid_length);
    memcpy(config.ap.password, WIFI_AP_PASSWORD, password_length);
    config.ap.ssid_len = (uint8_t)ssid_length;
    config.ap.channel = WIFI_AP_CHANNEL;
    config.ap.max_connection = WIFI_AP_MAX_CLIENTS;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    step = "esp_wifi_set_mode";
    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) goto failed;
    step = "esp_wifi_set_config";
    err = esp_wifi_set_config(WIFI_IF_AP, &config);
    if (err != ESP_OK) goto failed;
    step = "esp_wifi_start";
    err = esp_wifi_start();
    if (err != ESP_OK) goto failed;
    wifi_started = true;

    esp_netif_ip_info_t ip;
    step = "esp_netif_get_ip_info";
    err = esp_netif_get_ip_info(netif, &ip);
    if (err != ESP_OK) goto failed;
    ESP_LOGI(TAG, "AP listo. SSID=%s IP=" IPSTR, WIFI_AP_SSID, IP2STR(&ip.ip));
    return s_result = ESP_OK;

failed:
    ESP_LOGE(TAG, "%s: %s; UART continuara", step, esp_err_to_name(err));
    if (wifi_started) {
        esp_err_t cleanup = esp_wifi_stop();
        if (cleanup != ESP_OK) ESP_LOGW(TAG, "esp_wifi_stop: %s", esp_err_to_name(cleanup));
    }
    if (client_handler != NULL) {
        esp_err_t cleanup = esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, client_handler);
        if (cleanup != ESP_OK) ESP_LOGW(TAG, "Retiro de handler: %s", esp_err_to_name(cleanup));
    }
    if (netif != NULL) esp_netif_destroy_default_wifi(netif);
    if (wifi_initialized) {
        esp_err_t cleanup = esp_wifi_deinit();
        if (cleanup != ESP_OK) ESP_LOGW(TAG, "esp_wifi_deinit: %s", esp_err_to_name(cleanup));
    }
    return s_result = err;
}
