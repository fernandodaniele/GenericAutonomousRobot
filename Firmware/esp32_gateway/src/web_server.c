#include "web_server.h"

#include <stdbool.h>
#include <stdint.h>
#include "esp_http_server.h"
#include "esp_log.h"

/* EMBED_FILES usa el nombre base del archivo y no agrega terminador NUL. */
extern const uint8_t index_start[] asm("_binary_index_html_start");
extern const uint8_t index_end[] asm("_binary_index_html_end");
extern const uint8_t style_start[] asm("_binary_style_css_start");
extern const uint8_t style_end[] asm("_binary_style_css_end");
extern const uint8_t script_start[] asm("_binary_app_js_start");
extern const uint8_t script_end[] asm("_binary_app_js_end");

typedef struct {
    const uint8_t *start;
    const uint8_t *end;
    const char *type;
} WebResource;

static const WebResource resources[] = {
    { index_start, index_end, "text/html; charset=utf-8" },
    { style_start, style_end, "text/css; charset=utf-8" },
    { script_start, script_end, "application/javascript; charset=utf-8" }
};
static const char *TAG = "WEB_SERVER";
static httpd_handle_t s_server;
static bool s_attempted;
static esp_err_t s_result = ESP_ERR_INVALID_STATE;

static esp_err_t WebServer_Resource(httpd_req_t *request)
{
    const WebResource *resource = request->user_ctx;
    esp_err_t err = httpd_resp_set_type(request, resource->type);
    if (err != ESP_OK) return err;
    err = httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (err != ESP_OK) return err;
    return httpd_resp_send(request, (const char *)resource->start, resource->end - resource->start);
}

static esp_err_t WebServer_Health(httpd_req_t *request)
{
    static const char response[] = "{\"status\":\"ok\",\"service\":\"esp32-web\"}";
    esp_err_t err = httpd_resp_set_type(request, "application/json; charset=utf-8");
    if (err != ESP_OK) return err;
    err = httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (err != ESP_OK) return err;
    return httpd_resp_send(request, response, sizeof(response) - 1U);
}

esp_err_t WebServer_Start(void)
{
    if (s_attempted) return s_result;
    s_attempted = true;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s; UART continuara", esp_err_to_name(err));
        return s_result = err;
    }
    const httpd_uri_t routes[] = {
        { .uri = "/", .method = HTTP_GET, .handler = WebServer_Resource, .user_ctx = (void *)&resources[0] },
        { .uri = "/style.css", .method = HTTP_GET, .handler = WebServer_Resource, .user_ctx = (void *)&resources[1] },
        { .uri = "/app.js", .method = HTTP_GET, .handler = WebServer_Resource, .user_ctx = (void *)&resources[2] },
        { .uri = "/health", .method = HTTP_GET, .handler = WebServer_Health }
    };
    for (size_t i = 0U; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        err = httpd_register_uri_handler(s_server, &routes[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Registro de %s: %s", routes[i].uri, esp_err_to_name(err));
            esp_err_t cleanup = httpd_stop(s_server);
            if (cleanup == ESP_OK) s_server = NULL;
            else ESP_LOGW(TAG, "httpd_stop: %s", esp_err_to_name(cleanup));
            return s_result = err;
        }
    }
    ESP_LOGI(TAG, "HTTP listo en puerto %u", (unsigned)config.server_port);
    return s_result = ESP_OK;
}
