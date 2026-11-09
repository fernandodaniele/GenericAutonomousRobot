#include "web_server.h"

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "stm32_status.h"
#include "web_commands.h"
#include "esp_timer.h"
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

static esp_err_t WebServer_Status(httpd_req_t *request)
{
    Stm32StatusSnapshot status;
    Stm32Status_GetSnapshot(&status);
    char response[384];
    int length;
    if (!status.valid) {
        length = snprintf(response, sizeof(response),
                          "{\"valid\":false,\"connected\":false,\"state\":null,"
                          "\"faults\":null,\"stop\":null,\"age_ms\":null}");
    } else {
        length = snprintf(response, sizeof(response),
                          "{\"valid\":true,\"connected\":%s,\"state\":%" PRIu32
                          ",\"faults\":%" PRIu32 ",\"stop\":%" PRIu32 ",\"age_ms\":%" PRIu64 "}",
                          status.connected ? "true" : "false", status.state,
                          status.faults, status.stop, status.age_ms);
    }
    if (length < 0 || (size_t)length >= sizeof(response)) return ESP_FAIL;
    WebCommandReceipt receipt;
    if (WebCommands_GetLatest(&receipt)) {
        const int extra = snprintf(response + length - 1, sizeof(response) - (size_t)length + 1U,
            ",\"command\":{\"id\":\"%s\",\"name\":\"%s\",\"phase\":\"%s\",\"status_after_send\":%s}}",
            receipt.id, WebCommands_Name(receipt.command), WebCommands_Phase(receipt.phase),
            status.valid && status.connected && receipt.phase == WEB_CMD_TRANSMITTED &&
            status.received_us > receipt.sent_us ? "true" : "false");
        if (extra < 0 || (size_t)extra >= sizeof(response) - (size_t)length + 1U) return ESP_FAIL;
        length = length - 1 + extra;
    }
    esp_err_t err = httpd_resp_set_type(request, "application/json; charset=utf-8");
    if (err != ESP_OK) return err;
    err = httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (err != ESP_OK) return err;
    return httpd_resp_send(request, response, length);
}

static bool ReadHeader(httpd_req_t *request, const char *name, char *value, size_t capacity)
{
    const size_t length = httpd_req_get_hdr_value_len(request, name);
    return length > 0U && length < capacity &&
           httpd_req_get_hdr_value_str(request, name, value, capacity) == ESP_OK;
}

static esp_err_t CommandReply(httpd_req_t *request, const char *status, const char *body, bool close)
{
    esp_err_t err = httpd_resp_set_status(request, status);
    if (err != ESP_OK) return err;
    err = httpd_resp_set_type(request, "application/json; charset=utf-8");
    if (err != ESP_OK) return err;
    err = httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (err != ESP_OK) return err;
    if (close && httpd_resp_set_hdr(request, "Connection", "close") != ESP_OK) return ESP_FAIL;
    err = httpd_resp_send(request, body, strlen(body));
    /* Cerrar ante cuerpo sin consumir: no drenar un contenido arbitrario. */
    return close ? ESP_FAIL : err;
}

static esp_err_t WebServer_Command(httpd_req_t *request)
{
    char id[WEB_COMMAND_ID_LENGTH + 1U], type[40], host[80], origin[88], expected[88];
    if (!ReadHeader(request, "Host", host, sizeof(host)) ||
        !ReadHeader(request, "Origin", origin, sizeof(origin)))
        return CommandReply(request, "403 Forbidden", "{\"accepted\":false,\"error\":\"origin_required\"}", true);
    const int count = snprintf(expected, sizeof(expected), "http://%s", host);
    if (count < 0 || (size_t)count >= sizeof(expected) || strcmp(origin, expected) != 0)
        return CommandReply(request, "403 Forbidden", "{\"accepted\":false,\"error\":\"foreign_origin\"}", true);
    if (!ReadHeader(request, "Content-Type", type, sizeof(type)) || strcmp(type, "text/plain") != 0)
        return CommandReply(request, "415 Unsupported Media Type", "{\"accepted\":false,\"error\":\"use_text_plain\"}", true);
    if (!ReadHeader(request, "X-Request-ID", id, sizeof(id)))
        return CommandReply(request, "400 Bad Request", "{\"accepted\":false,\"error\":\"request_id_required\"}", true);
    if (request->content_len == 0U)
        return CommandReply(request, "400 Bad Request", "{\"accepted\":false,\"error\":\"body_required\"}", true);
    if (request->content_len > 6U)
        return CommandReply(request, "413 Content Too Large", "{\"accepted\":false,\"error\":\"body_length\"}", true);

    char body[7];
    size_t used = 0U;
    const int64_t started = esp_timer_get_time();
    while (used < request->content_len) {
        if (esp_timer_get_time() - started >= 1000000)
            return CommandReply(request, "408 Request Timeout", "{\"accepted\":false,\"error\":\"body_timeout\"}", true);
        const int got = httpd_req_recv(request, body + used, request->content_len - used);
        if (got <= 0 || (size_t)got > request->content_len - used)
            return CommandReply(request, "408 Request Timeout", "{\"accepted\":false,\"error\":\"body_incomplete\"}", true);
        used += (size_t)got;
    }
    body[used] = '\0';
    UartControlCommand command;
    if (used == 4U && memcmp(body, "STOP", 4U) == 0) command = UART_CONTROL_STOP;
    else if (used == 5U && memcmp(body, "RESET", 5U) == 0) command = UART_CONTROL_RESET;
    else if (used == 6U && memcmp(body, "MANUAL", 6U) == 0) command = UART_CONTROL_MANUAL;
    else return CommandReply(request, "400 Bad Request", "{\"accepted\":false,\"error\":\"unsupported_command\"}", false);

    WebCommandReceipt receipt;
    const WebCommandResult result = WebCommands_Submit(id, command, &receipt);
    if (result == WEB_CMD_BUSY)
        return CommandReply(request, "409 Conflict", "{\"accepted\":false,\"error\":\"command_busy\"}", false);
    if (result == WEB_CMD_CONFLICT || result == WEB_CMD_INVALID)
        return CommandReply(request, "400 Bad Request", "{\"accepted\":false,\"error\":\"invalid_request_id\"}", false);
    char response[192];
    const int length = snprintf(response, sizeof(response),
        "{\"accepted\":true,\"id\":\"%s\",\"command\":\"%s\",\"phase\":\"%s\",\"duplicate\":%s,\"confirmed\":false}",
        receipt.id, WebCommands_Name(receipt.command), WebCommands_Phase(receipt.phase),
        result == WEB_CMD_DUPLICATE ? "true" : "false");
    if (length < 0 || (size_t)length >= sizeof(response)) return ESP_FAIL;
    return CommandReply(request, "202 Accepted", response, false);
}

esp_err_t WebServer_Start(void)
{
    if (s_attempted) return s_result;
    s_attempted = true;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.recv_wait_timeout = 1; /* Lecturas POST acotadas; sin esperar UART. */
    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s; UART continuara", esp_err_to_name(err));
        return s_result = err;
    }
    const httpd_uri_t routes[] = {
        { .uri = "/", .method = HTTP_GET, .handler = WebServer_Resource, .user_ctx = (void *)&resources[0] },
        { .uri = "/style.css", .method = HTTP_GET, .handler = WebServer_Resource, .user_ctx = (void *)&resources[1] },
        { .uri = "/app.js", .method = HTTP_GET, .handler = WebServer_Resource, .user_ctx = (void *)&resources[2] },
        { .uri = "/health", .method = HTTP_GET, .handler = WebServer_Health },
        { .uri = "/api/status", .method = HTTP_GET, .handler = WebServer_Status },
        { .uri = "/api/command", .method = HTTP_POST, .handler = WebServer_Command }
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
