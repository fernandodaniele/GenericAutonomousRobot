#include "web_server.h"
#include "stm32_navigation.h"

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "stm32_status.h"
#include "stm32_sensors.h"
#include "stm32_mapping.h"
#include "stm32_imu.h"
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

static esp_err_t WebServer_Map(httpd_req_t *request)
{
    static char response[MAP_JSON_CAPACITY]; /* HTTP unico; no ocupa su stack. */
    Stm32StatusSnapshot status;Stm32Status_GetSnapshot(&status);
    int n=Stm32Mapping_Json(response,sizeof(response),status.connected);
    if(n<0) return ESP_FAIL;
    esp_err_t err=httpd_resp_set_type(request,"application/json; charset=utf-8");
    if(err!=ESP_OK) return err;
    err=httpd_resp_set_hdr(request,"Cache-Control","no-store");
    if(err!=ESP_OK) return err;
    return httpd_resp_send(request,response,n);
}
static esp_err_t WebServer_MapScript(httpd_req_t *request)
{
    extern const uint8_t map_start[] asm("_binary_map_js_start");
    extern const uint8_t map_end[] asm("_binary_map_js_end");
    static const WebResource resource={map_start,map_end,"application/javascript; charset=utf-8"};
    request->user_ctx=(void *)&resource;return WebServer_Resource(request);
}

static bool ReadHeader(httpd_req_t *request, const char *name, char *value, size_t capacity)
{
    const size_t length = httpd_req_get_hdr_value_len(request, name);
    return length > 0U && length < capacity &&
           httpd_req_get_hdr_value_str(request, name, value, capacity) == ESP_OK;
}

static esp_err_t WebServer_Sensors(httpd_req_t *request)
{
    Stm32StatusSnapshot status; Stm32Status_GetSnapshot(&status);
    char response[768];
    int length = Stm32Sensors_Json(response, sizeof(response), status.connected, !WebCommands_CanPollSensors());
    if (length < 0) return ESP_FAIL;
    esp_err_t err = httpd_resp_set_type(request, "application/json; charset=utf-8");
    if (err != ESP_OK) return err;
    err = httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (err != ESP_OK) return err;
    return httpd_resp_send(request, response, length);
}

static esp_err_t WebServer_Navigation(httpd_req_t *request)
{
    Stm32StatusSnapshot status; Stm32Status_GetSnapshot(&status);
    /* Unica tarea HTTP, envio sincrono: conservar el buffer fuera de su stack,
     * igual que /api/imu y /api/map. No se comparte con el receptor UART. */
    static char response[1792];
    int n=Stm32Navigation_Json(response,sizeof response,status.connected,!WebCommands_CanPollSensors());
    const char *body=response;
    esp_err_t err;
    if (n<0 || (size_t)n>=sizeof response) {
        body="{\"error\":\"navigation_unavailable\"}";
        n=(int)strlen(body);
        err=httpd_resp_set_status(request,"500 Internal Server Error");
        if (err!=ESP_OK) { return err; }
    }
    err=httpd_resp_set_type(request,"application/json; charset=utf-8");
    if (err!=ESP_OK) { return err; }
    err=httpd_resp_set_hdr(request,"Cache-Control","no-store");
    if (err!=ESP_OK) { return err; }
    return httpd_resp_send(request,body,n);
}

static esp_err_t WebServer_SensorsScript(httpd_req_t *request)
{
    extern const uint8_t sensors_start[] asm("_binary_sensors_js_start");
    extern const uint8_t sensors_end[] asm("_binary_sensors_js_end");
    static const WebResource resource = { sensors_start, sensors_end, "application/javascript; charset=utf-8" };
    request->user_ctx = (void *)&resource;
    return WebServer_Resource(request);
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

static bool CommandHeaders(httpd_req_t *request, char id[33])
{
    char type[40], host[80], origin[88], expected[88];
    if (!ReadHeader(request, "Host", host, sizeof(host)) ||
        !ReadHeader(request, "Origin", origin, sizeof(origin)))
        { (void)CommandReply(request, "403 Forbidden", "{\"accepted\":false,\"error\":\"origin_required\"}", true); return false; }
    const int count = snprintf(expected, sizeof(expected), "http://%s", host);
    if (count < 0 || (size_t)count >= sizeof(expected) || strcmp(origin, expected) != 0)
        { (void)CommandReply(request, "403 Forbidden", "{\"accepted\":false,\"error\":\"foreign_origin\"}", true); return false; }
    if (!ReadHeader(request, "Content-Type", type, sizeof(type)) || strcmp(type, "text/plain") != 0)
        { (void)CommandReply(request, "415 Unsupported Media Type", "{\"accepted\":false,\"error\":\"use_text_plain\"}", true); return false; }
    if (!ReadHeader(request, "X-Request-ID", id, WEB_COMMAND_ID_LENGTH + 1U))
        { (void)CommandReply(request, "400 Bad Request", "{\"accepted\":false,\"error\":\"request_id_required\"}", true); return false; }
    if (strlen(id) != 32U) { (void)CommandReply(request, "400 Bad Request", "{\"accepted\":false,\"error\":\"request_id\"}", true); return false; }
    for (unsigned i = 0; i < 32U; ++i) if (!((id[i] >= '0' && id[i] <= '9') || (id[i] >= 'a' && id[i] <= 'f'))) {
        (void)CommandReply(request, "400 Bad Request", "{\"accepted\":false,\"error\":\"request_id\"}", true); return false;
    }
    return true;
}

static bool CommandBody(httpd_req_t *request, char *body, size_t capacity)
{
    if (request->content_len == 0U)
        { (void)CommandReply(request, "400 Bad Request", "{\"accepted\":false,\"error\":\"body_required\"}", true); return false; }
    if (request->content_len >= capacity)
        { (void)CommandReply(request, "413 Content Too Large", "{\"accepted\":false,\"error\":\"body_length\"}", true); return false; }

    size_t used = 0U;
    const int64_t started = esp_timer_get_time();
    while (used < request->content_len) {
        if (esp_timer_get_time() - started >= 1000000)
            { (void)CommandReply(request, "408 Request Timeout", "{\"accepted\":false,\"error\":\"body_timeout\"}", true); return false; }
        const int got = httpd_req_recv(request, body + used, request->content_len - used);
        if (got <= 0 || (size_t)got > request->content_len - used)
            { (void)CommandReply(request, "408 Request Timeout", "{\"accepted\":false,\"error\":\"body_incomplete\"}", true); return false; }
        used += (size_t)got;
    }
    body[used] = '\0';
    return true;
}

static esp_err_t WebServer_Command(httpd_req_t *request)
{
    char id[33], body[7];
    if (!CommandHeaders(request, id) || !CommandBody(request, body, sizeof(body))) return ESP_FAIL;
    const size_t used = request->content_len;
    UartControlCommand command;
    if (used == 4U && memcmp(body, "STOP", 4U) == 0) command = UART_CONTROL_STOP;
    else if (used == 5U && memcmp(body, "RESET", 5U) == 0) command = UART_CONTROL_RESET;
    else if (used == 6U && memcmp(body, "MANUAL", 6U) == 0) command = UART_CONTROL_MANUAL;
    else if (used == 4U && memcmp(body, "AUTO", 4U) == 0) command = UART_CONTROL_AUTO;
    else return CommandReply(request, "400 Bad Request", "{\"accepted\":false,\"error\":\"unsupported_command\"}", false);

    WebCommandReceipt receipt;
    const WebCommandResult result = WebCommands_Submit(id, command, &receipt);
    if (result == WEB_CMD_UNSAFE)
        return CommandReply(request, "412 Precondition Failed", "{\"accepted\":false,\"error\":\"auto_requires_safe_idle\"}", false);
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

static esp_err_t WebServer_Manual(httpd_req_t *request, unsigned operation)
{
    char id[33], body[16], token[33], seq_text[11];
    if (!CommandHeaders(request, id) || !CommandBody(request, body, sizeof(body))) return ESP_FAIL;
    WebManualResult result = WEB_MANUAL_INVALID;
    if (operation == 0U && request->content_len == 4U && memcmp(body, "OPEN", 4U) == 0) {
        result = WebManual_Open(id, token);
    } else if (operation == 1U) {
        UartMotionCommand direction;
        if (request->content_len == 7U && memcmp(body, "AVANZAR", 7U) == 0) direction = UART_MOTION_FORWARD;
        else if (request->content_len == 10U && memcmp(body, "RETROCEDER", 10U) == 0) direction = UART_MOTION_REVERSE;
        else if (request->content_len == 15U && memcmp(body, "GIRAR_IZQUIERDA", 15U) == 0) direction = UART_MOTION_LEFT;
        else if (request->content_len == 13U && memcmp(body, "GIRAR_DERECHA", 13U) == 0) direction = UART_MOTION_RIGHT;
        else return CommandReply(request, "400 Bad Request", "{\"accepted\":false,\"error\":\"direction\"}", false);
        uint32_t sequence = 0U;
        if (!ReadHeader(request, "X-Session-Token", token, sizeof(token)) ||
            !ReadHeader(request, "X-Sequence", seq_text, sizeof(seq_text)))
            return CommandReply(request, "400 Bad Request", "{\"accepted\":false,\"error\":\"token_sequence\"}", false);
        for (const char *p = seq_text; *p; ++p) {
            if (*p < '0' || *p > '9' || sequence > (UINT32_MAX - (uint32_t)(*p - '0')) / 10U)
                return CommandReply(request, "400 Bad Request", "{\"accepted\":false,\"error\":\"sequence\"}", false);
            sequence = sequence * 10U + (uint32_t)(*p - '0');
        }
        result = WebManual_Renew(token, sequence, direction);
    } else if (operation == 2U && request->content_len == 3U && memcmp(body, "END", 3U) == 0 &&
               ReadHeader(request, "X-Session-Token", token, sizeof(token))) {
        result = WebManual_End(token);
    } else if (operation == 3U && request->content_len == 7U && memcmp(body, "DETENER", 7U) == 0) {
        result = WebManual_Halt(id);
    }
    if (result != WEB_MANUAL_OK) {
        static const char *errors[] = { "", "invalid", "busy", "unsafe", "expired", "replay" };
        char response[80];
        int length = snprintf(response, sizeof(response), "{\"accepted\":false,\"error\":\"%s\"}", errors[result]);
        if (length < 0 || (size_t)length >= sizeof(response)) return ESP_FAIL;
        return CommandReply(request, result == WEB_MANUAL_INVALID ? "400 Bad Request" :
            result == WEB_MANUAL_UNSAFE ? "412 Precondition Failed" :
            result == WEB_MANUAL_EXPIRED ? "410 Gone" : "409 Conflict", response, false);
    }
    if (operation == 0U) {
        char response[112];
        int length = snprintf(response, sizeof(response), "{\"accepted\":true,\"token\":\"%s\",\"lease_ms\":%u}", token, WEB_MANUAL_LEASE_MS);
        if (length < 0 || (size_t)length >= sizeof(response)) return ESP_FAIL;
        return CommandReply(request, "201 Created", response, false);
    }
    if (operation == 1U) {
        char response[64];
        int length = snprintf(response, sizeof(response), "{\"accepted\":true,\"lease_ms\":%u}", WEB_MANUAL_LEASE_MS);
        if (length < 0 || (size_t)length >= sizeof(response)) return ESP_FAIL;
        return CommandReply(request, "202 Accepted", response, false);
    }
    return CommandReply(request, "202 Accepted", "{\"accepted\":true,\"phase\":\"halt_requested\"}", false);
}
static esp_err_t WebServer_ManualOpen(httpd_req_t *r) { return WebServer_Manual(r, 0U); }
static esp_err_t WebServer_ManualRenew(httpd_req_t *r) { return WebServer_Manual(r, 1U); }
static esp_err_t WebServer_ManualEnd(httpd_req_t *r) { return WebServer_Manual(r, 2U); }
static esp_err_t WebServer_ManualHalt(httpd_req_t *r) { return WebServer_Manual(r, 3U); }
static esp_err_t WebServer_ManualScript(httpd_req_t *r);

static esp_err_t WebServer_Imu(httpd_req_t *request)
{
    static char response[1792]; /* Single HTTP task, including IMU-3 orientation. */
    Stm32StatusSnapshot status; Stm32Status_GetSnapshot(&status);
    int n = Stm32Imu_Json(response, sizeof(response), status.connected, !WebCommands_CanPollSensors());
    if (n < 0) return ESP_FAIL;
    esp_err_t err = httpd_resp_set_type(request, "application/json; charset=utf-8");
    if (err != ESP_OK) return err;
    err = httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (err != ESP_OK) return err;
    return httpd_resp_send(request, response, n);
}

static esp_err_t WebServer_ImuCalibrate(httpd_req_t *request)
{
    char id[33],body[10];
    if(!CommandHeaders(request,id) || !CommandBody(request,body,sizeof(body)))return ESP_FAIL;
    if(request->content_len!=9U || memcmp(body,"CALIBRATE",9U)!=0)
        return CommandReply(request,"400 Bad Request","{\"accepted\":false,\"error\":\"operation\"}",false);
    int result=Stm32Imu_RequestCalibration(id);
    if(result==0)return CommandReply(request,"202 Accepted","{\"accepted\":true,\"phase\":\"queued\"}",false);
    return CommandReply(request,result==3 ? "412 Precondition Failed" : "409 Conflict",
        result==1 ? "{\"accepted\":false,\"error\":\"replay\"}" :
        result==2 ? "{\"accepted\":false,\"error\":\"busy\"}" : "{\"accepted\":false,\"error\":\"unsafe\"}",false);
}

static esp_err_t WebServer_ImuScript(httpd_req_t *request)
{
    extern const uint8_t start[] asm("_binary_imu_js_start");
    extern const uint8_t end[] asm("_binary_imu_js_end");
    const WebResource script = { start, end, "application/javascript; charset=utf-8" };
    request->user_ctx = (void *)&script;
    return WebServer_Resource(request);
}

esp_err_t WebServer_Start(void)
{
    if (s_attempted) return s_result;
    s_attempted = true;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.recv_wait_timeout = 1; /* Lecturas POST acotadas; sin esperar UART. */
    config.max_uri_handlers = 19;
    config.lru_purge_enable = true;
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
        { .uri = "/api/command", .method = HTTP_POST, .handler = WebServer_Command },
        { .uri = "/api/manual/start", .method = HTTP_POST, .handler = WebServer_ManualOpen },
        { .uri = "/api/manual/renew", .method = HTTP_POST, .handler = WebServer_ManualRenew },
        { .uri = "/api/manual/end", .method = HTTP_POST, .handler = WebServer_ManualEnd },
        { .uri = "/api/manual/halt", .method = HTTP_POST, .handler = WebServer_ManualHalt },
        { .uri = "/manual.js", .method = HTTP_GET, .handler = WebServer_ManualScript },
        { .uri = "/api/sensors", .method = HTTP_GET, .handler = WebServer_Sensors },
        { .uri = "/sensors.js", .method = HTTP_GET, .handler = WebServer_SensorsScript },
        { .uri = "/api/map", .method = HTTP_GET, .handler = WebServer_Map },
        { .uri = "/map.js", .method = HTTP_GET, .handler = WebServer_MapScript },
        { .uri = "/api/imu", .method = HTTP_GET, .handler = WebServer_Imu },
        { .uri = "/imu.js", .method = HTTP_GET, .handler = WebServer_ImuScript },
        { .uri = "/api/imu/calibrate", .method = HTTP_POST, .handler = WebServer_ImuCalibrate },
        { .uri = "/api/navigation", .method = HTTP_GET, .handler = WebServer_Navigation }
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

extern const uint8_t manual_start[] asm("_binary_manual_js_start");
extern const uint8_t manual_end[] asm("_binary_manual_js_end");
static esp_err_t WebServer_ManualScript(httpd_req_t *r)
{
    static const WebResource resource = { manual_start, manual_end, "application/javascript; charset=utf-8" };
    r->user_ctx = (void *)&resource;
    return WebServer_Resource(r);
}
