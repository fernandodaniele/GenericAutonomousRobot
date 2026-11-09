#ifndef TEST_SDK_H
#define TEST_SDK_H
#include <windows.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_STATE 0x103
void test_log(const char *, const char *, ...);
const char *esp_err_to_name(esp_err_t);
#define ESP_LOGI(tag, ...) test_log(tag, __VA_ARGS__)
#define ESP_LOGE(tag, ...) test_log(tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) test_log(tag, __VA_ARGS__)
typedef struct { CRITICAL_SECTION native; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
void test_enter(portMUX_TYPE *);
void test_exit(portMUX_TYPE *);
#define taskENTER_CRITICAL(lock) test_enter(lock)
#define taskEXIT_CRITICAL(lock) test_exit(lock)
typedef int BaseType_t;
#define pdPASS 1
#define pdTRUE 1
typedef struct { CRITICAL_SECTION lock; } StaticSemaphore_t;
typedef StaticSemaphore_t *SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *);
BaseType_t xSemaphoreTake(SemaphoreHandle_t, unsigned);
BaseType_t xSemaphoreGive(SemaphoreHandle_t);
void esp_fill_random(void *, size_t);
#define pdMS_TO_TICKS(ms) ((ms) / 10U)
int64_t esp_timer_get_time(void);
BaseType_t xTaskCreate(void (*)(void *), const char *, unsigned, void *, unsigned, void *);
void vTaskDelay(unsigned);

#define UART_NUM_2 2
#define GPIO_NUM_17 17
#define GPIO_NUM_16 16
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_DEFAULT 0
#define UART_PIN_NO_CHANGE -1
typedef struct {
    int baud_rate, data_bits, parity, stop_bits, flow_ctrl, source_clk;
} uart_config_t;
int uart_driver_install(int, int, int, int, void *, int);
int uart_param_config(int, const uart_config_t *);
int uart_set_pin(int, int, int, int, int);
int uart_write_bytes(int, const void *, size_t);
int uart_read_bytes(int, void *, uint32_t, unsigned);
int uart_wait_tx_done(int, unsigned);

typedef void *httpd_handle_t;
typedef struct { void *user_ctx; size_t content_len; } httpd_req_t;
typedef esp_err_t (*http_handler_t)(httpd_req_t *);
typedef struct { const char *uri; int method; http_handler_t handler; void *user_ctx; } httpd_uri_t;
typedef struct { unsigned server_port, stack_size, task_priority; int recv_wait_timeout; unsigned max_uri_handlers; bool lru_purge_enable; } httpd_config_t;
#define HTTPD_DEFAULT_CONFIG() {80U, 4096U, 5U, 5, 8U, false}
#define HTTP_GET 0
#define HTTP_POST 1
esp_err_t httpd_start(httpd_handle_t *, const httpd_config_t *);
esp_err_t httpd_stop(httpd_handle_t);
esp_err_t httpd_register_uri_handler(httpd_handle_t, const httpd_uri_t *);
esp_err_t httpd_resp_set_type(httpd_req_t *, const char *);
esp_err_t httpd_resp_set_hdr(httpd_req_t *, const char *, const char *);
esp_err_t httpd_resp_send(httpd_req_t *, const char *, ptrdiff_t);
esp_err_t httpd_resp_set_status(httpd_req_t *, const char *);
size_t httpd_req_get_hdr_value_len(httpd_req_t *, const char *);
esp_err_t httpd_req_get_hdr_value_str(httpd_req_t *, const char *, char *, size_t);
int httpd_req_recv(httpd_req_t *, char *, size_t);
#endif
