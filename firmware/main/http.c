// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * HTTP on port 80 (CONFIG_LR20XX_HTTP):
 *   GET  /api/status  {"uptime_s", "reset", "firmware", "internal_free", the bridge's counts}
 *   POST /api/ota     the body is an app image (build/.../lr20xx_bridge.bin): written to the other OTA slot, then
 *                     a restart about a second after the answer. Receiving pauses while it's written.
 */
#include <stdio.h>
#include <string.h>

#include "bridge.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "links.h"
#include "sdkconfig.h"

#if CONFIG_LR20XX_HTTP

static const char *TAG = "http";

static const char *reset_name(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_USB: return "usb";
    case ESP_RST_JTAG: return "jtag";
    default: return "other";
    }
}

static esp_err_t status_get(httpd_req_t *req)
{
    static char body[768];
    int n = snprintf(body, sizeof body, "{\"uptime_s\":%lld,\"reset\":\"%s\",\"firmware\":\"%s\",\"internal_free\":%u,",
                     (long long)(esp_timer_get_time() / 1000000), reset_name(esp_reset_reason()),
                     esp_app_get_description()->version, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    n += bridge_status_json(body + n, sizeof body - (size_t)n - 2);
    snprintf(body + n, sizeof body - (size_t)n, "}");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

static void restart_later(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

static esp_err_t ota_post(httpd_req_t *req)
{
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t h;
    if (!part || esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &h) != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA slot");
    bool was = bridge_pause(); /* flash writes stall the caches: the reader would lose samples anyway */
    static char buf[4096];
    int left = req->content_len, got = 0;
    esp_err_t e = ESP_OK;
    while (left > 0 && e == ESP_OK) {
        int k = httpd_req_recv(req, buf, left < (int)sizeof buf ? left : (int)sizeof buf);
        if (k == HTTPD_SOCK_ERR_TIMEOUT)
            continue;
        if (k <= 0) {
            e = ESP_FAIL;
            break;
        }
        e = esp_ota_write(h, buf, (size_t)k);
        left -= k;
        got += k;
    }
    if (e == ESP_OK)
        e = esp_ota_end(h);
    else
        esp_ota_abort(h);
    if (e == ESP_OK)
        e = esp_ota_set_boot_partition(part);
    if (e != ESP_OK) {
        bridge_resume(was);
        ESP_LOGE(TAG, "OTA failed after %d bytes: %s", got, esp_err_to_name(e));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(e));
    }
    ESP_LOGI(TAG, "OTA: %d bytes into %s; restarting", got, part->label);
    httpd_resp_sendstr(req, "{\"ok\":true}");
    xTaskCreate(restart_later, "restart", 2048, NULL, 5, NULL);
    return ESP_OK;
}

void http_start(void)
{
    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.core_id = 1;
    c.stack_size = 6144;
    c.recv_wait_timeout = 10;
    httpd_handle_t s;
    if (httpd_start(&s, &c) != ESP_OK) {
        ESP_LOGE(TAG, "not started");
        return;
    }
    const httpd_uri_t st = {.uri = "/api/status", .method = HTTP_GET, .handler = status_get};
    const httpd_uri_t ota = {.uri = "/api/ota", .method = HTTP_POST, .handler = ota_post};
    httpd_register_uri_handler(s, &st);
    httpd_register_uri_handler(s, &ota);
}

#else

void http_start(void) {}

#endif
