// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * HTTP on port 80 (CONFIG_LR20XX_HTTP):
 *   GET  /api/status  {"uptime_s", "reset", "firmware", "internal_free", "wifi_rssi", "wifi_channel", the bridge's
 *                     counts}
 *   GET  /api/speedtest?kb=N  N KB of zeros (1024 by default), for the link's speed with no samples behind it
 *   POST /api/speedtest  the body read and dropped: the speed the other way
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
#include "esp_wifi.h"
#include <stdlib.h>
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
    static char body[1024];
    int n = snprintf(body, sizeof body, "{\"uptime_s\":%lld,\"reset\":\"%s\",\"firmware\":\"%s\",\"internal_free\":%u,",
                     (long long)(esp_timer_get_time() / 1000000), reset_name(esp_reset_reason()),
                     esp_app_get_description()->version, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) /* the link the stream goes over */
        n += snprintf(body + n, sizeof body - (size_t)n, "\"wifi_rssi\":%d,\"wifi_channel\":%u,", ap.rssi,
                      (unsigned)ap.primary);
    n += bridge_status_json(body + n, sizeof body - (size_t)n - 2);
    snprintf(body + n, sizeof body - (size_t)n, "}");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

/* N KB of zeros (?kb=, 1 MB by default): what the link carries out with no samples behind it */
static esp_err_t speedtest_get(httpd_req_t *req)
{
    char q[16], v[8];
    long kb = 1024;
    if (httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK && httpd_query_key_value(q, "kb", v, sizeof v) == ESP_OK)
        kb = strtol(v, NULL, 10);
    if (kb < 1 || kb > 65536)
        kb = 1024;
    static uint8_t zeros[4096];
    httpd_resp_set_type(req, "application/octet-stream");
    for (long i = 0; i < kb / 4; i++)
        if (httpd_resp_send_chunk(req, (const char *)zeros, sizeof zeros) != ESP_OK)
            return ESP_FAIL;
    return httpd_resp_send_chunk(req, NULL, 0);
}

/* A body read and dropped: the link's speed into the bridge */
static esp_err_t speedtest_post(httpd_req_t *req)
{
    static char buf[4096];
    size_t left = req->content_len;
    while (left) {
        int k = httpd_req_recv(req, buf, left < sizeof buf ? left : sizeof buf);
        if (k == HTTPD_SOCK_ERR_TIMEOUT)
            continue;
        if (k <= 0)
            return ESP_FAIL;
        left -= (size_t)k;
    }
    return httpd_resp_sendstr(req, "{\"ok\":true}");
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
    const httpd_uri_t sp = {.uri = "/api/speedtest", .method = HTTP_GET, .handler = speedtest_get};
    httpd_register_uri_handler(s, &sp);
    const httpd_uri_t spp = {.uri = "/api/speedtest", .method = HTTP_POST, .handler = speedtest_post};
    httpd_register_uri_handler(s, &spp);
}

#else

void http_start(void) {}

#endif
