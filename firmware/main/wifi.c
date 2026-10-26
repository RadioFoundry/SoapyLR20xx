// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * Wi-Fi as a station, with the network's name and password from NVS, and the serial console that sets them:
 *   wifi <ssid> <password>   save them and connect (no password: an open network)
 *   status                   the address, the chip, what it's doing
 *   restart
 * The console is the USB serial port (USB Serial/JTAG) or UART0, as the sdkconfig's console is set; with the
 * network over USB (CONFIG_LR20XX_USB_NCM), the CDC port beside it.
 */
#include <stdio.h>
#include <string.h>

#include "bridge.h"
#include "esp_console.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "links.h"
#include "nvs.h"
#include "sdkconfig.h"
#if CONFIG_LR20XX_USB_NCM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"
#endif

static const char *TAG = "wifi";

static esp_netif_t *sta;
static volatile bool connected;
static char ip[16];

bool wifi_connected(void) { return connected; }

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (connected)
            ESP_LOGW(TAG, "disconnected");
        connected = false;
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        snprintf(ip, sizeof ip, IPSTR, IP2STR(&e->ip_info.ip));
        connected = true;
        ESP_LOGI(TAG, "%s, TCP %d", ip, LR20XX_LINK_TCP_PORT);
    }
}

static bool load(wifi_config_t *c)
{
    nvs_handle_t h;
    if (nvs_open("wifi", NVS_READONLY, &h) != ESP_OK)
        return false;
    size_t n = sizeof c->sta.ssid;
    bool ok = nvs_get_str(h, "ssid", (char *)c->sta.ssid, &n) == ESP_OK && c->sta.ssid[0];
    n = sizeof c->sta.password;
    if (nvs_get_str(h, "pass", (char *)c->sta.password, &n) != ESP_OK)
        c->sta.password[0] = 0;
    nvs_close(h);
    return ok;
}

void wifi_start(void)
{
    sta = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(sta, CONFIG_LR20XX_HOSTNAME);
    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    wifi_config_t c = {0};
    if (!load(&c)) {
        ESP_LOGW(TAG, "no network set: give one on the console with: wifi <ssid> <password>");
        return;
    }
    esp_wifi_set_config(WIFI_IF_STA, &c);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE); /* power save holds packets for the beacon: the stream can't wait */
    ESP_LOGI(TAG, "joining \"%s\"", (char *)c.sta.ssid);
}

static int cmd_wifi(int argc, char **argv)
{
    if (argc < 2 || argc > 3) {
        printf("wifi <ssid> [password]\n");
        return 1;
    }
    nvs_handle_t h;
    if (nvs_open("wifi", NVS_READWRITE, &h) != ESP_OK) {
        printf("NVS not available\n");
        return 1;
    }
    nvs_set_str(h, "ssid", argv[1]);
    nvs_set_str(h, "pass", argc == 3 ? argv[2] : "");
    nvs_commit(h);
    nvs_close(h);
    wifi_config_t c = {0};
    load(&c);
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_set_config(WIFI_IF_STA, &c);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    printf("saved; joining \"%s\"\n", argv[1]);
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    static char js[512];
    bridge_status_json(js, sizeof js);
    printf("wifi: %s %s\n{%s}\n", connected ? "up" : "down", connected ? ip : "", js);
#if CONFIG_LR20XX_USB_NCM
    link_ncm_status(js, sizeof js);
    printf("%s\n", js);
#endif
    return 0;
}

static int cmd_restart(int argc, char **argv)
{
    esp_restart();
    return 0;
}

#if CONFIG_LR20XX_USB_NCM
/* When the host last opened the port (DTR up). For a moment after, a host tty still in its cooked mode echoes back
 * what the bridge logged: that's dropped, not taken for a command. */
static volatile int64_t opened_at;

static void on_line(int itf, cdcacm_event_t *event)
{
    if (event->line_state_changed_data.dtr)
        opened_at = esp_timer_get_time();
}

/* esp_console's REPLs are for the UART and the USB Serial/JTAG; on TinyUSB's CDC it's this. Echoes what's typed;
 * stdin doesn't block, so it waits a little when there's nothing. */
static void cdc_repl(void *arg)
{
    static char line[160];
    size_t n = 0;
    int64_t prompted = 0;
    printf("lr20xx> ");
    fflush(stdout);
    for (;;) {
        /* A fresh prompt for a terminal that just opened, in two transfers: a USB CDC port under a VM's passthrough
         * lost the first two after it opened, and better these than a command's answer. */
        const int64_t opened = opened_at;
        if (opened != prompted && esp_timer_get_time() - opened >= 200000) {
            prompted = opened;
            n = 0;
            printf("\n");
            fflush(stdout);
            printf("lr20xx> ");
            fflush(stdout);
        }
        int c = fgetc(stdin);
        if (c == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (esp_timer_get_time() - opened_at < 200000) {
            n = 0;
            continue;
        }
        if (c == '\r' || c == '\n') {
            printf("\n");
            line[n] = 0;
            int ret;
            if (n && esp_console_run(line, &ret) == ESP_ERR_NOT_FOUND)
                printf("%s: no such command (help lists them)\n", line);
            n = 0;
            printf("lr20xx> ");
        } else if ((c == 8 || c == 127) && n) {
            n--;
            printf("\b \b");
        } else if (c >= ' ' && n < sizeof line - 1) {
            line[n++] = (char)c;
            putchar(c);
        }
        fflush(stdout);
    }
}

static esp_err_t cdc_console(void)
{
    const tinyusb_config_cdcacm_t acm = {.cdc_port = TINYUSB_CDC_ACM_0, .callback_line_state_changed = on_line};
    esp_err_t e = tinyusb_cdcacm_init(&acm);
    if (e == ESP_OK)
        e = tinyusb_console_init(TINYUSB_CDC_ACM_0);
    if (e != ESP_OK)
        return e;
    setvbuf(stdin, NULL, _IONBF, 0);
    const esp_console_config_t cc = ESP_CONSOLE_CONFIG_DEFAULT();
    e = esp_console_init(&cc);
    if (e == ESP_OK)
        esp_console_register_help_command();
    return e;
}
#endif

#if CONFIG_LR20XX_USB_NCM
static int cmd_download(int argc, char **argv)
{
    link_ncm_download_mode();
    return 0;
}
#endif

static void register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        {.command = "wifi", .help = "wifi <ssid> [password]: save the network and join it", .func = cmd_wifi},
        {.command = "status", .help = "the address, the chip, what it's doing", .func = cmd_status},
        {.command = "restart", .help = "restart", .func = cmd_restart},
#if CONFIG_LR20XX_USB_NCM
        {.command = "download", .help = "restart into the ROM's download mode, for idf.py flash", .func = cmd_download},
#endif
    };
    for (size_t i = 0; i < sizeof cmds / sizeof cmds[0]; i++)
        esp_console_cmd_register(&cmds[i]);
}

void console_start(void)
{
#if CONFIG_LR20XX_USB_NCM
    esp_err_t e = cdc_console();
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "no console: %s", esp_err_to_name(e));
        return;
    }
    register_commands();
    xTaskCreatePinnedToCore(cdc_repl, "console", 4096, NULL, 2, NULL, LINK_CORE);
#else
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t rc = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    rc.prompt = "lr20xx>";
    rc.task_priority = 2; /* far under the reader and the link */
    esp_err_t e = ESP_ERR_NOT_SUPPORTED;
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t hw = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    e = esp_console_new_repl_usb_serial_jtag(&hw, &rc, &repl);
#elif CONFIG_ESP_CONSOLE_UART_DEFAULT || CONFIG_ESP_CONSOLE_UART_CUSTOM
    esp_console_dev_uart_config_t hw = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    e = esp_console_new_repl_uart(&hw, &rc, &repl);
#endif
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "no console: %s", esp_err_to_name(e));
        return;
    }
    register_commands();
    esp_console_start_repl(repl);
#endif
}
