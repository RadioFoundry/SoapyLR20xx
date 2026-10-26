// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * The link over a serial port (CONFIG_LR20XX_LINK_SERIAL), for chips and boards without a network to the host: the
 * USB Serial/JTAG port (the C3, C6, H2 and S3 have one), a UART (through a USB-UART bridge, or straight to the
 * host's), or a TinyUSB CDC port (S2, S3; link_usb.c). The same byte stream as over TCP.
 *
 * A serial port has no connection to accept, so a HELLO starts a session (bridge.c), and a port that stops taking
 * bytes, or a USB port the host has let go, ends one. The console and the log must be on another port: their text
 * would land in the middle of the frames.
 */
#include "bridge.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "links.h"
#include "sdkconfig.h"

#if CONFIG_LR20XX_SERIAL_USB_JTAG
#include "driver/usb_serial_jtag.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
#error "The link is on the USB Serial/JTAG port: move the console off it (ESP_CONSOLE_UART_DEFAULT or NONE, and no secondary console)"
#endif
#elif CONFIG_LR20XX_SERIAL_UART
#include "driver/uart.h"
#if CONFIG_ESP_CONSOLE_UART && CONFIG_ESP_CONSOLE_UART_NUM == CONFIG_LR20XX_SERIAL_UART_NUM
#error "The link and the console are on the same UART: move one"
#endif
#endif

static const char *TAG = "link_serial";

EXT_RAM_BSS_ATTR static bridge_link_t serial_link;

#if CONFIG_LR20XX_SERIAL_USB_JTAG

void serial_port_init(void)
{
    usb_serial_jtag_driver_config_t c = {.rx_buffer_size = 4096, .tx_buffer_size = 8192};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&c));
}

int serial_port_read(uint8_t *buf, size_t n, int wait_ms)
{
    return usb_serial_jtag_read_bytes(buf, n, pdMS_TO_TICKS(wait_ms));
}

int serial_port_write(const uint8_t *data, size_t n)
{
    while (n) {
        int k = usb_serial_jtag_write_bytes(data, n, pdMS_TO_TICKS(200));
        if (k <= 0)
            return -1; /* the host stopped reading */
        data += k;
        n -= (size_t)k;
    }
    return 0;
}

/* usb_serial_jtag_is_connected() follows the host's start-of-frame packets and can read false for a moment while
 * the host is there (a session ended at random under it): gone only after half a second of no USB. */
bool serial_port_connected(void)
{
    static int64_t last_seen;
    const int64_t now = esp_timer_get_time();
    if (usb_serial_jtag_is_connected())
        last_seen = now;
    return last_seen && now - last_seen < 500000;
}

#elif CONFIG_LR20XX_SERIAL_UART

void serial_port_init(void)
{
    const uart_config_t c = {
        .baud_rate = CONFIG_LR20XX_SERIAL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_LR20XX_SERIAL_UART_NUM, 4096, 8192, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_LR20XX_SERIAL_UART_NUM, &c));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_LR20XX_SERIAL_UART_NUM, CONFIG_LR20XX_SERIAL_TX, CONFIG_LR20XX_SERIAL_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

int serial_port_read(uint8_t *buf, size_t n, int wait_ms)
{
    return uart_read_bytes(CONFIG_LR20XX_SERIAL_UART_NUM, buf, n, pdMS_TO_TICKS(wait_ms));
}

/* A UART drains at its baud rate whether anyone listens or not: the samples it can't carry are dropped at the
 * buffer and flagged as gaps. */
int serial_port_write(const uint8_t *data, size_t n)
{
    return uart_write_bytes(CONFIG_LR20XX_SERIAL_UART_NUM, data, n) == (int)n ? 0 : -1;
}

bool serial_port_connected(void) { return true; }

#endif /* the TinyUSB CDC port is link_usb.c's */

static int serial_send(bridge_link_t *l, const uint8_t *data, size_t n) { return serial_port_write(data, n); }

static void serial_task(void *arg)
{
    static uint8_t in[512];
    bool up = false;
    for (;;) {
        const bool now = serial_port_connected();
        if (now != up) {
            if (up)
                bridge_link_closed(&serial_link);
            up = now;
            if (up)
                bridge_link_open(&serial_link);
            ESP_LOGI(TAG, "host %s", up ? "here" : "gone");
        }
        int n = serial_port_read(in, sizeof in, bridge_streaming() ? 0 : 20);
        if (!up) {
            if (n <= 0)
                vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if ((n > 0 && bridge_link_input(&serial_link, in, (size_t)n) < 0) || bridge_link_pump(&serial_link, 5) < 0) {
            /* the host stopped reading: drop the session, keep listening for the next HELLO */
            bridge_link_closed(&serial_link);
            bridge_link_open(&serial_link);
        }
    }
}

void link_serial_start(void)
{
    serial_port_init();
    serial_link.send = serial_send;
    xTaskCreatePinnedToCore(serial_task, "link_serial", 6144, NULL, 10, NULL, LINK_CORE);
}
