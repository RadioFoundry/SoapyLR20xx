// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * The serial link's TinyUSB CDC port (CONFIG_LR20XX_SERIAL_TINYUSB: the S2's and S3's native USB; link_serial.c has
 * the link). The host opening the port (DTR up) is a connection, closing it the end of one. Full speed, like the
 * USB Serial/JTAG port, but the S3 can carry its network instead (CONFIG_LR20XX_USB_NCM), which is quicker.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "links.h"
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_default_config.h"

static StreamBufferHandle_t rx;
static volatile bool dtr;

static void on_rx(int itf, cdcacm_event_t *event)
{
    static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
    size_t n = 0;
    if (tinyusb_cdcacm_read(itf, buf, sizeof buf, &n) == ESP_OK && n)
        xStreamBufferSend(rx, buf, n, 0);
}

static void on_line(int itf, cdcacm_event_t *event) { dtr = event->line_state_changed_data.dtr; }

void serial_port_init(void)
{
    rx = xStreamBufferCreate(4096, 1);
    const tinyusb_config_t tc = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tc));
    const tinyusb_config_cdcacm_t acm = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = on_rx,
        .callback_line_state_changed = on_line,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm));
}

int serial_port_read(uint8_t *buf, size_t n, int wait_ms)
{
    return (int)xStreamBufferReceive(rx, buf, n, pdMS_TO_TICKS(wait_ms));
}

int serial_port_write(const uint8_t *data, size_t n)
{
    while (n) {
        if (!dtr)
            return -1;
        size_t k = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, data, n);
        data += k;
        n -= k;
        if (tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(200)) != ESP_OK && !k)
            return -1; /* the host stopped reading */
    }
    return 0;
}

bool serial_port_connected(void) { return dtr; }
