// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * The LR20xx bridge (README.md): an ESP32-S3 with an LR20xx on its SPI bus, streaming the chip's raw I/Q to a host
 * over TCP or USB (proto/include/lr20xx_link.h), for SoapyLR20xx's "bridge" transport.
 */
#include "bridge.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "links.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_netif_init();
    esp_event_loop_create_default();

    bridge_init();
#if CONFIG_LR20XX_USB_NCM
    link_ncm_start();
#endif
    wifi_start();
#if CONFIG_LR20XX_LINK_SERIAL
    link_serial_start();
#else
    link_discovery_start();
    link_tcp_start();
#endif
    http_start();
    console_start();
}
