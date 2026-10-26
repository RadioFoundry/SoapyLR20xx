// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * The network over USB (CONFIG_LR20XX_USB_NCM): TinyUSB's NCM class on the S3's native USB, with an lwIP interface
 * on it. The host sees a USB network adapter and gets an address from the bridge's DHCP server. The TCP link,
 * discovery and HTTP listen on every interface, so they work over it as over Wi-Fi.
 *
 * Each bridge has its own /30, picked from its MAC in 10.73.0.0/16 (or set in menuconfig), so several on one host
 * don't share a subnet. The DHCP server offers no router, so the host keeps its own default route. A CDC port next
 * to the network carries the console (console_start).
 */
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dhcpserver/dhcpserver.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "links.h"
#include "lwip/ip4_addr.h"
#include "sdkconfig.h"
#include "soc/rtc_cntl_reg.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_net.h"

static const char *TAG = "ncm";

/* Frames from lwIP waiting for TinyUSB, and how long lwIP waits for room before a frame is dropped. A drop costs TCP
 * a retransmit timeout (about 1 s: lwIP has no SACK), so a slow host (a VM's USB passthrough, at 125 kS/s's many
 * small frames) gets a short wait first; past it the host isn't reading, and the frame goes as on a congested link. */
#define TX_QUEUE 64
#define TX_WAIT_MS 20

typedef struct {
    void *buf;
    uint16_t len;
} frame_t;

static esp_netif_t *ncm;
static char addr[16], peer[16];
static QueueHandle_t txq;
static atomic_uint n_in, n_out, n_dropped, n_waits;

/* lwIP to USB, on lwIP's task: a copy into the queue, for usb_tx. */
static esp_err_t ncm_transmit(void *h, void *buffer, size_t len)
{
    frame_t f = {.buf = malloc(len), .len = (uint16_t)len};
    if (!f.buf) {
        atomic_fetch_add(&n_dropped, 1);
        return ESP_OK;
    }
    memcpy(f.buf, buffer, len);
    if (xQueueSend(txq, &f, pdMS_TO_TICKS(TX_WAIT_MS)) != pdTRUE) {
        free(f.buf);
        atomic_fetch_add(&n_dropped, 1);
    }
    return ESP_OK;
}

/* The queue to TinyUSB. TinyUSB drops a frame its NTB buffers have no room for, and over full-speed USB they fill
 * whenever the stream is near the link's ~1 MB/s; every drop then costs a TCP retransmit, and the stream stalls.
 * So this waits for room. The sync send without a timeout: with one, a timeout can leave the send's semaphore
 * taken. It returns ESP_FAIL at once when there's no room, and TinyUSB has copied the frame when it returns OK. */
static void usb_tx(void *arg)
{
    frame_t f;
    for (;;) {
        xQueueReceive(txq, &f, portMAX_DELAY);
        for (;;) {
            esp_err_t e = tinyusb_net_send_sync(f.buf, f.len, NULL, portMAX_DELAY);
            if (e == ESP_OK) {
                atomic_fetch_add(&n_out, 1);
                break;
            }
            if (e == ESP_ERR_INVALID_STATE) { /* no host */
                atomic_fetch_add(&n_dropped, 1);
                break;
            }
            atomic_fetch_add(&n_waits, 1);
            vTaskDelay(1);
        }
        free(f.buf);
    }
}

static void ncm_free_rx(void *h, void *buffer) { free(buffer); }

/* USB to lwIP. TinyUSB reuses its buffer when this returns: lwIP gets a copy, freed by ncm_free_rx. */
static esp_err_t ncm_recv(void *buffer, uint16_t len, void *ctx)
{
    if (!ncm)
        return ESP_OK;
    atomic_fetch_add(&n_in, 1);
    void *copy = malloc(len);
    if (!copy)
        return ESP_ERR_NO_MEM;
    memcpy(copy, buffer, len);
    return esp_netif_receive(ncm, copy, len, copy);
}

/* The bridge's address: CONFIG_LR20XX_USB_NCM_IP on a /24, or one of 16384 /30s in 10.73.0.0/16 picked from the
 * MAC. The host gets the next address. */
static void pick(const uint8_t mac[6], esp_netif_ip_info_t *ip)
{
    uint32_t me, mask;
    if (CONFIG_LR20XX_USB_NCM_IP[0]) {
        me = lwip_ntohl(ipaddr_addr(CONFIG_LR20XX_USB_NCM_IP));
        mask = 0xFFFFFF00u;
    } else {
        uint32_t slot = ((uint32_t)mac[3] << 16 | (uint32_t)mac[4] << 8 | mac[5]) % 16384u;
        me = (10u << 24 | 73u << 16) + slot * 4 + 1;
        mask = 0xFFFFFFFCu;
    }
    ip->ip.addr = lwip_htonl(me);
    ip->gw.addr = ip->ip.addr;
    ip->netmask.addr = lwip_htonl(mask);
    esp_ip4_addr_t p = {.addr = lwip_htonl(me + 1)};
    snprintf(addr, sizeof addr, IPSTR, IP2STR(&ip->ip));
    snprintf(peer, sizeof peer, IPSTR, IP2STR(&p));
}

void link_ncm_start(void)
{
    const tinyusb_config_t tc = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tc));

    /* Two addresses from the chip's Ethernet MAC: the bridge's own, and a locally administered one for the host's
     * side of the link (NCM gives the host its adapter's address). */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_ETH);
    tinyusb_net_config_t nc = {.on_recv_callback = ncm_recv};
    memcpy(nc.mac_addr, mac, sizeof mac);
    nc.mac_addr[0] |= 0x02;
    nc.mac_addr[5] ^= 0x01;
    ESP_ERROR_CHECK(tinyusb_net_init(&nc));
    txq = xQueueCreate(TX_QUEUE, sizeof(frame_t));
    xTaskCreatePinnedToCore(usb_tx, "usb_tx", 3072, NULL, 12, NULL, LINK_CORE);

    esp_netif_ip_info_t ip = {0};
    pick(mac, &ip);
    esp_netif_inherent_config_t base = {
        .flags = ESP_NETIF_DHCP_SERVER | ESP_NETIF_FLAG_AUTOUP,
        .ip_info = &ip,
        .if_key = "USB_NCM",
        .if_desc = "usb",
        .route_prio = 5, /* under Wi-Fi's: never the bridge's default route */
    };
    memcpy(base.mac, mac, sizeof mac);
    const esp_netif_driver_ifconfig_t drv = {
        .handle = (void *)1, /* unused, but esp_netif wants one */
        .transmit = ncm_transmit,
        .driver_free_rx_buffer = ncm_free_rx,
    };
    const esp_netif_config_t cfg = {.base = &base, .driver = &drv, .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH};
    ncm = esp_netif_new(&cfg);
    if (!ncm) {
        ESP_LOGE(TAG, "no interface");
        return;
    }

    /* addresses only: no router, so the host keeps its own default route, and no DNS (the bridge answers none; a
     * Linux host otherwise sends it lookups) */
    dhcps_offer_t off = 0;
    esp_netif_dhcps_option(ncm, ESP_NETIF_OP_SET, ESP_NETIF_ROUTER_SOLICITATION_ADDRESS, &off, sizeof off);
    esp_netif_dhcps_option(ncm, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &off, sizeof off);
    esp_netif_action_start(ncm, NULL, 0, NULL);
    esp_netif_action_connected(ncm, NULL, 0, NULL);
    ESP_LOGI(TAG, "USB network up: the bridge is %s, the host gets %s by DHCP", addr, peer);
}

int link_ncm_status(char *out, size_t size)
{
    return snprintf(out, size, "usb network: %s (host %s), %u frames in, %u out, %u dropped, %u waits for room",
                    addr, peer, atomic_load(&n_in), atomic_load(&n_out), atomic_load(&n_dropped),
                    atomic_load(&n_waits));
}

void link_ncm_download_mode(void)
{
    ESP_LOGW(TAG, "restarting into the ROM's download mode on the USB Serial/JTAG port");
    vTaskDelay(pdMS_TO_TICKS(50));
    tinyusb_driver_uninstall(); /* the host sees the port go away cleanly */
    /* Give the PHY back to the USB Serial/JTAG and ask the ROM for download mode. esp_restart() resets only the
     * CPUs, and the ROM boots past the flag then: a full system reset. */
    CLEAR_PERI_REG_MASK(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_HW_USB_PHY_SEL);
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_rom_software_reset_system();
}
