// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/* The transports and the network around them (main.c starts them). */
#pragma once

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include <stddef.h>
#include <stdint.h>

/* The links' and the console's core: core 1, the reader having core 0 (with the network stack); a single-core chip
 * (C3, C6, H2) runs everything on its one. */
#define LINK_CORE (portNUM_PROCESSORS > 1 ? 1 : 0)

void link_tcp_start(void);       /* link_tcp.c */
void link_discovery_start(void); /* link_tcp.c: UDP discovery */
void link_serial_start(void);    /* link_serial.c (CONFIG_LR20XX_LINK_SERIAL) */
/* The serial link's port: link_serial.c's (USB Serial/JTAG, UART) or link_usb.c's (TinyUSB CDC). */
void serial_port_init(void);
int serial_port_read(uint8_t *buf, size_t n, int wait_ms); /* bytes read, 0 if none came */
int serial_port_write(const uint8_t *data, size_t n);      /* 0, or -1 if the host isn't taking them */
bool serial_port_connected(void);
void link_ncm_start(void);       /* link_ncm.c (CONFIG_LR20XX_USB_NCM): the network over USB */
int link_ncm_status(char *out, size_t size); /* its addresses and frame counts, for the console */
void link_ncm_download_mode(void); /* restart into the ROM's download mode, for idf.py flash */

void wifi_start(void);           /* wifi.c: STA with the credentials in NVS, if any */
bool wifi_connected(void);
void console_start(void);        /* wifi.c: the serial console (wifi, status) */

void http_start(void);           /* http.c (CONFIG_LR20XX_HTTP) */
