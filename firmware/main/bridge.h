// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * The bridge: the chip, the reader task and its sample buffer, and the link's commands (lr20xx_link.h) for
 * whichever transport carries them (link_tcp.c, link_usb.c). One host at a time.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lr20xx_link.h"

/* Open the chip and start the reader. The bridge runs without a chip too: HELLO then says so (caps 0) and an
 * event gives the error. */
void bridge_init(void);

/* A host's connection, as a transport sees it. */
typedef struct bridge_link {
    /* Send all of n bytes; < 0 if the connection failed. */
    int (*send)(struct bridge_link *l, const uint8_t *data, size_t n);
    /* Whether the host has sent something not yet read (NULL: not known): the pump's waits for samples end then,
     * so a request isn't held behind a frame filling */
    bool (*pending)(struct bridge_link *l);
    void *ctx;
    lr20xx_link_parser_t parser;
    uint8_t out[LR20XX_LINK_HEADER + LR20XX_LINK_MAX_PAYLOAD];
    bool hello_sent_event; /* the chip's error told after the HELLO */
} bridge_link_t;

/* A new host (one at a time: the transport closes the old one first). */
void bridge_link_open(bridge_link_t *l);
/* Bytes from the host: frames handled, answers sent. < 0 if a send failed. */
int bridge_link_input(bridge_link_t *l, const uint8_t *data, size_t n);
/* Send what's waiting: samples (waiting up to wait_ms for them while receiving) and events. < 0 if a send
 * failed. */
int bridge_link_pump(bridge_link_t *l, int wait_ms);
/* The host went away: receiving stops, a transmission begun is dropped. */
void bridge_link_closed(bridge_link_t *l);
/* Whether samples are flowing (the transport waits on the buffer then, not on the host). */
bool bridge_streaming(void);

/* The HELLO answer's payload (LR20XX_LINK_HELLO_SIZE bytes), for UDP discovery too. */
size_t bridge_hello(uint8_t *p);

/* For OTA: stop receiving and keep the chip idle until resumed (true if it was receiving). */
bool bridge_pause(void);
void bridge_resume(bool was_receiving);

/* A line of JSON for GET /api/status (without braces): the chip, receiving, the counts. */
int bridge_status_json(char *buf, size_t n);
