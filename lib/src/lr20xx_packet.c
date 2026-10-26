// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * Send and receive one packet with the chip's own modems (LoRa, FSK, OOK, FLRC and the rest; lr20xx.h,
 * lr20xx_packet_send / _receive). The modem is set up first (its lr20xx_<modem>_* frames); these move the payload
 * through the FIFO, start TX or RX, and wait for the IRQ that ends it. Payloads longer than the FIFO are fed or
 * drained as they go (DS 5.3.3). Only the driver's public calls.
 */
#include <string.h>

#include "lr20xx_port.h"
#include "lr20xx.h"
#include "lr20xx_radio.h"

/* the IRQs that end a packet (DS table 5-20) */
#define IRQ_ENDS_RX                                                                                                   \
    (LR20XX_IRQ_RX_DONE | LR20XX_IRQ_TIMEOUT | LR20XX_IRQ_LEN_ERROR | LR20XX_IRQ_ADDR_ERROR | LR20XX_IRQ_LORA_HEADER_ERR)
#define CHUNK 128                   /* FIFO bytes a transfer, at most (the driver's FIFO calls take 258) */

static size_t fifo_size(const lr20xx_packet_cfg_t *c) { return c && c->fifo_bytes ? c->fifo_bytes : 256; }
static int64_t max_us(const lr20xx_packet_cfg_t *c) { return (int64_t)(c && c->max_ms > 0 ? c->max_ms : 10000) * 1000; }

/* Wait a little for something to happen: cfg.poll_us, or the IRQ pin if there is one, else a tick. */
static void idle(lr20xx_t *d, const lr20xx_packet_cfg_t *c)
{
    if (c && c->poll_us > 0)
        lr20xx_delay_us((uint32_t)c->poll_us);
    else if (!lr20xx_wait_irq(d, 1))
        lr20xx_sleep_tick();
}

lr20xx_err_t lr20xx_packet_send(lr20xx_t *d, const uint8_t *data, size_t n, const lr20xx_packet_cfg_t *c,
                             lr20xx_packet_result_t *r)
{
    lr20xx_packet_result_t z;
    if (!r)
        r = &z;
    memset(r, 0, sizeof *r);
    if (!d || (!data && n))
        return LR20XX_ERR_INVALID_ARG;
    const size_t cap = fifo_size(c);
    uint8_t f[8];
    lr20xx_err_t e;
    size_t sent = 0;
    /* from standby: the FIFO and IRQ commands are refused in RX or TX (a run left going, or another caller's) */
    if ((e = lr20xx_write(d, f, lr20xx_cmd_u8(f, LR20XX_SET_STANDBY, 0))) != LR20XX_OK ||
        (e = lr20xx_write(d, f, lr20xx_cmd(f, LR20XX_CLEAR_TX_FIFO))) != LR20XX_OK ||
        (e = lr20xx_write(d, f, lr20xx_cmd_clear_irq(f, LR20XX_IRQ_ALL))) != LR20XX_OK)
        goto out;
    while (sent < n && sent < cap) { /* the FIFO full before the start */
        size_t k = n - sent < CHUNK ? n - sent : CHUNK;
        if (k > cap - sent)
            k = cap - sent;
        if ((e = lr20xx_write_tx_fifo(d, data + sent, k)) != LR20XX_OK)
            goto out;
        sent += k;
    }
    if ((e = lr20xx_write(d, f, lr20xx_cmd_set_tx(f, c ? c->timeout : 0))) != LR20XX_OK)
        goto out;
    const int64_t t0 = lr20xx_now_us();
    for (;;) {
        uint32_t irq = 0;
        if ((e = lr20xx_irq(d, &irq)) != LR20XX_OK)
            break;
        r->irq |= irq;
        if (irq & LR20XX_IRQ_TX_DONE) {
            r->done_us = lr20xx_now_us() - t0;
            break;
        }
        if (irq & LR20XX_IRQ_TIMEOUT) {
            e = LR20XX_ERR_TIMEOUT;
            break;
        }
        if (lr20xx_now_us() - t0 > max_us(c)) {
            e = LR20XX_ERR_TIMEOUT;
            break;
        }
        if (sent < n) { /* top the FIFO up as it drains */
            uint16_t level = 0;
            if ((e = lr20xx_tx_fifo_level(d, &level)) != LR20XX_OK)
                break;
            size_t room = cap > level ? cap - level : 0;
            if (room >= CHUNK || room >= n - sent) {
                size_t k = n - sent < room ? n - sent : room;
                if (k > CHUNK)
                    k = CHUNK;
                if ((e = lr20xx_write_tx_fifo(d, data + sent, k)) != LR20XX_OK)
                    break;
                sent += k;
                continue;
            }
        }
        idle(d, c);
    }
out:
    r->len = sent;
    r->timed_out = e == LR20XX_ERR_TIMEOUT;
    if (e != LR20XX_OK) {
        lr20xx_status_t s;
        if (lr20xx_get_status(d, &s) == LR20XX_OK && s.mode == LR20XX_MODE_TX)
            lr20xx_write(d, f, lr20xx_cmd_u8(f, LR20XX_SET_STANDBY, 0));
    }
    return e;
}

/* Up to want bytes of the RX FIFO into buf from got on, in transfers the driver takes. */
static lr20xx_err_t drain(lr20xx_t *d, uint8_t *buf, size_t max, size_t *got, size_t want)
{
    while (want > 0) {
        size_t k = want < CHUNK ? want : CHUNK;
        uint8_t tmp[CHUNK];
        lr20xx_err_t e = lr20xx_read_fifo(d, tmp, k);
        if (e != LR20XX_OK)
            return e;
        size_t keep = *got < max ? (max - *got < k ? max - *got : k) : 0;
        memcpy(buf + *got, tmp, keep);
        *got += k;
        want -= k;
    }
    return LR20XX_OK;
}

lr20xx_err_t lr20xx_packet_receive(lr20xx_t *d, uint8_t *buf, size_t max, const lr20xx_packet_cfg_t *c,
                                lr20xx_packet_result_t *r)
{
    lr20xx_packet_result_t z;
    if (!r)
        r = &z;
    memset(r, 0, sizeof *r);
    if (!d || (!buf && max))
        return LR20XX_ERR_INVALID_ARG;
    uint8_t f[8];
    lr20xx_err_t e;
    size_t got = 0;
    if ((e = lr20xx_write(d, f, lr20xx_cmd_u8(f, LR20XX_SET_STANDBY, 0))) != LR20XX_OK ||
        (e = lr20xx_write(d, f, lr20xx_cmd(f, LR20XX_CLEAR_RX_FIFO))) != LR20XX_OK ||
        (e = lr20xx_write(d, f, lr20xx_cmd_clear_irq(f, LR20XX_IRQ_ALL))) != LR20XX_OK ||
        (e = lr20xx_write(d, f, lr20xx_cmd_set_rx(f, c ? c->timeout : 0))) != LR20XX_OK)
        goto out;
    const int64_t t0 = lr20xx_now_us();
    for (;;) {
        uint32_t irq = 0;
        if ((e = lr20xx_irq(d, &irq)) != LR20XX_OK)
            break;
        r->irq |= irq;
        if (irq & IRQ_ENDS_RX) {
            r->done_us = lr20xx_now_us() - t0;
            break;
        }
        if (lr20xx_now_us() - t0 > max_us(c)) {
            e = LR20XX_ERR_TIMEOUT;
            break;
        }
        /* a long packet: take what's in the FIFO while it comes in */
        uint16_t level = 0;
        if ((e = lr20xx_rx_fifo_level(d, &level)) != LR20XX_OK)
            break;
        if (level >= CHUNK) {
            if ((e = drain(d, buf, max, &got, CHUNK)) != LR20XX_OK)
                break;
            continue;
        }
        idle(d, c);
    }
    if (e == LR20XX_OK && (r->irq & LR20XX_IRQ_RX_DONE)) {
        uint8_t len[2] = {0};
        uint8_t q[2];
        if ((e = lr20xx_read(d, q, lr20xx_radio_get_rx_pkt_length(q), len, 2)) == LR20XX_OK) {
            r->len = (size_t)len[0] << 8 | len[1];
            uint16_t level = 0;
            if ((e = lr20xx_rx_fifo_level(d, &level)) == LR20XX_OK)
                e = drain(d, buf, max, &got, level);
        }
    }
out:
    r->crc_error = (r->irq & LR20XX_IRQ_CRC_ERROR) != 0;
    r->timed_out = e == LR20XX_ERR_TIMEOUT || (!(r->irq & LR20XX_IRQ_RX_DONE) && (r->irq & LR20XX_IRQ_TIMEOUT));
    if (e == LR20XX_OK && !(r->irq & LR20XX_IRQ_RX_DONE))
        e = r->timed_out ? LR20XX_ERR_TIMEOUT : LR20XX_ERR_INVALID_RESPONSE; /* a header, length or address error */
    if (e == LR20XX_OK && r->crc_error)
        e = LR20XX_ERR_INVALID_CRC;
    if (e == LR20XX_OK && got > max)
        e = LR20XX_ERR_INVALID_SIZE; /* the packet didn't fit: buf has its first max bytes */
    if (e != LR20XX_OK) {
        lr20xx_status_t s;
        if (lr20xx_get_status(d, &s) == LR20XX_OK && s.mode == LR20XX_MODE_RX)
            lr20xx_write(d, f, lr20xx_cmd_u8(f, LR20XX_SET_STANDBY, 0));
    }
    return e;
}
