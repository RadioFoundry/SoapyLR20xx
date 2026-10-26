// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * A transmission from a pull callback (lr20xx.h, lr20xx_tx_stream): prefill the 1 KB TX FIFO, key up, keep the
 * FIFO topped up, and start the next run on each TX_DONE until the data ends. Only the driver's public calls:
 * docs/iq.md section 3 has the facts (R3.11-R3.17).
 */
#include <stdlib.h>
#include <string.h>

#include "lr20xx_port.h"
#include "lr20xx.h"

#define FIFO_BYTES 1020 /* the 1 KB TX FIFO's size register, 0x3FC (R2.22) */

static lr20xx_err_t send_frame(lr20xx_t *d, const uint8_t *f, size_t n) { return lr20xx_send(d, f, n); }

static void standby(lr20xx_t *d)
{
    uint8_t f[4];
    send_frame(d, f, lr20xx_cmd_u8(f, LR20XX_SET_STANDBY, 0));
}

lr20xx_err_t lr20xx_tx_stream(lr20xx_t *d, const lr20xx_tx_stream_cfg_t *c, lr20xx_tx_pull_t pull, void *ctx,
                           lr20xx_tx_stream_stats_t *st)
{
    lr20xx_tx_stream_stats_t z = {0};
    if (!st)
        st = &z;
    memset(st, 0, sizeof *st);
    if (!c || !c->start || !c->start_len)
        return LR20XX_ERR_INVALID_ARG;
    uint8_t f[12];
    lr20xx_err_t e;

    if (c->run_bytes == 0) { /* a carrier: key up and leave it on (SetStandby ends it) */
        st->key_us = lr20xx_now_us();
        return lr20xx_write(d, c->start, c->start_len);
    }
    if (!pull)
        return LR20XX_ERR_INVALID_ARG;
    uint8_t *buf = c->buf && c->buf_len >= FIFO_BYTES ? c->buf : malloc(FIFO_BYTES);
    if (!buf)
        return LR20XX_ERR_NO_MEM;
    const int chunk = c->chunk > 0 ? c->chunk : 256;
    const int64_t max_us = (int64_t)(c->max_ms > 0 ? c->max_ms : 10000) * 1000;

    /* an empty FIFO, its flags and the IRQs a run raises cleared */
    if ((e = lr20xx_write(d, f, lr20xx_cmd(f, LR20XX_CLEAR_TX_FIFO))) != LR20XX_OK ||
        (e = lr20xx_write(d, f, lr20xx_cmd_clear_fifo_flags(f, 0, 0x3F))) != LR20XX_OK ||
        (e = lr20xx_write(d, f, lr20xx_cmd_clear_irq(f, LR20XX_IRQ_TX_DONE | LR20XX_IRQ_TIMEOUT))) != LR20XX_OK)
        goto out;

    bool pulled_all = false;
    size_t n = pull(ctx, buf, FIFO_BYTES);
    if (n == 0) {
        e = LR20XX_ERR_INVALID_SIZE; /* nothing to send */
        goto out;
    }
    if ((e = lr20xx_write_tx_fifo(d, buf, n)) != LR20XX_OK)
        goto out;
    st->sent = n;
    if ((e = send_frame(d, c->start, c->start_len)) != LR20XX_OK)
        goto out;
    st->key_us = lr20xx_now_us();

    for (;;) {
        const int64_t now = lr20xx_now_us();
        if (now - st->key_us > max_us) {
            st->timed_out = true;
            e = LR20XX_ERR_TIMEOUT;
            break;
        }
        uint32_t irq = 0;
        uint16_t level = 0;
        if ((e = lr20xx_irq(d, &irq)) != LR20XX_OK || (e = lr20xx_tx_fifo_level(d, &level)) != LR20XX_OK)
            break;
        if (irq & LR20XX_IRQ_TIMEOUT) {
            st->timed_out = true;
            e = LR20XX_ERR_TIMEOUT;
            break;
        }
        if (irq & LR20XX_IRQ_TX_DONE) {
            st->runs++;
            st->end_us = now;
            /* a run that ended short of its bytes with the FIFO still holding some sent nothing (R3.17): the caller
             * sends it again. Short with the FIFO empty is an underflow: the run went out, holding its last sample. */
            const size_t taken = st->sent - level;
            if (level > 0 && taken + 4 < (size_t)st->runs * c->run_bytes) {
                st->early = true;
                e = LR20XX_ERR_INVALID_RESPONSE;
                break;
            }
            if (pulled_all && level == 0) {
                e = LR20XX_OK;
                break;
            }
            /* the next run: the setup persists, SetTx again (R3.15) */
            if ((e = lr20xx_write(d, f, lr20xx_cmd_clear_irq(f, LR20XX_IRQ_TX_DONE))) != LR20XX_OK ||
                (e = send_frame(d, c->start, c->start_len)) != LR20XX_OK)
                break;
            continue;
        }
        const int free_bytes = FIFO_BYTES - (int)level;
        if (!pulled_all && free_bytes >= chunk) {
            n = pull(ctx, buf, (size_t)free_bytes);
            if (n == 0) {
                pulled_all = true;
            } else {
                if ((e = lr20xx_write_tx_fifo(d, buf, n)) != LR20XX_OK)
                    break;
                st->sent += n;
                st->refills++;
            }
            continue;
        }
        /* nothing to do: sleep a tick if the FIFO lasts well past it, else a short wait */
        const int64_t until_chunk_us = c->bytes_per_s ? (int64_t)(chunk - free_bytes) * 1000000 / c->bytes_per_s : 0;
        if (pulled_all || until_chunk_us > 3 * (int64_t)LR20XX_TICK_MS * 1000)
            lr20xx_sleep_tick();
        else
            lr20xx_delay_us(c->poll_us > 0 ? (uint32_t)c->poll_us : 100);
    }

out:;
    uint8_t rx = 0, tx = 0;
    lr20xx_fifo_flags_peek(d, &rx, &tx);
    st->tx_flags = tx;
    lr20xx_status_t s;
    if (e != LR20XX_OK && lr20xx_get_status(d, &s) == LR20XX_OK && s.mode == LR20XX_MODE_TX)
        standby(d);
    if (buf != c->buf)
        free(buf);
    return e;
}
