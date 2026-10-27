// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * The LR20xx over a HAL (lr20xx.h, lr20xx_hal.h). Every frame waits for BUSY low first (5.2: a command sent while
 * BUSY is high may not run). From RadioFoundry Gateway's components/lr2021/src/lr2021.c, with the ESP-IDF calls
 * behind the HAL (tools/upstream/files.txt).
 */
#include "lr20xx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUSY_TIMEOUT_US 200000 /* calibrations take a few ms; a stuck BUSY means a dead or unpowered chip */
#define MAX_FRAME 260          /* a command frame, or an answer and its two status bytes */
#define MAX_FIFO_FRAME 1026    /* a 1 KB FIFO transfer (the extended FIFO, 5.3.3) and its two status bytes */

struct lr20xx {
    lr20xx_hal_t hal;
    /* Aligned to 4, an ESP32's DMA receive alignment: with the received length a multiple of 4 too, its SPI driver
     * uses rx as it is (else it allocates a bounce buffer for the transfer). */
    uint8_t tx[MAX_FIFO_FRAME] __attribute__((aligned(4))), rx[MAX_FIFO_FRAME] __attribute__((aligned(4)));
    char error[96];
    uint32_t failures;
    bool own_mem; /* allocated here, not the caller's cfg.mem */
};

size_t lr20xx_handle_size(void) { return sizeof(struct lr20xx); }

static lr20xx_err_t fail(lr20xx_t *d, lr20xx_err_t e, const char *what)
{
    snprintf(d->error, sizeof d->error, "%s", what);
    d->failures++;
    return e;
}

static lr20xx_err_t wait_busy(lr20xx_t *d)
{
    int64_t t0 = lr20xx_now_us();
    for (;;) {
        int b = d->hal.busy(d->hal.ctx);
        if (b < 0)
            return fail(d, LR20XX_FAIL, "BUSY pin not readable");
        if (!b)
            return LR20XX_OK;
        int64_t dt = lr20xx_now_us() - t0;
        if (dt > BUSY_TIMEOUT_US)
            return fail(d, LR20XX_ERR_TIMEOUT, "BUSY stayed high");
        if (dt > 2000)
            lr20xx_sleep_tick();
        else
            lr20xx_delay_us(dt > 50 ? 5 : 1); /* most commands hold BUSY a few us: 5 us steps added half of that
                                                again to every frame, several thousand a second while receiving */
    }
}

/* A frame of len bytes, of which the first want come back in rx (0: none). rx is received rounded up to 4 where the
 * frame is that long (the status of a 6-byte command: 4), and not at all when nothing is wanted: an ESP32's SPI DMA
 * takes that without a buffer of its own. */
static lr20xx_err_t xfer_rx(lr20xx_t *d, size_t len, size_t want)
{
    lr20xx_err_t e = wait_busy(d);
    if (e != LR20XX_OK)
        return e;
    size_t rx = want ? (want + 3) & ~(size_t)3 : 0;
    if (rx > len)
        rx = want; /* can't round up past the frame */
    e = d->hal.xfer(d->hal.ctx, d->tx, rx ? d->rx : NULL, len, rx);
    return e == LR20XX_OK ? LR20XX_OK : fail(d, e, "SPI transfer failed");
}

lr20xx_err_t lr20xx_wake(lr20xx_t *d)
{
    /* NSS low for 100 us wakes it (4.4.1): a frame of zeros that long at 16 MHz (the slowest clock it takes much
     * longer: the frame only has to be at least that long), with a margin */
    size_t n = ((size_t)(16000000 / 1000000 * 120 / 8) + 4) & ~(size_t)3;
    memset(d->tx, 0, n);
    lr20xx_err_t e = d->hal.xfer(d->hal.ctx, d->tx, NULL, n, 0);
    if (e != LR20XX_OK)
        return fail(d, e, "SPI transfer failed");
    return wait_busy(d);
}

/* A frame whose answer is all of it. */
static lr20xx_err_t xfer(lr20xx_t *d, size_t len) { return xfer_rx(d, len, len); }

static lr20xx_err_t check(lr20xx_t *d)
{
    lr20xx_status_t s = lr20xx_status(d->rx[0], d->rx[1]);
    if (s.cmd == LR20XX_CMD_FAIL || s.cmd == LR20XX_CMD_PERR) {
        snprintf(d->error, sizeof d->error, "command %s (chip in %s)", s.cmd ? "not understood" : "failed",
                 lr20xx_mode_name(s.mode));
        d->failures++;
        return LR20XX_FAIL;
    }
    return LR20XX_OK;
}

lr20xx_err_t lr20xx_get_status(lr20xx_t *d, lr20xx_status_t *st)
{
    memset(d->tx, 0, 6);
    lr20xx_cmd(d->tx, LR20XX_GET_STATUS);
    lr20xx_err_t e = xfer_rx(d, 6, 2);
    if (e == LR20XX_OK)
        *st = lr20xx_status(d->rx[0], d->rx[1]);
    return e;
}

lr20xx_err_t lr20xx_send(lr20xx_t *d, const uint8_t *frame, size_t len)
{
    if (len > MAX_FRAME)
        return LR20XX_ERR_INVALID_SIZE;
    memcpy(d->tx, frame, len);
    return xfer_rx(d, len, 0);
}

lr20xx_err_t lr20xx_write(lr20xx_t *d, const uint8_t *frame, size_t len)
{
    if (len > MAX_FRAME)
        return LR20XX_ERR_INVALID_SIZE;
    memcpy(d->tx, frame, len);
    lr20xx_err_t e = xfer_rx(d, len, 0);
    if (e != LR20XX_OK)
        return e;
    /* the status a frame starts with is the previous command's: ask for this one's (6.7.1) */
    lr20xx_status_t s;
    e = lr20xx_get_status(d, &s);
    return e == LR20XX_OK ? check(d) : e;
}

lr20xx_err_t lr20xx_read(lr20xx_t *d, const uint8_t *frame, size_t len, uint8_t *out, size_t n)
{
    if (len > MAX_FRAME || n + 2 > MAX_FRAME)
        return LR20XX_ERR_INVALID_SIZE;
    memcpy(d->tx, frame, len);
    lr20xx_err_t e = xfer_rx(d, len, 0);
    if (e != LR20XX_OK)
        return e;
    memset(d->tx, 0, n + 2);
    e = xfer(d, n + 2);
    if (e != LR20XX_OK)
        return e;
    lr20xx_status_t s = lr20xx_status(d->rx[0], d->rx[1]);
    if (s.cmd != LR20XX_CMD_DATA && s.cmd != LR20XX_CMD_OK) {
        e = check(d);
        return e != LR20XX_OK ? e : fail(d, LR20XX_FAIL, "unexpected command status"); /* a reserved code: no answer */
    }
    memcpy(out, d->rx + 2, n);
    return LR20XX_OK;
}

lr20xx_err_t lr20xx_read_fifo(lr20xx_t *d, uint8_t *out, size_t n)
{
    if (n + 2 > MAX_FIFO_FRAME)
        return LR20XX_ERR_INVALID_SIZE;
    memset(d->tx, 0, n + 2);
    lr20xx_cmd(d->tx, LR20XX_READ_RX_FIFO);
    lr20xx_err_t e = xfer(d, n + 2);
    if (e == LR20XX_OK)
        memcpy(out, d->rx + 2, n);
    return e;
}

lr20xx_err_t lr20xx_version(lr20xx_t *d, uint8_t *major, uint8_t *minor)
{
    uint8_t f[2], r[2] = {0};
    lr20xx_err_t e = lr20xx_read(d, f, lr20xx_cmd(f, LR20XX_GET_VERSION), r, 2);
    *major = r[0];
    *minor = r[1];
    return e;
}

lr20xx_err_t lr20xx_errors(lr20xx_t *d, uint16_t *errors)
{
    uint8_t f[2], r[2] = {0};
    lr20xx_err_t e = lr20xx_read(d, f, lr20xx_cmd(f, LR20XX_GET_ERRORS), r, 2);
    *errors = (uint16_t)(r[0] << 8 | r[1]);
    return e;
}

lr20xx_err_t lr20xx_rx_fifo_level(lr20xx_t *d, uint16_t *level)
{
    uint8_t f[2], r[2] = {0};
    lr20xx_err_t e = lr20xx_read(d, f, lr20xx_cmd(f, LR20XX_GET_RX_FIFO_LEVEL), r, 2);
    *level = (uint16_t)(r[0] << 8 | r[1]);
    return e;
}

lr20xx_err_t lr20xx_fifo_flags(lr20xx_t *d, uint8_t *rx, uint8_t *tx)
{
    uint8_t f[2], r[2] = {0};
    lr20xx_err_t e = lr20xx_read(d, f, lr20xx_cmd(f, LR20XX_GET_AND_CLEAR_FIFO_IRQ_FLAGS), r, 2);
    *rx = r[0];
    *tx = r[1];
    return e;
}

lr20xx_err_t lr20xx_fifo_flags_peek(lr20xx_t *d, uint8_t *rx, uint8_t *tx)
{
    uint8_t f[2], r[2] = {0};
    lr20xx_err_t e = lr20xx_read(d, f, lr20xx_cmd(f, LR20XX_GET_FIFO_IRQ_FLAGS), r, 2);
    *rx = r[0];
    *tx = r[1];
    return e;
}

lr20xx_err_t lr20xx_tx_fifo_level(lr20xx_t *d, uint16_t *level)
{
    uint8_t f[2], r[2] = {0};
    lr20xx_err_t e = lr20xx_read(d, f, lr20xx_cmd(f, LR20XX_GET_TX_FIFO_LEVEL), r, 2);
    *level = (uint16_t)(r[0] << 8 | r[1]);
    return e;
}

lr20xx_err_t lr20xx_write_tx_fifo(lr20xx_t *d, const uint8_t *data, size_t n)
{
    if (n + 2 > MAX_FIFO_FRAME)
        return LR20XX_ERR_INVALID_SIZE;
    lr20xx_cmd(d->tx, LR20XX_WRITE_TX_FIFO);
    memcpy(d->tx + 2, data, n);
    return xfer_rx(d, n + 2, 0);
}

lr20xx_err_t lr20xx_read32(lr20xx_t *d, uint32_t addr, uint32_t *value)
{
    uint8_t f[8], r[4] = {0};
    lr20xx_err_t e = lr20xx_read(d, f, lr20xx_cmd_read_reg32(f, addr, 1), r, 4);
    *value = (uint32_t)r[0] << 24 | (uint32_t)r[1] << 16 | (uint32_t)r[2] << 8 | r[3];
    return e;
}

lr20xx_err_t lr20xx_read_mem(lr20xx_t *d, uint32_t addr, uint32_t *w, int n)
{
    uint8_t f[8], r[256];
    if (n < 1 || n > 64)
        return LR20XX_ERR_INVALID_SIZE;
    lr20xx_err_t e = lr20xx_read(d, f, lr20xx_cmd_read_reg32(f, addr, (uint8_t)n), r, 4 * (size_t)n);
    for (int i = 0; i < n; i++)
        w[i] = (uint32_t)r[4 * i] << 24 | (uint32_t)r[4 * i + 1] << 16 | (uint32_t)r[4 * i + 2] << 8 | r[4 * i + 3];
    return e;
}

lr20xx_err_t lr20xx_irq(lr20xx_t *d, uint32_t *irq)
{
    memset(d->tx, 0, 6);
    lr20xx_cmd(d->tx, LR20XX_GET_STATUS);
    lr20xx_err_t e = xfer(d, 6);
    *irq = e == LR20XX_OK ? (uint32_t)d->rx[2] << 24 | (uint32_t)d->rx[3] << 16 | (uint32_t)d->rx[4] << 8 | d->rx[5]
                          : 0;
    return e;
}

lr20xx_err_t lr20xx_reset(lr20xx_t *d)
{
    lr20xx_err_t e = d->hal.nreset(d->hal.ctx, 0);
    if (e != LR20XX_OK)
        return fail(d, e, "NRESET not driven");
    lr20xx_sleep_ms(2);
    if ((e = d->hal.nreset(d->hal.ctx, 1)) != LR20XX_OK)
        return fail(d, e, "NRESET not driven");
    lr20xx_sleep_ms(1);
    return wait_busy(d);
}

bool lr20xx_wait_irq(lr20xx_t *d, int timeout_ms) { return d->hal.wait_irq && d->hal.wait_irq(d->hal.ctx, timeout_ms); }

bool lr20xx_has_irq(const lr20xx_t *d) { return d->hal.wait_irq != NULL; }

int lr20xx_irq_pin(lr20xx_t *d) { return d->hal.irq ? d->hal.irq(d->hal.ctx) : -1; }

lr20xx_err_t lr20xx_open(const lr20xx_config_t *cfg, lr20xx_t **out)
{
    const lr20xx_hal_t *h = &cfg->hal;
    if (!h->xfer || !h->busy || !h->nreset || (cfg->mem && ((uintptr_t)cfg->mem & 3))) {
        if (h->close)
            h->close(h->ctx);
        return LR20XX_ERR_INVALID_ARG;
    }
    lr20xx_t *d = cfg->mem ? cfg->mem : calloc(1, sizeof *d);
    if (!d) {
        if (h->close)
            h->close(h->ctx);
        return LR20XX_ERR_NO_MEM;
    }
    if (cfg->mem)
        memset(d, 0, sizeof *d);
    d->own_mem = !cfg->mem;
    d->hal = *h;
    lr20xx_err_t e = lr20xx_reset(d);
    if (e != LR20XX_OK) {
        lr20xx_close(d);
        return e;
    }
    *out = d;
    return LR20XX_OK;
}

lr20xx_err_t lr20xx_set_clock(lr20xx_t *d, int hz, int input_delay_ns)
{
    if (hz <= 0 || hz > 80000000)
        return LR20XX_ERR_INVALID_ARG;
    if (!d->hal.set_clock)
        return LR20XX_ERR_NOT_SUPPORTED;
    lr20xx_err_t e = d->hal.set_clock(d->hal.ctx, hz, input_delay_ns);
    return e == LR20XX_OK ? LR20XX_OK : fail(d, e, "SPI clock not changed");
}

void lr20xx_close(lr20xx_t *d)
{
    if (!d)
        return;
    d->hal.nreset(d->hal.ctx, 0);
    if (d->hal.close)
        d->hal.close(d->hal.ctx);
    if (d->own_mem)
        free(d);
}

const char *lr20xx_last_error(const lr20xx_t *d) { return d->error; }
uint32_t lr20xx_failures(const lr20xx_t *d) { return d->failures; }

const char *lr20xx_err_name(lr20xx_err_t e)
{
    switch (e) {
    case LR20XX_OK: return "LR20XX_OK";
    case LR20XX_FAIL: return "LR20XX_FAIL";
    case LR20XX_ERR_NO_MEM: return "LR20XX_ERR_NO_MEM";
    case LR20XX_ERR_INVALID_ARG: return "LR20XX_ERR_INVALID_ARG";
    case LR20XX_ERR_INVALID_STATE: return "LR20XX_ERR_INVALID_STATE";
    case LR20XX_ERR_INVALID_SIZE: return "LR20XX_ERR_INVALID_SIZE";
    case LR20XX_ERR_NOT_FOUND: return "LR20XX_ERR_NOT_FOUND";
    case LR20XX_ERR_NOT_SUPPORTED: return "LR20XX_ERR_NOT_SUPPORTED";
    case LR20XX_ERR_TIMEOUT: return "LR20XX_ERR_TIMEOUT";
    case LR20XX_ERR_INVALID_RESPONSE: return "LR20XX_ERR_INVALID_RESPONSE";
    case LR20XX_ERR_INVALID_CRC: return "LR20XX_ERR_INVALID_CRC";
    default: return "unknown error";
    }
}
