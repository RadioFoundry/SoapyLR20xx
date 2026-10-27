// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/* The bridge link's encoding (lr20xx_link.h). */
#include "lr20xx_link.h"

#include <string.h>

void lr20xx_link_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

void lr20xx_link_put_u32(uint8_t *p, uint32_t v)
{
    for (int i = 0; i < 4; i++)
        p[i] = (uint8_t)(v >> 8 * i);
}

void lr20xx_link_put_u64(uint8_t *p, uint64_t v)
{
    for (int i = 0; i < 8; i++)
        p[i] = (uint8_t)(v >> 8 * i);
}

void lr20xx_link_put_f32(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, 4);
    lr20xx_link_put_u32(p, u);
}

void lr20xx_link_put_f64(uint8_t *p, double v)
{
    uint64_t u;
    memcpy(&u, &v, 8);
    lr20xx_link_put_u64(p, u);
}

uint16_t lr20xx_link_u16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }

uint32_t lr20xx_link_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

uint64_t lr20xx_link_u64(const uint8_t *p) { return lr20xx_link_u32(p) | (uint64_t)lr20xx_link_u32(p + 4) << 32; }

float lr20xx_link_f32(const uint8_t *p)
{
    uint32_t u = lr20xx_link_u32(p);
    float v;
    memcpy(&v, &u, 4);
    return v;
}

double lr20xx_link_f64(const uint8_t *p)
{
    uint64_t u = lr20xx_link_u64(p);
    double v;
    memcpy(&v, &u, 8);
    return v;
}

/* A frame's header around a payload of n bytes already at buf + 4 (or copied there from src). */
static size_t frame(uint8_t *buf, size_t cap, uint8_t type, size_t n)
{
    if (n > LR20XX_LINK_MAX_PAYLOAD || LR20XX_LINK_HEADER + n > cap)
        return 0;
    buf[0] = LR20XX_LINK_MAGIC;
    buf[1] = type;
    lr20xx_link_put_u16(buf + 2, (uint16_t)n);
    return LR20XX_LINK_HEADER + n;
}

size_t lr20xx_link_req(uint8_t *buf, size_t cap, uint16_t seq, uint16_t cmd, const void *args, size_t n)
{
    if (LR20XX_LINK_HEADER + 4 + n > cap || 4 + n > LR20XX_LINK_MAX_PAYLOAD)
        return 0;
    lr20xx_link_put_u16(buf + 4, seq);
    lr20xx_link_put_u16(buf + 6, cmd);
    if (n)
        memmove(buf + 8, args, n);
    return frame(buf, cap, LR20XX_LINK_REQ, 4 + n);
}

size_t lr20xx_link_rsp(uint8_t *buf, size_t cap, uint16_t seq, int32_t err, const void *ans, size_t n)
{
    if (LR20XX_LINK_HEADER + 6 + n > cap || 6 + n > LR20XX_LINK_MAX_PAYLOAD)
        return 0;
    lr20xx_link_put_u16(buf + 4, seq);
    lr20xx_link_put_u32(buf + 6, (uint32_t)err);
    if (n)
        memmove(buf + 10, ans, n);
    return frame(buf, cap, LR20XX_LINK_RSP, 6 + n);
}

size_t lr20xx_link_event(uint8_t *buf, size_t cap, uint16_t kind, int32_t err, const char *text)
{
    size_t n = text ? strlen(text) : 0;
    if (n > LR20XX_LINK_MAX_PAYLOAD - 6)
        n = LR20XX_LINK_MAX_PAYLOAD - 6;
    if (LR20XX_LINK_HEADER + 6 + n > cap)
        return 0;
    lr20xx_link_put_u16(buf + 4, kind);
    lr20xx_link_put_u32(buf + 6, (uint32_t)err);
    if (n)
        memcpy(buf + 10, text, n);
    return frame(buf, cap, LR20XX_LINK_EVENT, 6 + n);
}

size_t lr20xx_link_rx_data(uint8_t *buf, size_t cap, const lr20xx_link_rx_hdr_t *h, const uint8_t *samples, size_t n)
{
    if (LR20XX_LINK_HEADER + LR20XX_LINK_RX_HDR + n > cap || LR20XX_LINK_RX_HDR + n > LR20XX_LINK_MAX_PAYLOAD)
        return 0;
    if (n)
        memmove(buf + LR20XX_LINK_HEADER + LR20XX_LINK_RX_HDR, samples, n);
    lr20xx_link_put_rx_hdr(buf + LR20XX_LINK_HEADER, h);
    return frame(buf, cap, LR20XX_LINK_RX_DATA, LR20XX_LINK_RX_HDR + n);
}

size_t lr20xx_link_tx_data(uint8_t *buf, size_t cap, const uint8_t *samples, size_t n)
{
    if (LR20XX_LINK_HEADER + n > cap || n > LR20XX_LINK_MAX_PAYLOAD)
        return 0;
    if (n)
        memmove(buf + LR20XX_LINK_HEADER, samples, n);
    return frame(buf, cap, LR20XX_LINK_TX_DATA, n);
}

static void put_str(uint8_t *p, const char *s, size_t n)
{
    memset(p, 0, n);
    if (s)
        memcpy(p, s, strnlen(s, n - 1));
}

static void get_str(const uint8_t *p, char *s, size_t n)
{
    memcpy(s, p, n);
    s[n - 1] = 0;
}

size_t lr20xx_link_put_hello(uint8_t *p, const lr20xx_link_hello_t *v)
{
    lr20xx_link_put_u16(p, v->version);
    p[2] = v->rom_major;
    p[3] = v->rom_minor;
    p[4] = v->rom_known;
    p[5] = v->caps;
    lr20xx_link_put_u32(p + 6, v->tx_max_samples);
    lr20xx_link_put_f64(p + 10, v->freq_min);
    lr20xx_link_put_f64(p + 18, v->freq_max);
    lr20xx_link_put_f64(p + 26, v->tx_power_max);
    put_str(p + 34, v->board, 24);
    put_str(p + 58, v->firmware, 24);
    put_str(p + 82, v->serial, 24);
    return LR20XX_LINK_HELLO_SIZE;
}

bool lr20xx_link_get_hello(const uint8_t *p, size_t n, lr20xx_link_hello_t *v)
{
    if (n < LR20XX_LINK_HELLO_SIZE)
        return false;
    v->version = lr20xx_link_u16(p);
    v->rom_major = p[2];
    v->rom_minor = p[3];
    v->rom_known = p[4];
    v->caps = p[5];
    v->tx_max_samples = lr20xx_link_u32(p + 6);
    v->freq_min = lr20xx_link_f64(p + 10);
    v->freq_max = lr20xx_link_f64(p + 18);
    v->tx_power_max = lr20xx_link_f64(p + 26);
    get_str(p + 34, v->board, 24);
    get_str(p + 58, v->firmware, 24);
    get_str(p + 82, v->serial, 24);
    return true;
}

size_t lr20xx_link_put_rx_config(uint8_t *p, const lr20xx_link_rx_config_t *v)
{
    lr20xx_link_put_f64(p, v->rate);
    p[8] = v->mode;
    p[9] = (uint8_t)v->gain;
    lr20xx_link_put_f64(p + 10, v->center_hz);
    lr20xx_link_put_f64(p + 18, v->ppm);
    return LR20XX_LINK_RX_CONFIG_SIZE;
}

bool lr20xx_link_get_rx_config(const uint8_t *p, size_t n, lr20xx_link_rx_config_t *v)
{
    if (n < LR20XX_LINK_RX_CONFIG_SIZE)
        return false;
    v->rate = lr20xx_link_f64(p);
    v->mode = p[8];
    v->gain = (int8_t)p[9];
    v->center_hz = lr20xx_link_f64(p + 10);
    v->ppm = lr20xx_link_f64(p + 18);
    return true;
}

size_t lr20xx_link_put_plan(uint8_t *p, const lr20xx_link_plan_t *v)
{
    p[0] = v->mode;
    p[1] = v->format;
    lr20xx_link_put_u32(p + 2, v->rate_num);
    lr20xx_link_put_u32(p + 6, v->rate_den);
    lr20xx_link_put_f64(p + 10, v->passband_hz);
    return LR20XX_LINK_PLAN_SIZE;
}

bool lr20xx_link_get_plan(const uint8_t *p, size_t n, lr20xx_link_plan_t *v)
{
    if (n < LR20XX_LINK_PLAN_SIZE)
        return false;
    v->mode = p[0];
    v->format = p[1];
    v->rate_num = lr20xx_link_u32(p + 2);
    v->rate_den = lr20xx_link_u32(p + 6);
    v->passband_hz = lr20xx_link_f64(p + 10);
    return true;
}

size_t lr20xx_link_put_rx_hdr(uint8_t *p, const lr20xx_link_rx_hdr_t *v)
{
    lr20xx_link_put_u64(p, v->first);
    lr20xx_link_put_u64(p + 8, (uint64_t)v->t_us);
    p[16] = v->format;
    p[17] = v->flags;
    p[18] = v->bits;
    return LR20XX_LINK_RX_HDR;
}

bool lr20xx_link_get_rx_hdr(const uint8_t *p, size_t n, lr20xx_link_rx_hdr_t *v)
{
    if (n < LR20XX_LINK_RX_HDR)
        return false;
    v->first = lr20xx_link_u64(p);
    v->t_us = (int64_t)lr20xx_link_u64(p + 8);
    v->format = p[16];
    v->flags = p[17];
    v->bits = p[18];
    return v->bits >= LR20XX_LINK_BITS_MIN && v->bits <= (v->format ? 8 : 16);
}

size_t lr20xx_link_put_tx_begin(uint8_t *p, const lr20xx_link_tx_begin_t *v)
{
    p[0] = v->mode;
    lr20xx_link_put_f64(p + 1, v->center_hz);
    lr20xx_link_put_f64(p + 9, v->ppm);
    lr20xx_link_put_u32(p + 17, v->rate);
    lr20xx_link_put_u32(p + 21, v->fdev_hz);
    lr20xx_link_put_f32(p + 25, v->power_dbm);
    lr20xx_link_put_u64(p + 29, v->samples);
    lr20xx_link_put_u64(p + 37, (uint64_t)v->at_us);
    return LR20XX_LINK_TX_BEGIN_SIZE;
}

bool lr20xx_link_get_tx_begin(const uint8_t *p, size_t n, lr20xx_link_tx_begin_t *v)
{
    if (n < 37)
        return false;
    v->mode = p[0];
    v->center_hz = lr20xx_link_f64(p + 1);
    v->ppm = lr20xx_link_f64(p + 9);
    v->rate = lr20xx_link_u32(p + 17);
    v->fdev_hz = lr20xx_link_u32(p + 21);
    v->power_dbm = lr20xx_link_f32(p + 25);
    v->samples = lr20xx_link_u64(p + 29);
    v->at_us = n >= LR20XX_LINK_TX_BEGIN_SIZE ? (int64_t)lr20xx_link_u64(p + 37) : 0;
    return true;
}

size_t lr20xx_link_put_tx_result(uint8_t *p, const lr20xx_link_tx_result_t *v)
{
    lr20xx_link_put_u32(p, (uint32_t)v->runs);
    lr20xx_link_put_u32(p + 4, v->sent);
    p[8] = v->tx_flags;
    p[9] = v->early;
    p[10] = v->timed_out;
    lr20xx_link_put_u64(p + 11, (uint64_t)v->key_us);
    p[19] = v->late;
    return LR20XX_LINK_TX_RESULT_SIZE;
}

bool lr20xx_link_get_tx_result(const uint8_t *p, size_t n, lr20xx_link_tx_result_t *v)
{
    if (n < 11)
        return false;
    v->runs = (int32_t)lr20xx_link_u32(p);
    v->sent = lr20xx_link_u32(p + 4);
    v->tx_flags = p[8];
    v->early = p[9];
    v->timed_out = p[10];
    v->key_us = n >= LR20XX_LINK_TX_RESULT_SIZE ? (int64_t)lr20xx_link_u64(p + 11) : 0;
    v->late = n >= LR20XX_LINK_TX_RESULT_SIZE ? p[19] : 0;
    return true;
}

size_t lr20xx_link_put_stats(uint8_t *p, const lr20xx_link_stats_t *v)
{
    lr20xx_link_put_u64(p, v->samples);
    const uint32_t w[7] = {v->losses, v->restarts, v->errors, v->reads, v->irq_late, v->link_drops, v->ring_peak};
    for (int i = 0; i < 7; i++)
        lr20xx_link_put_u32(p + 8 + 4 * i, w[i]);
    return LR20XX_LINK_STATS_SIZE;
}

bool lr20xx_link_get_stats(const uint8_t *p, size_t n, lr20xx_link_stats_t *v)
{
    if (n < LR20XX_LINK_STATS_SIZE)
        return false;
    v->samples = lr20xx_link_u64(p);
    uint32_t *w[7] = {&v->losses, &v->restarts, &v->errors, &v->reads, &v->irq_late, &v->link_drops, &v->ring_peak};
    for (int i = 0; i < 7; i++)
        *w[i] = lr20xx_link_u32(p + 8 + 4 * i);
    return true;
}

void lr20xx_link_parser_init(lr20xx_link_parser_t *p)
{
    p->have = 0;
    p->drop = 0;
    p->skipped = 0;
}

/* Drop the frame given out last. */
static void compact(lr20xx_link_parser_t *p)
{
    if (!p->drop)
        return;
    memmove(p->buf, p->buf + p->drop, p->have - p->drop);
    p->have -= p->drop;
    p->drop = 0;
}

static bool known_type(uint8_t t)
{
    return t == LR20XX_LINK_REQ || t == LR20XX_LINK_RSP || t == LR20XX_LINK_RX_DATA || t == LR20XX_LINK_TX_DATA ||
           t == LR20XX_LINK_EVENT;
}

/* Drop bytes from the front until the buffer starts with what may be a frame. */
static void resync(lr20xx_link_parser_t *p)
{
    size_t i = 0;
    while (i < p->have) {
        if (p->buf[i] == LR20XX_LINK_MAGIC) {
            if (p->have - i < LR20XX_LINK_HEADER)
                break; /* can't tell yet */
            if (known_type(p->buf[i + 1]) && lr20xx_link_u16(p->buf + i + 2) <= LR20XX_LINK_MAX_PAYLOAD)
                break;
        }
        i++;
    }
    if (i) {
        memmove(p->buf, p->buf + i, p->have - i);
        p->have -= i;
        p->skipped += (uint32_t)i;
    }
}

static size_t frame_len(const lr20xx_link_parser_t *p)
{
    return p->have >= LR20XX_LINK_HEADER ? LR20XX_LINK_HEADER + lr20xx_link_u16(p->buf + 2) : 0;
}

size_t lr20xx_link_feed(lr20xx_link_parser_t *p, const uint8_t *data, size_t n)
{
    size_t took = 0;
    compact(p);
    while (took < n) {
        resync(p);
        size_t want = frame_len(p);
        if (want && p->have >= want)
            break; /* a whole frame waits to be taken */
        size_t room = want ? want - p->have : LR20XX_LINK_HEADER - p->have;
        if (room > n - took)
            room = n - took;
        memcpy(p->buf + p->have, data + took, room);
        p->have += room;
        took += room;
    }
    resync(p);
    return took;
}

bool lr20xx_link_next(lr20xx_link_parser_t *p, lr20xx_link_frame_t *f)
{
    compact(p);
    resync(p);
    size_t want = frame_len(p);
    if (!want || p->have < want)
        return false;
    /* The payload stays at the front of buf until the next call, which drops it first. */
    f->type = p->buf[1];
    f->len = (uint16_t)(want - LR20XX_LINK_HEADER);
    f->payload = p->buf + LR20XX_LINK_HEADER;
    p->drop = want;
    return true;
}

bool lr20xx_link_parse_req(const lr20xx_link_frame_t *f, uint16_t *seq, uint16_t *cmd, const uint8_t **args,
                           size_t *n)
{
    if (f->type != LR20XX_LINK_REQ || f->len < 4)
        return false;
    *seq = lr20xx_link_u16(f->payload);
    *cmd = lr20xx_link_u16(f->payload + 2);
    *args = f->payload + 4;
    *n = f->len - 4u;
    return true;
}

bool lr20xx_link_parse_rsp(const lr20xx_link_frame_t *f, uint16_t *seq, int32_t *err, const uint8_t **ans, size_t *n)
{
    if (f->type != LR20XX_LINK_RSP || f->len < 6)
        return false;
    *seq = lr20xx_link_u16(f->payload);
    *err = (int32_t)lr20xx_link_u32(f->payload + 2);
    *ans = f->payload + 6;
    *n = f->len - 6u;
    return true;
}

/* ── Samples on the wire ── */

static int32_t value(const uint8_t *samples, size_t i, unsigned format)
{
    return format ? (int8_t)samples[i] : (int16_t)(samples[2 * i] | samples[2 * i + 1] << 8);
}

unsigned lr20xx_link_bits(const uint8_t *samples, size_t values, unsigned format)
{
    uint32_t m = 0; /* every value's magnitude bits: v for v >= 0, ~v (= -v - 1) below */
    for (size_t i = 0; i < values; i++) {
        const int32_t v = value(samples, i, format);
        m |= (uint32_t)(v < 0 ? ~v : v);
    }
    unsigned bits = 1; /* a b-bit value holds m < 2^(b-1) */
    while (m >> (bits - 1))
        bits++;
    return bits < LR20XX_LINK_BITS_MIN ? LR20XX_LINK_BITS_MIN : bits;
}

size_t lr20xx_link_pack(uint8_t *out, const uint8_t *samples, size_t values, unsigned format, unsigned bits)
{
    if (bits == (format ? 8u : 16u)) { /* the chip's own format */
        const size_t n = values * (format ? 1 : 2);
        if (out != samples)
            memmove(out, samples, n);
        return n;
    }
    /* In place is safe: after i values, at most i * bits / 8 bytes are out, behind the next value's bytes. */
    const uint32_t mask = (1u << bits) - 1;
    uint32_t acc = 0;
    unsigned have = 0;
    size_t o = 0;
    for (size_t i = 0; i < values; i++) {
        acc |= ((uint32_t)value(samples, i, format) & mask) << have;
        have += bits;
        while (have >= 8) {
            out[o++] = (uint8_t)acc;
            acc >>= 8;
            have -= 8;
        }
    }
    if (have)
        out[o++] = (uint8_t)acc;
    return o;
}

size_t lr20xx_link_packed_values(size_t n, unsigned bits)
{
    if (bits < LR20XX_LINK_BITS_MIN || bits > 16)
        return 0;
    return (n * 8 / bits) & ~(size_t)1;
}

size_t lr20xx_link_unpack(int16_t *out, size_t max, const uint8_t *in, size_t n, unsigned bits)
{
    size_t values = lr20xx_link_packed_values(n, bits);
    if (values > max)
        values = max & ~(size_t)1;
    const uint32_t mask = (1u << bits) - 1, sign = 1u << (bits - 1);
    uint32_t acc = 0;
    unsigned have = 0;
    size_t j = 0;
    for (size_t i = 0; i < values; i++) {
        while (have < bits) {
            acc |= (uint32_t)in[j++] << have;
            have += 8;
        }
        const uint32_t v = acc & mask;
        out[i] = (int16_t)(v & sign ? (int32_t)v - (int32_t)(mask + 1) : (int32_t)v);
        acc >>= bits;
        have -= bits;
    }
    return values;
}
