// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
// Host tests of the bridge link's encoding (lr20xx_link.h): frames byte for byte, every struct both ways, and the
// parser across split reads, junk and lost sync.
#include <stdio.h>
#include <string.h>

#include "lr20xx_link.h"

static int checks, fails;

#define CHECK(cond, ...)                                \
    do {                                                \
        checks++;                                       \
        if (!(cond)) {                                  \
            fails++;                                    \
            printf("FAIL %s:%d: ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                        \
            printf("\n");                               \
        }                                               \
    } while (0)

static uint8_t buf[LR20XX_LINK_HEADER + LR20XX_LINK_MAX_PAYLOAD];

static void frames(void)
{
    const uint8_t args[2] = {0x34, 0x12};
    size_t n = lr20xx_link_req(buf, sizeof buf, 0x0102, LR20XX_CMD_SET_GAIN, args, 2);
    const uint8_t want[] = {0xA5, 0x01, 0x06, 0x00, 0x02, 0x01, 0x13, 0x00, 0x34, 0x12};
    CHECK(n == sizeof want && !memcmp(buf, want, n), "REQ bytes");
    n = lr20xx_link_rsp(buf, sizeof buf, 7, -1, "ab", 2);
    const uint8_t want2[] = {0xA5, 0x02, 0x08, 0x00, 0x07, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 'a', 'b'};
    CHECK(n == sizeof want2 && !memcmp(buf, want2, n), "RSP bytes");
    CHECK(!lr20xx_link_req(buf, 9, 1, 1, args, 2), "too small a buffer");
    static uint8_t big[LR20XX_LINK_MAX_PAYLOAD];
    CHECK(!lr20xx_link_tx_data(buf, sizeof buf + 10, big, LR20XX_LINK_MAX_PAYLOAD + 1), "past the most payload");
    CHECK(lr20xx_link_tx_data(buf, sizeof buf, big, LR20XX_LINK_MAX_PAYLOAD) == sizeof buf, "the most payload");
    lr20xx_link_rx_hdr_t h = {.first = 0x0102030405060708ull, .t_us = -2, .format = 1, .flags = 1, .bits = 6};
    const uint8_t s[4] = {1, 2, 3, 4};
    n = lr20xx_link_rx_data(buf, sizeof buf, &h, s, 4);
    CHECK(n == 4 + 19 + 4 && buf[1] == LR20XX_LINK_RX_DATA && buf[4] == 0x08 && buf[11] == 0x01 && buf[12] == 0xFE &&
              buf[19] == 0xFF && buf[20] == 1 && buf[21] == 1 && buf[22] == 6 && !memcmp(buf + 23, s, 4),
          "RX_DATA bytes");
    lr20xx_link_rx_hdr_t h2;
    CHECK(lr20xx_link_get_rx_hdr(buf + 4, 19, &h2) && h2.bits == 6, "RX_DATA header back");
    buf[22] = 9;
    CHECK(!lr20xx_link_get_rx_hdr(buf + 4, 19, &h2), "int8 wider than 8 bits refused");
    buf[22] = 3;
    CHECK(!lr20xx_link_get_rx_hdr(buf + 4, 19, &h2), "under 4 bits refused");
    n = lr20xx_link_event(buf, sizeof buf, LR20XX_EVENT_LOG, 0, "hi");
    CHECK(n == 12 && buf[1] == LR20XX_LINK_EVENT && buf[10] == 'h', "EVENT");
}

static void structs(void)
{
    uint8_t p[128];
    lr20xx_link_hello_t a = {LR20XX_LINK_VERSION, 1, 0x18, 1, 7, 1000000, 150e6, 960e6, 22, "w12", "0.1.0", "ABCDEF"}, b;
    CHECK(lr20xx_link_put_hello(p, &a) == LR20XX_LINK_HELLO_SIZE && lr20xx_link_get_hello(p, LR20XX_LINK_HELLO_SIZE, &b),
          "hello");
    CHECK(b.version == 1 && b.rom_minor == 0x18 && b.caps == 7 && b.tx_max_samples == 1000000 && b.freq_max == 960e6 &&
              !strcmp(b.board, "w12") && !strcmp(b.serial, "ABCDEF"),
          "hello back");
    CHECK(!lr20xx_link_get_hello(p, LR20XX_LINK_HELLO_SIZE - 1, &b), "short");
    lr20xx_link_rx_config_t c = {384000, 3, -1, 915e6, -28.5}, c2;
    CHECK(lr20xx_link_put_rx_config(p, &c) == LR20XX_LINK_RX_CONFIG_SIZE &&
              lr20xx_link_get_rx_config(p, LR20XX_LINK_RX_CONFIG_SIZE, &c2) && c2.rate == 384000 && c2.mode == 3 &&
              c2.gain == -1 && c2.center_hz == 915e6 && c2.ppm == -28.5,
          "rx config");
    lr20xx_link_plan_t pl = {2, 0, 2000000, 8, 92500}, pl2;
    CHECK(lr20xx_link_put_plan(p, &pl) == LR20XX_LINK_PLAN_SIZE && lr20xx_link_get_plan(p, LR20XX_LINK_PLAN_SIZE, &pl2) &&
              pl2.rate_num == 2000000 && pl2.rate_den == 8 && pl2.passband_hz == 92500,
          "plan");
    lr20xx_link_tx_begin_t t = {4, 0, 0, 48000, 2048, -9.5f, 123456789012ull}, t2;
    CHECK(lr20xx_link_put_tx_begin(p, &t) == LR20XX_LINK_TX_BEGIN_SIZE &&
              lr20xx_link_get_tx_begin(p, LR20XX_LINK_TX_BEGIN_SIZE, &t2) && t2.mode == 4 && t2.rate == 48000 &&
              t2.power_dbm == -9.5f && t2.samples == 123456789012ull,
          "tx begin");
    lr20xx_link_tx_result_t r = {3, 99999, 0x20, 1, 0}, r2;
    CHECK(lr20xx_link_put_tx_result(p, &r) == LR20XX_LINK_TX_RESULT_SIZE &&
              lr20xx_link_get_tx_result(p, LR20XX_LINK_TX_RESULT_SIZE, &r2) && r2.runs == 3 && r2.sent == 99999 &&
              r2.tx_flags == 0x20 && r2.early == 1,
          "tx result");
    lr20xx_link_stats_t st = {1ull << 40, 1, 2, 3, 4, 5, 6, 7}, st2;
    CHECK(lr20xx_link_put_stats(p, &st) == LR20XX_LINK_STATS_SIZE &&
              lr20xx_link_get_stats(p, LR20XX_LINK_STATS_SIZE, &st2) && st2.samples == 1ull << 40 && st2.losses == 1 &&
              st2.ring_peak == 7,
          "stats");
}

static void parser(void)
{
    lr20xx_link_parser_t ps;
    lr20xx_link_parser_init(&ps);
    uint8_t stream[200];
    size_t n = 0;
    stream[n++] = 0x00; /* junk */
    stream[n++] = 0xA5; /* a magic with a bad type */
    stream[n++] = 0x77;
    const uint8_t a[3] = {9, 8, 7};
    n += lr20xx_link_req(stream + n, sizeof stream - n, 1, LR20XX_CMD_HELLO, NULL, 0);
    n += lr20xx_link_req(stream + n, sizeof stream - n, 2, LR20XX_CMD_SET_GAIN, a, 3);
    /* a frame whose payload holds what looks like a header */
    const uint8_t fake[6] = {0xA5, 0x01, 0x00, 0x00, 0xA5, 0x02};
    n += lr20xx_link_tx_data(stream + n, sizeof stream - n, fake, 6);
    /* byte by byte */
    int got = 0;
    uint16_t seqs[4], cmds[4];
    size_t lens[4];
    for (size_t i = 0; i < n; i++) {
        size_t took = lr20xx_link_feed(&ps, stream + i, 1);
        lr20xx_link_frame_t f;
        while (lr20xx_link_next(&ps, &f)) {
            const uint8_t *args;
            size_t an;
            if (got < 4) {
                lens[got] = f.len;
                if (lr20xx_link_parse_req(&f, &seqs[got], &cmds[got], &args, &an) && f.len == 7)
                    CHECK(!memcmp(args, a, 3) && an == 3, "args");
            }
            got++;
        }
        if (!took)
            i--; /* a frame was waiting: feed the byte again */
    }
    CHECK(got == 3 && seqs[0] == 1 && cmds[0] == LR20XX_CMD_HELLO && seqs[1] == 2 && cmds[1] == LR20XX_CMD_SET_GAIN &&
              lens[2] == 6,
          "three frames byte by byte (%d)", got);
    CHECK(ps.skipped == 3, "three junk bytes skipped (%u)", ps.skipped);
    /* all at once */
    lr20xx_link_parser_init(&ps);
    size_t off = 0;
    got = 0;
    while (off < n) {
        off += lr20xx_link_feed(&ps, stream + off, n - off);
        lr20xx_link_frame_t f;
        while (lr20xx_link_next(&ps, &f))
            got++;
    }
    lr20xx_link_frame_t f;
    while (lr20xx_link_next(&ps, &f))
        got++;
    CHECK(got == 3, "three frames at once (%d)", got);
}

static void packing(void)
{
    /* the bit order, by hand: int8 values 1, -1, -32, 31 at 6 bits: 000001 111111 100000 011111, LSB first */
    const uint8_t v8[4] = {1, 0xFF, 0xE0, 31};
    uint8_t out[64];
    CHECK(lr20xx_link_bits(v8, 4, 1) == 6, "int8 -32..31: 6 bits");
    size_t n = lr20xx_link_pack(out, v8, 4, 1, 6);
    CHECK(n == 3 && out[0] == 0xC1 && out[1] == 0x0F && out[2] == 0x7E, "6-bit bytes (%02X %02X %02X)", out[0], out[1],
          out[2]);
    int16_t back[64];
    CHECK(lr20xx_link_unpack(back, 64, out, n, 6) == 4 && back[0] == 1 && back[1] == -1 && back[2] == -32 && back[3] == 31,
          "6 bits back");
    CHECK(lr20xx_link_pack(out, v8, 4, 1, 8) == 4 && !memcmp(out, v8, 4), "int8 at 8 bits: the chip's bytes");

    /* widths: the narrowest that holds every value, never under 4 */
    const uint8_t zero[4] = {0};
    CHECK(lr20xx_link_bits(zero, 2, 0) == 4, "silence: 4 bits");
    const uint8_t w[] = {0xFF, 0x00, 0x00, 0x01, 0x00, 0xFF}; /* int16 255, 256, -256 */
    CHECK(lr20xx_link_bits(w, 3, 0) == 10 && lr20xx_link_bits(w, 2, 0) == 10 && lr20xx_link_bits(w + 4, 1, 0) == 9,
          "255 / 256: 10 bits, -256: 9");
    const uint8_t ext[] = {0xFF, 0x7F, 0x00, 0x80}; /* 32767, -32768 */
    CHECK(lr20xx_link_bits(ext, 2, 0) == 16, "the extremes: 16 bits");

    /* every width, int16, against a pseudo-random signal at that width; in place too */
    uint32_t seed = 1;
    for (unsigned bits = 4; bits <= 16; bits++) {
        enum { N = 202 };
        uint8_t raw[2 * N], packed[2 * N];
        int16_t want[N], got[N];
        const int32_t lo = -(1 << (bits - 1)), span = 1 << bits;
        for (int i = 0; i < N; i++) {
            seed = seed * 1103515245u + 12345u;
            want[i] = (int16_t)(lo + (int32_t)((seed >> 8) % (uint32_t)span));
            raw[2 * i] = (uint8_t)want[i];
            raw[2 * i + 1] = (uint8_t)((uint16_t)want[i] >> 8);
        }
        want[0] = (int16_t)lo; /* both ends present */
        want[1] = (int16_t)(lo + span - 1);
        for (int i = 0; i < 2; i++) {
            raw[2 * i] = (uint8_t)want[i];
            raw[2 * i + 1] = (uint8_t)((uint16_t)want[i] >> 8);
        }
        const unsigned b = lr20xx_link_bits(raw, N, 0);
        CHECK(b == (bits < 4 ? 4 : bits), "width %u found (%u)", bits, b);
        n = lr20xx_link_pack(packed, raw, N, 0, bits);
        CHECK(n == (N * bits + 7) / 8, "width %u: %zu bytes", bits, n);
        CHECK(lr20xx_link_unpack(got, N, packed, n, bits) == N && !memcmp(got, want, sizeof want), "width %u round trip",
              bits);
        CHECK(lr20xx_link_pack(raw, raw, N, 0, bits) == n && !memcmp(raw, packed, n), "width %u in place", bits);
    }

    /* the count from the length: padding never reads as a value */
    CHECK(lr20xx_link_packed_values(3, 6) == 4 && lr20xx_link_packed_values(4, 6) == 4 &&
              lr20xx_link_packed_values(5, 4) == 10 && lr20xx_link_packed_values(2, 10) == 0,
          "counts from lengths");
    CHECK(lr20xx_link_unpack(back, 3, out, 3, 6) == 2, "max rounds down to whole pairs");
}

int main(void)
{
    frames();
    packing();
    structs();
    parser();
    printf("lr20xx link: %d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
