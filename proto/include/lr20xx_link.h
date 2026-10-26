// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * The bridge link: how a host drives an LR20xx that sits on a microcontroller's SPI bus (firmware/), over TCP or a
 * USB serial port. The chip's 1 KB FIFO lasts 2 ms, so the microcontroller reads it (lr20xx_iq_poll) and streams
 * the samples; the host only configures and receives. docs/link.md is the protocol's description; this is its
 * encoding, portable C with no I/O.
 *
 * A byte stream of frames, all little-endian:
 *
 *   0  u8   LR20XX_LINK_MAGIC (0xA5)
 *   1  u8   type (lr20xx_link_type_t)
 *   2  u16  payload length (at most LR20XX_LINK_MAX_PAYLOAD)
 *   4  ...  payload
 *
 * The host sends REQ frames (a sequence number and a command) and TX_DATA; the device answers each REQ with one RSP
 * (the same sequence number, an lr20xx_err_t, the command's answer) and streams RX_DATA while receiving. A host
 * starts with LR20XX_CMD_HELLO. A reader that loses sync skips to the next magic byte with a valid header.
 */
#ifndef LR20XX_LINK_H
#define LR20XX_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LR20XX_LINK_MAGIC 0xA5
#define LR20XX_LINK_VERSION 1
#define LR20XX_LINK_HEADER 4
#define LR20XX_LINK_MAX_PAYLOAD 4096
#define LR20XX_LINK_TCP_PORT 7373      /* the device listens here */
/* UDP discovery: a host broadcasts the 7 bytes "LR20XX?" to this port; each device answers the sender with one RSP
 * frame, seq 0, err 0, whose answer is the HELLO payload (lr20xx_link_put_hello). */
#define LR20XX_LINK_DISCOVERY_PORT 7374
#define LR20XX_LINK_DISCOVERY "LR20XX?"

typedef enum {
    LR20XX_LINK_REQ = 0x01,     /* host -> device: u16 seq, u16 cmd, args */
    LR20XX_LINK_RSP = 0x02,     /* device -> host: u16 seq, i32 err, answer */
    LR20XX_LINK_RX_DATA = 0x10, /* device -> host: lr20xx_link_rx_hdr_t, then samples */
    LR20XX_LINK_TX_DATA = 0x11, /* host -> device: samples for the transmission begun (TX_BEGIN), in its format */
    LR20XX_LINK_EVENT = 0x12,   /* device -> host: u16 kind, i32 err, text (lr20xx_link_event_t) */
} lr20xx_link_type_t;

typedef enum {
    LR20XX_CMD_HELLO = 0x0001,     /* -> lr20xx_link_hello_t */
    LR20XX_CMD_RX_CONFIG = 0x0010, /* lr20xx_link_rx_config_t -> lr20xx_link_plan_t; takes effect now if receiving */
    LR20XX_CMD_RX_START = 0x0011,  /* -> nothing; RX_DATA follows */
    LR20XX_CMD_RX_STOP = 0x0012,   /* -> nothing; no RX_DATA after the answer */
    LR20XX_CMD_SET_GAIN = 0x0013,  /* i8 step (0 AGC, 1-13) -> nothing */
    LR20XX_CMD_TX_BEGIN = 0x0020,  /* lr20xx_link_tx_begin_t -> nothing; TX_DATA follows */
    LR20XX_CMD_TX_COMMIT = 0x0021, /* -> lr20xx_link_tx_result_t, once the transmission has gone out */
    LR20XX_CMD_TX_ABORT = 0x0022,  /* -> nothing; drops what TX_DATA sent */
    LR20XX_CMD_READ_REG = 0x0030,  /* u32 addr, u8 n (1-64) -> n x u32; not while receiving */
    LR20XX_CMD_WRITE_REG = 0x0031, /* u32 addr, u32 value -> nothing; not while receiving */
    LR20XX_CMD_RAW = 0x0032,       /* u16 answer bytes, then a command frame -> u8 command status, u8 chip mode (lr20xx_status), the answer; not while
                                    * receiving (a debug path: the chip's state is the host's problem after it) */
    LR20XX_CMD_STATS = 0x0040,     /* -> lr20xx_link_stats_t */
    LR20XX_CMD_TEMP = 0x0041,      /* -> f32 degrees C; not while receiving */
} lr20xx_link_cmd_t;

typedef struct {
    uint16_t version;            /* LR20XX_LINK_VERSION */
    uint8_t rom_major, rom_minor;
    uint8_t rom_known;           /* the ROM is in lr20xx_chips.h's table */
    uint8_t caps;                /* LR20XX_LINK_CAP_* */
    uint32_t tx_max_samples;     /* the longest transmission the device buffers */
    double freq_min, freq_max;   /* the board's */
    double tx_power_max;
    char board[24];
    char firmware[24];
    char serial[24];
} lr20xx_link_hello_t;
#define LR20XX_LINK_CAP_RX 0x01
#define LR20XX_LINK_CAP_TX 0x02
#define LR20XX_LINK_CAP_IRQ 0x04

typedef struct {
    double rate;
    uint8_t mode;     /* lr20xx_iq_mode_t */
    int8_t gain;
    double center_hz;
    double ppm;
} lr20xx_link_rx_config_t;

typedef struct {
    uint8_t mode, format; /* lr20xx_iq_mode_t, lr20xx_iq_format_t */
    uint32_t rate_num, rate_den;
    double passband_hz;
} lr20xx_link_plan_t;

typedef struct {
    uint64_t first;   /* the first sample's index in the stream (samples delivered since RX_START) */
    int64_t t_us;     /* the device's clock when it was read */
    uint8_t format;   /* lr20xx_iq_format_t: what the values are (int16 or int8) */
    uint8_t flags;    /* LR20XX_LINK_GAP: samples were lost ahead of these (by the chip, or the link's buffer) */
    uint8_t bits;     /* each value's width on the wire, LR20XX_LINK_BITS_MIN to 16 (lr20xx_link_pack) */
} lr20xx_link_rx_hdr_t;
#define LR20XX_LINK_RX_HDR 19
#define LR20XX_LINK_GAP 0x01 /* the same bit as lr20xx_iq.h's LR20XX_IQ_GAP */

typedef struct {
    uint8_t mode;     /* lr20xx_tx_mode_t */
    double center_hz; /* 0: the receive center */
    double ppm;
    uint32_t rate;
    uint32_t fdev_hz;
    float power_dbm;
    uint64_t samples; /* the length; TX_DATA must bring exactly this many */
} lr20xx_link_tx_begin_t;

typedef struct {
    int32_t runs;
    uint32_t sent;    /* bytes into the FIFO */
    uint8_t tx_flags; /* the TX FIFO's flags at the end (0x20: it ran dry, the waveform broke) */
    uint8_t early, timed_out;
} lr20xx_link_tx_result_t;

typedef struct {
    uint64_t samples;
    uint32_t losses, restarts, errors, reads, irq_late;
    uint32_t link_drops;  /* RX_DATA frames dropped because the host didn't keep up */
    uint32_t ring_peak;   /* the device's sample buffer at its fullest, bytes */
} lr20xx_link_stats_t;

typedef enum {
    LR20XX_EVENT_RX_FAILED = 1, /* the session failed and stopped; err and text say why */
    LR20XX_EVENT_LOG = 2,       /* a line of the device's log */
} lr20xx_link_event_kind_t;

/* ── Encoding: each writes a whole frame into buf (room for LR20XX_LINK_HEADER + payload) and returns its length,
 * or 0 if it doesn't fit in cap. ── */

size_t lr20xx_link_req(uint8_t *buf, size_t cap, uint16_t seq, uint16_t cmd, const void *args, size_t n);
size_t lr20xx_link_rsp(uint8_t *buf, size_t cap, uint16_t seq, int32_t err, const void *ans, size_t n);
size_t lr20xx_link_event(uint8_t *buf, size_t cap, uint16_t kind, int32_t err, const char *text);
/* RX_DATA's header and n bytes of samples. */
size_t lr20xx_link_rx_data(uint8_t *buf, size_t cap, const lr20xx_link_rx_hdr_t *h, const uint8_t *samples, size_t n);
size_t lr20xx_link_tx_data(uint8_t *buf, size_t cap, const uint8_t *samples, size_t n);

/* The structs' payloads, packed little-endian: put returns the bytes written into p (room for _SIZE), get
 * returns false if n is short. */
#define LR20XX_LINK_HELLO_SIZE 106
#define LR20XX_LINK_RX_CONFIG_SIZE 26
#define LR20XX_LINK_PLAN_SIZE 18
#define LR20XX_LINK_TX_BEGIN_SIZE 37
#define LR20XX_LINK_TX_RESULT_SIZE 11
#define LR20XX_LINK_STATS_SIZE 36
size_t lr20xx_link_put_hello(uint8_t *p, const lr20xx_link_hello_t *v);
bool lr20xx_link_get_hello(const uint8_t *p, size_t n, lr20xx_link_hello_t *v);
size_t lr20xx_link_put_rx_config(uint8_t *p, const lr20xx_link_rx_config_t *v);
bool lr20xx_link_get_rx_config(const uint8_t *p, size_t n, lr20xx_link_rx_config_t *v);
size_t lr20xx_link_put_plan(uint8_t *p, const lr20xx_link_plan_t *v);
bool lr20xx_link_get_plan(const uint8_t *p, size_t n, lr20xx_link_plan_t *v);
size_t lr20xx_link_put_rx_hdr(uint8_t *p, const lr20xx_link_rx_hdr_t *v);
bool lr20xx_link_get_rx_hdr(const uint8_t *p, size_t n, lr20xx_link_rx_hdr_t *v);
size_t lr20xx_link_put_tx_begin(uint8_t *p, const lr20xx_link_tx_begin_t *v);
bool lr20xx_link_get_tx_begin(const uint8_t *p, size_t n, lr20xx_link_tx_begin_t *v);
size_t lr20xx_link_put_tx_result(uint8_t *p, const lr20xx_link_tx_result_t *v);
bool lr20xx_link_get_tx_result(const uint8_t *p, size_t n, lr20xx_link_tx_result_t *v);
size_t lr20xx_link_put_stats(uint8_t *p, const lr20xx_link_stats_t *v);
bool lr20xx_link_get_stats(const uint8_t *p, size_t n, lr20xx_link_stats_t *v);

/* ── Samples on the wire (RX_DATA) ──
 * Each value (I, then Q) is a `bits`-bit two's complement integer, packed LSB first into a little-endian bit stream;
 * the last byte's spare bits are 0. The device picks the narrowest width that holds a frame's values, so nothing is
 * lost: the noise at a moderate gain needs about 10 of int16's bits, and int8 never more than 6 (docs/iq.md R2.33).
 * At full width it's the chip's own format, byte for byte (16: int16 little-endian, 8: int8). At least 4 bits, so the
 * padding is under one I/Q pair and the length gives the count. `format` is 0 for int16, 1 for int8
 * (lr20xx_iq_format_t). */
#define LR20XX_LINK_BITS_MIN 4

/* The narrowest width (LR20XX_LINK_BITS_MIN or more) that holds the values in samples (the chip's format). */
unsigned lr20xx_link_bits(const uint8_t *samples, size_t values, unsigned format);
/* Packs the values at bits each into out (which may be samples: it packs in place); returns the bytes written. */
size_t lr20xx_link_pack(uint8_t *out, const uint8_t *samples, size_t values, unsigned format, unsigned bits);
/* How many values n bytes packed at bits hold (an even count: whole I/Q pairs). */
size_t lr20xx_link_packed_values(size_t n, unsigned bits);
/* Unpacks up to max values (rounded down to whole pairs) from n bytes packed at bits; returns how many. */
size_t lr20xx_link_unpack(int16_t *out, size_t max, const uint8_t *in, size_t n, unsigned bits);

/* Little-endian field helpers, for the commands with plain arguments. */
void lr20xx_link_put_u16(uint8_t *p, uint16_t v);
void lr20xx_link_put_u32(uint8_t *p, uint32_t v);
void lr20xx_link_put_u64(uint8_t *p, uint64_t v);
void lr20xx_link_put_f32(uint8_t *p, float v);
void lr20xx_link_put_f64(uint8_t *p, double v);
uint16_t lr20xx_link_u16(const uint8_t *p);
uint32_t lr20xx_link_u32(const uint8_t *p);
uint64_t lr20xx_link_u64(const uint8_t *p);
float lr20xx_link_f32(const uint8_t *p);
double lr20xx_link_f64(const uint8_t *p);

/* ── Decoding a byte stream ── */

typedef struct {
    uint8_t type;
    uint16_t len;
    const uint8_t *payload;
} lr20xx_link_frame_t;

/* A frame parser with its own buffer: feed it bytes as they come, take frames out. */
typedef struct {
    uint8_t buf[LR20XX_LINK_HEADER + LR20XX_LINK_MAX_PAYLOAD];
    size_t have;
    size_t drop;      /* the frame lr20xx_link_next gave out, dropped on the next call */
    uint32_t skipped; /* bytes dropped to find a frame (lost sync) */
} lr20xx_link_parser_t;

void lr20xx_link_parser_init(lr20xx_link_parser_t *p);
/* Take up to n bytes of data; returns how many it took (it takes no more than completes one frame: call
 * lr20xx_link_next, then feed the rest). */
size_t lr20xx_link_feed(lr20xx_link_parser_t *p, const uint8_t *data, size_t n);
/* A complete frame, if there is one: true and *f (its payload points into the parser, valid until the next feed).
 * The frame is consumed. */
bool lr20xx_link_next(lr20xx_link_parser_t *p, lr20xx_link_frame_t *f);

/* A REQ's or RSP's fixed part. */
bool lr20xx_link_parse_req(const lr20xx_link_frame_t *f, uint16_t *seq, uint16_t *cmd, const uint8_t **args,
                           size_t *n);
bool lr20xx_link_parse_rsp(const lr20xx_link_frame_t *f, uint16_t *seq, int32_t *err, const uint8_t **ans,
                           size_t *n);

#ifdef __cplusplus
}
#endif

#endif
