// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * The chip as an SDR: raw I/Q receive sessions (docs/iq.md section 2) and sample transmit (section 3) on an open
 * lr20xx_t. Portable and single-threaded: the caller runs lr20xx_iq_poll in a loop on a thread of its own, at a
 * priority that keeps the 1 KB FIFO read (2 ms at 125 kS/s int16, R2.18), and serializes every other call with it.
 *
 * From RadioFoundry Gateway's esp_idf/main/fe_lr2021.c (its session, reader and transmit run), without the
 * gateway's ring, resampler and scheduling: samples go to a sink as they come, at the chip's own rate.
 *
 * Receive modes (lr20xx_iq_plan picks one for a rate):
 *
 * | Mode  | Format | Rates                                   | Passband               | Found       |
 * |-------|--------|-----------------------------------------|------------------------|-------------|
 * | LORA  | cs16   | 125 000, 250 000 (4 x the LoRa bandwidth)| about the bandwidth    | R2.1-R2.10  |
 * | FSK16 | cs16   | an FSK rx_bw code's, 32 MHz / (m x d)    | 0.27-0.38 of the rate  | R2.19-R2.20 |
 * | CONV8 | cs8    | any, exact, through the rate converter  | the code's filter      | R2.21, R2.26|
 *
 * cs16 is I then Q, little-endian int16; cs8 int8. cs8 clips at -32..+31 (6 bits, R2.26): a strong signal anywhere
 * in the band clips the whole stream. FSK16 has at least 50 dB more room.
 */
#ifndef LR20XX_IQ_H
#define LR20XX_IQ_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lr20xx.h"
#include "lr20xx_board.h"
#include "lr20xx_chips.h"
#include "lr20xx_tx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LR20XX_IQ_AUTO,  /* FSK16 up to LR20XX_IQ_FSK16_MAX, then CONV8 */
    LR20XX_IQ_LORA,
    LR20XX_IQ_FSK16,
    LR20XX_IQ_CONV8,
} lr20xx_iq_mode_t;

typedef enum { LR20XX_IQ_CS16, LR20XX_IQ_CS8 } lr20xx_iq_format_t;

/* The fastest rates each mode keeps up at over 16 MHz SPI (R2.29): int16 at 4 bytes a sample, int8 at 2. */
#define LR20XX_IQ_FSK16_MAX 260000.0
#define LR20XX_IQ_CONV8_MAX 576000.0
#define LR20XX_IQ_FIFO_BYTES 1024

typedef struct {
    lr20xx_iq_mode_t mode;
    lr20xx_iq_format_t format;
    int sample_bytes;          /* on the FIFO: 4 (cs16) or 2 (cs8) */
    uint32_t rate_num, rate_den; /* the samples' rate, num / den S/s */
    double rate;               /* the same as a double */
    uint8_t lora_bw;           /* LORA: the LoRa bandwidth code */
    uint8_t rx_bw;             /* FSK16, CONV8: the FSK rx_bw code */
    double chip_rate;          /* CONV8: the code's rate, ahead of the converter */
    double passband_hz;        /* what the chip's own filter passes, about (two-sided) */
} lr20xx_iq_plan_t;

/* The plan for a rate in a mode: LORA takes the nearer of its two rates; FSK16 the code with the lowest rate at
 * least `rate`; CONV8 `rate` exactly. LR20XX_ERR_NOT_SUPPORTED out of the mode's range. */
lr20xx_err_t lr20xx_iq_plan(double rate, lr20xx_iq_mode_t mode, lr20xx_iq_plan_t *out);
/* The rates a mode has as a list (LORA, FSK16; up to max into rates), lowest first, and their span in *lo, *hi
 * (either may be NULL). CONV8 lists none: any whole rate from *lo to *hi. AUTO lists FSK16's and spans both.
 * Returns how many listed. lr20xx_iq_plan and lr20xx_iq_rates do no I/O (src/lr20xx_iq_plan.c). */
int lr20xx_iq_rates(lr20xx_iq_mode_t mode, double *rates, int max, double *lo, double *hi);

typedef struct {
    double rate;            /* S/s wanted (lr20xx_iq_plan) */
    lr20xx_iq_mode_t mode;
    double center_hz;
    double ppm;             /* the crystal's error, corrected in the frequency sent */
    int gain;               /* SetAgcGainManual's step, 1 (-8 dB) to 13 (53.5 dB; R2.25); 0 the chip's AGC */
} lr20xx_iq_cfg_t;

/* The front end's gain at each manual step 1 to 13, dB (R2.25: the ROM's table at 0xF30174). */
extern const double lr20xx_iq_gain_db[13];

typedef struct {
    uint64_t samples;       /* delivered to the sink */
    uint32_t losses;        /* FIFO overflows seen (each a gap of unknown length) */
    uint32_t restarts;      /* sessions started again after a loss (LORA) */
    uint32_t errors;        /* SPI or command failures while streaming */
    uint32_t reads, irq_late;
} lr20xx_iq_stats_t;

/* Flags with a sink's samples. */
#define LR20XX_IQ_GAP 0x1u   /* samples were lost just ahead of these */

/* Where the samples go: n whole samples (plan.sample_bytes each) at data, t_us the time the read ended
 * (lr20xx_now_us). */
typedef void (*lr20xx_iq_sink_t)(void *ctx, const uint8_t *data, size_t n, unsigned flags, int64_t t_us);

typedef struct {
    lr20xx_t *dev;
    const lr20xx_board_t *board;
    const lr20xx_rom_t *rom;   /* NULL: a ROM not in the table, allowed by lr20xx_iq_init's allow_untested */
    uint8_t rom_major, rom_minor;
    lr20xx_iq_cfg_t cfg;
    lr20xx_iq_plan_t plan;
    bool in_session, have_cfg;
    bool by_level;             /* reading by the FIFO level, not waiting for the IRQ */
    bool gap;                  /* a loss not yet told to the sink */
    uint16_t cal_word;         /* the CalibFE cell the session calibrated (0: none) */
    uint8_t carry_n;           /* bytes of a sample read and not yet delivered (0 or 2) */
    uint32_t since_flags;
    lr20xx_iq_stats_t stats;
    char error[128];
    uint8_t buf[LR20XX_IQ_FIFO_BYTES + 4] __attribute__((aligned(4)));
} lr20xx_iq_t;

/* Take an open chip as an SDR: check its ROM (lr20xx_chips.h; LR20XX_ERR_NOT_SUPPORTED for one not in the table
 * unless allow_untested) and that no patch runs (R1.2), then standby, the clock, the regulator and calibration. */
lr20xx_err_t lr20xx_iq_init(lr20xx_iq_t *iq, lr20xx_t *dev, const lr20xx_board_t *board, bool allow_untested);

/* Start receiving (a session, docs/iq.md 2.1-2.2): ends any session first. */
lr20xx_err_t lr20xx_iq_start(lr20xx_iq_t *iq, const lr20xx_iq_cfg_t *cfg);
/* Change the gain in a session (sent again after each SetRx, R2.25), or for the next one. */
lr20xx_err_t lr20xx_iq_set_gain(lr20xx_iq_t *iq, int gain);
/* One step of the reader: wait for samples (the IRQ pin, else the level) up to about wait_ms, read what's there
 * (1 KB at most), give it to the sink. Returns the samples delivered (0: none yet), or < 0 (an lr20xx_err_t) when
 * the session failed and couldn't be restarted. A loss is handled here (R2.15, R2.28) and flagged to the sink. */
int lr20xx_iq_poll(lr20xx_iq_t *iq, lr20xx_iq_sink_t sink, void *ctx, int wait_ms);
/* End the session: standby, reset, the chip set up again (R2.16). */
void lr20xx_iq_stop(lr20xx_iq_t *iq);

/* ── Transmit ── */

typedef struct {
    lr20xx_tx_mode_t mode;  /* FREQ, PHASE_STEP, IQ_PHASE, IQ_POLAR or CW */
    double center_hz;       /* 0: the receive center */
    double ppm;             /* used when center_hz is given */
    uint32_t rate;          /* S/s (R3.10: 31 250 to 312 500 tried) */
    uint32_t fdev_hz;       /* FREQ: count x fdev / 64 Hz (R3.10) */
    double power_dbm;
    uint64_t samples;       /* the transmission's length in samples, to size the runs (0: unknown, full runs) */
    int max_ms;             /* give up after this (0: the length plus 2 s, or 10 s) */
} lr20xx_iq_tx_t;

/* One transmission, half duplex: a receive session is paused (no reset or new calibration where it already
 * calibrated the frequency, R3.16), the mode set up, the samples pulled and streamed in runs (R3.15), then the
 * session resumed. pull gives FIFO bytes in the mode's format (lr20xx_tx_sample_bytes); past its end the last run
 * is filled with zeros. */
lr20xx_err_t lr20xx_iq_transmit(lr20xx_iq_t *iq, const lr20xx_iq_tx_t *tx, lr20xx_tx_pull_t pull, void *ctx,
                                lr20xx_tx_stream_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif
