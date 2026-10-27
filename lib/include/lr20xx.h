// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * A Semtech LR20xx (LR2021, LR2022, LR2012) on an SPI bus: reset, the BUSY handshake, commands and responses, the
 * FIFOs. Written from the datasheet (DS.LR20xx Rev. 2.2: 3.7.2 SPI, 4.2 reset, 5.2 BUSY, 5.4 command frames); the
 * encodings are in lr20xx_cmd.h. The pins come through an lr20xx_hal_t (lr20xx_hal.h), so the same driver runs on
 * Linux's spidev and on an ESP32.
 *
 * From RadioFoundry Gateway's components/lr2021/include/lr2021.h (tools/upstream/files.txt): the same calls, with
 * the ESP-IDF bus setup moved into lib/hal/esp_idf.c and the patch RAM calls left out.
 */
#ifndef LR20XX_H
#define LR20XX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lr20xx_cmd.h"
#include "lr20xx_hal.h"
#include "lr20xx_port.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    lr20xx_hal_t hal;
    void *mem; /* the handle's memory, lr20xx_handle_size() bytes aligned to 4 (on an ESP32: internal, DMA-capable),
                * the caller's to keep until lr20xx_close; NULL: lr20xx_open allocates it */
} lr20xx_config_t;

typedef struct lr20xx lr20xx_t;

/* Bytes a handle takes (its 1 KB transmit and receive frames among them), for lr20xx_config_t.mem. */
size_t lr20xx_handle_size(void);

/* Take the HAL, reset the chip and wait for it. *out is the handle. On a failure the HAL is closed. */
lr20xx_err_t lr20xx_open(const lr20xx_config_t *cfg, lr20xx_t **out);
/* Hold the chip in reset and close the HAL. */
void lr20xx_close(lr20xx_t *dev);

/* NRESET low 2 ms (100 us at least, 4.2), then wait for BUSY to drop (boot calibration). */
lr20xx_err_t lr20xx_reset(lr20xx_t *dev);
/* Wake from SetSleep: NSS low for 120 us (a frame of zeros that long; the chip needs 100, DS 4.4.1) without waiting
 * for BUSY, which is high while it sleeps, then wait for BUSY to drop. For a sleeping chip only: an awake one takes
 * the zeros as a command. */
lr20xx_err_t lr20xx_wake(lr20xx_t *dev);

/* A write command: frame (from an lr20xx_cmd_* builder) out, BUSY waited for before and after. */
lr20xx_err_t lr20xx_write(lr20xx_t *dev, const uint8_t *frame, size_t len);
/* A write command's frame alone: no GetStatus after it, so a FAIL isn't seen (the next frame's status bytes
 * hold it). For a hot path's commands that can't fail. */
lr20xx_err_t lr20xx_send(lr20xx_t *dev, const uint8_t *frame, size_t len);
/* A read command: the frame, then a second frame for the answer: n bytes after the two status bytes. */
lr20xx_err_t lr20xx_read(lr20xx_t *dev, const uint8_t *frame, size_t len, uint8_t *out, size_t n);
/* n bytes of the RX FIFO (a direct read, 5.4.1.3), 1024 at most. */
lr20xx_err_t lr20xx_read_fifo(lr20xx_t *dev, uint8_t *out, size_t n);
/* The status of the last command (a GetStatus frame). */
lr20xx_err_t lr20xx_get_status(lr20xx_t *dev, lr20xx_status_t *st);

lr20xx_err_t lr20xx_version(lr20xx_t *dev, uint8_t *major, uint8_t *minor);
lr20xx_err_t lr20xx_errors(lr20xx_t *dev, uint16_t *errors);
lr20xx_err_t lr20xx_rx_fifo_level(lr20xx_t *dev, uint16_t *level);
lr20xx_err_t lr20xx_fifo_flags(lr20xx_t *dev, uint8_t *rx, uint8_t *tx); /* and clears them */

/* GetFifoIrqFlags without clearing them (the raw stream keeps a loss visible, docs/iq.md R2.14). */
lr20xx_err_t lr20xx_fifo_flags_peek(lr20xx_t *dev, uint8_t *rx, uint8_t *tx);
lr20xx_err_t lr20xx_tx_fifo_level(lr20xx_t *dev, uint16_t *level);
/* n bytes into the TX FIFO (a direct write, 5.4.1.3). */
lr20xx_err_t lr20xx_write_tx_fifo(lr20xx_t *dev, const uint8_t *data, size_t n);
/* A 32-bit word of the chip's memory or registers (ReadRegMem32). */
lr20xx_err_t lr20xx_read32(lr20xx_t *dev, uint32_t addr, uint32_t *value);
/* 1 to 64 words from addr on (ReadRegMem32 with a count). */
lr20xx_err_t lr20xx_read_mem(lr20xx_t *dev, uint32_t addr, uint32_t *words, int n);
/* The pending IRQs (GetStatus's IrqStatus). */
lr20xx_err_t lr20xx_irq(lr20xx_t *dev, uint32_t *irq);

/* Change the SPI clock, up to 80 MHz: past the chip's 16 MHz (3.7.2) is out of spec, for experiments.
 * LR20XX_ERR_NOT_SUPPORTED if the HAL can't. */
lr20xx_err_t lr20xx_set_clock(lr20xx_t *dev, int hz, int input_delay_ns);

/* Wait for the IRQ pin (a DIO set to IRQ) to be high: true at once if it is, false after timeout_ms or with no
 * IRQ pin. Clearing what raised it is the caller's (ClearIrq). */
bool lr20xx_wait_irq(lr20xx_t *dev, int timeout_ms);
/* Whether the HAL has an IRQ pin. */
bool lr20xx_has_irq(const lr20xx_t *dev);
/* The IRQ pin's level now (1 or 0), -1 with no pin: a GPIO read, no SPI. */
int lr20xx_irq_pin(lr20xx_t *dev);

/* ── Transmit (lr20xx_tx.h sets a mode up; docs/iq.md section 3) ── */

/* Where lr20xx_tx_stream gets its FIFO bytes: up to max into buf, the count back; 0 when there are no more;
 * LR20XX_TX_PULL_WAIT when there are none yet (a stream fed as it goes: asked again shortly). The total must be a
 * whole number of runs (run_bytes each), the last one padded, unless cfg.cut_end. */
typedef size_t (*lr20xx_tx_pull_t)(void *ctx, uint8_t *buf, size_t max);
#define LR20XX_TX_PULL_WAIT ((size_t)-1)

typedef struct {
    const uint8_t *start;   /* lr20xx_tx_start_frame's frame */
    size_t start_len;
    uint32_t run_bytes;     /* FIFO bytes a run takes: n x lr20xx_tx_sample_bytes (FSK4: the payload). 0: a carrier,
                             * keyed up and left on (SetStandby ends it) */
    uint32_t bytes_per_s;   /* how fast the FIFO drains (rate x bytes a sample), to sleep a tick when it can; 0: poll */
    int max_ms;             /* give up after this (0: 10 s) */
    int chunk;              /* top the FIFO up once this many bytes are free (0: 256) */
    int poll_us;            /* the wait between polls with nothing to do (0: 100) */
    uint8_t *buf;           /* a buffer of at least 1020 bytes for the pull (NULL: allocated for the call) */
    size_t buf_len;
    bool irq_pin;           /* the IRQ pin carries TX_DONE and TIMEOUT (SetDioIrqConfig, the caller's): watched instead
                             * of reading the IRQs, and the next run started the moment it rises (needs bytes_per_s) */
    bool cut_end;           /* the data may end inside a run: that run is cut (SetStandby) once the FIFO is empty,
                             * not padded out */
    const volatile bool *stop; /* set: SetStandby at once, the rest dropped (NULL: none) */
    int *learned;           /* irq_pin: TX_DONE's delay past a run's length for runs started on the pin, queued, and
                             * the first (from standby), us (-1: not known), carried from one call to the next with
                             * the same setup (NULL: learned afresh, R3.47) */
    int64_t at_us;          /* key up then (lr20xx_now_us), the FIFO prefilled before; past it by more than
                             * LR20XX_TX_LATE_US when ready: nothing sent, stats.late (0: at once) */
} lr20xx_tx_stream_cfg_t;

#define LR20XX_TX_LATE_US 100

typedef struct {
    int runs;               /* TX_DONEs seen */
    size_t sent;            /* bytes into the FIFO */
    unsigned refills;
    uint8_t tx_flags;       /* the TX FIFO flags at the end: 0x20, an underflow, means the waveform broke (R3.13) */
    bool early;             /* a run ended before taking its bytes and sent nothing (R3.17): send it again */
    bool timed_out;
    int64_t key_us, end_us; /* the first key-up and the last TX_DONE (lr20xx_now_us) */
    uint32_t restart_us;    /* the longest from a TX_DONE seen to the next run's SetTx sent */
    uint32_t gap_us;        /* the longest between two looks at the chip: what the FIFO has to cover */
    int done_level;         /* the fewest bytes in the FIFO at a TX_DONE between runs (-1: none) */
    unsigned queued;        /* runs started by a SetTx sent ahead of the run before's TX_DONE (R3.47) */
    unsigned refused;       /* such SetTx refused (too early): the run started at TX_DONE */
    unsigned starved;       /* level reads that found the FIFO empty while the data hadn't ended: the pull fell
                             * behind (the chip held its last sample, R3.13) */
    bool cut;               /* the last run was cut when its data ran out (cfg.cut_end) */
    bool stopped;           /* cfg.stop ended it */
    bool late;              /* ready past cfg.at_us: nothing was sent */
    int64_t ready_us;       /* the FIFO prefilled, ready to key up (cfg.at_us: before the wait for it) */
} lr20xx_tx_stream_stats_t;

/* Clear the TX FIFO, prefill it from pull, key up with cfg.start, keep the FIFO topped up, and start each next run
 * with cfg.start until the data ends (back-to-back runs: a seam between them, R3.15, R3.44, R3.45; with the IRQ pin,
 * sent just ahead of TX_DONE, R3.47). The mode must be set up first (lr20xx_tx_frames). Runs in the calling thread,
 * sleeping between refills (lr20xx_sleep_us): give it a priority that keeps the FIFO fed.
 * LR20XX_ERR_INVALID_RESPONSE with stats.early for R3.17's early end; LR20XX_ERR_TIMEOUT past max_ms or on the
 * chip's own timeout. On an error the chip is put in standby if it's still in TX. */
lr20xx_err_t lr20xx_tx_stream(lr20xx_t *dev, const lr20xx_tx_stream_cfg_t *cfg, lr20xx_tx_pull_t pull, void *ctx,
                              lr20xx_tx_stream_stats_t *stats);

/* ── Packets with the chip's modems (src/lr20xx_packet.c) ── */

typedef struct {
    uint32_t timeout;   /* SetTx / SetRx's timeout, 32.768 kHz periods (0: none; 0xFFFFFF: RX continuous, DS 7.2.4) */
    int max_ms;         /* the host gives up after this (0: 10 s) */
    size_t fifo_bytes;  /* the FIFO's size: 256 (0) or 1020 / 1000 after the 1 KB mapping (DS 5.3.3.1.1) */
    int poll_us;        /* the wait between polls: 0 sleeps a tick (or until the IRQ pin), fine while the FIFO
                         * outlasts a tick; for a payload longer than the FIFO at a fast rate, a few hundred us */
} lr20xx_packet_cfg_t;

typedef struct {
    uint32_t irq;       /* every IRQ seen while it ran */
    size_t len;         /* bytes sent; received: the packet's length (GetRxPacketLength) */
    int64_t done_us;    /* from the start to TX_DONE / the end of RX */
    bool timed_out, crc_error;
} lr20xx_packet_result_t;

/* One packet out: clear the TX FIFO and the IRQs, load the payload (feeding the rest as it drains when it's
 * longer than the FIFO), SetTx, wait for TX_DONE. The modem and its packet length are set up first.
 * LR20XX_ERR_TIMEOUT on the chip's timeout or past max_ms (the chip is put in standby). */
lr20xx_err_t lr20xx_packet_send(lr20xx_t *dev, const uint8_t *data, size_t n, const lr20xx_packet_cfg_t *cfg,
                                lr20xx_packet_result_t *res);
/* One packet in: SetRx, wait for RX_DONE (draining the FIFO as it fills), then read it out. LR20XX_ERR_TIMEOUT for
 * no packet, LR20XX_ERR_INVALID_CRC for a bad CRC (the payload is still in buf), LR20XX_ERR_INVALID_RESPONSE for a
 * header, length or address error, LR20XX_ERR_INVALID_SIZE when it was longer than max (buf has its start). */
lr20xx_err_t lr20xx_packet_receive(lr20xx_t *dev, uint8_t *buf, size_t max, const lr20xx_packet_cfg_t *cfg,
                                   lr20xx_packet_result_t *res);

/* What failed last, for logs. */
const char *lr20xx_last_error(const lr20xx_t *dev);
/* Commands that came back FAIL or PERR, transfers that timed out on BUSY. */
uint32_t lr20xx_failures(const lr20xx_t *dev);

#ifdef __cplusplus
}
#endif

#endif
