// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * The bridge (bridge.h). The reader task owns the chip while receiving: on core 0 above Wi-Fi (24), it runs
 * lr20xx_iq_poll and puts each read into a message buffer in PSRAM, one message a read (an lr20xx_link_rx_hdr_t and
 * the samples), dropping it when the buffer is full. The link's task takes them out and sends them as RX_DATA, a few
 * reads to a frame. Commands take the chip's mutex between reads; transmissions run on the reader's task (its
 * priority keeps the TX FIFO fed) while the link's task waits.
 *
 * Priorities from RadioFoundry Gateway's LR2021 front end: under Wi-Fi's task (23), Wi-Fi and its traffic kept the
 * reader off the CPU past the FIFO's 2 ms (a loss about every 5 s idle); above it, none in minutes.
 */
#include "bridge.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lr20xx.h"
#include "lr20xx_board.h"
#include "lr20xx_chips.h"
#include "lr20xx_hal_esp_idf.h"
#include "lr20xx_iq.h"
#include "lr20xx_radio.h"

static const char *TAG = "bridge";

#define RING_BYTES ((size_t)CONFIG_LR20XX_RING_KB * 1024)
#define TX_BYTES ((size_t)CONFIG_LR20XX_TX_BUF_KB * 1024)
#define MSG_MAX (LR20XX_LINK_RX_HDR + LR20XX_IQ_FIFO_BYTES + 4) /* one read and its header */
#define READER_PRIO 24
#define STATUS_US 20000000
#ifdef CONFIG_LR20XX_ALLOW_UNTESTED
#define ALLOW_UNTESTED true
#else
#define ALLOW_UNTESTED false
#endif

/* The session's state and its 1 KB read buffer: PSRAM (internal RAM is the SPI's DMA and the stacks). */
EXT_RAM_BSS_ATTR static lr20xx_iq_t iq;

static struct {
    lr20xx_t *dev;
    lr20xx_hal_t hal;
    lr20xx_board_buf_t board;
    bool chip_ok;
    lr20xx_err_t chip_e;
    char chip_err[128];
    uint8_t rom_major, rom_minor;

    SemaphoreHandle_t lock; /* the chip: the reader, or a command */
    volatile int wanting;   /* commands waiting for the lock: the reader gives them a tick */
    TaskHandle_t reader;

    MessageBufferHandle_t ring;
    StaticMessageBuffer_t ring_struct;
    volatile bool streaming;
    bool paused;            /* by OTA */
    bool configured;
    lr20xx_iq_cfg_t cfg;
    uint64_t next_index;    /* the reader's: samples delivered since RX_START */
    bool gap_pending;       /* the reader's: a loss not yet flagged in a message */
    volatile uint32_t link_drops, ring_peak;
    volatile bool rx_failed; /* the session failed: an event to send */
    lr20xx_err_t rx_e;
    char rx_err[128];

    /* the link task's merge buffer: a message taken out that didn't fit the frame being built */
    uint8_t *carry;
    size_t carry_len;

    /* transmit */
    uint8_t *tx_buf;
    size_t tx_need, tx_have;
    bool tx_open, tx_over;
    lr20xx_iq_tx_t tx;
    struct {
        volatile bool pending;
        lr20xx_err_t e;
        lr20xx_tx_stream_stats_t stats;
        size_t pos;
        SemaphoreHandle_t done;
    } job;
} B;

static void lock(void)
{
    __atomic_add_fetch(&B.wanting, 1, __ATOMIC_RELAXED);
    xSemaphoreTake(B.lock, portMAX_DELAY);
    __atomic_sub_fetch(&B.wanting, 1, __ATOMIC_RELAXED);
}

static void unlock(void) { xSemaphoreGive(B.lock); }

bool bridge_streaming(void) { return B.streaming; }

/* ── the reader ─────────────────────────────────────────────────────────── */

/* A read's samples into the buffer as one message (the reader's, under the lock). */
static void sink(void *ctx, const uint8_t *data, size_t n, unsigned flags, int64_t t_us)
{
    EXT_RAM_BSS_ATTR static uint8_t msg[MSG_MAX];
    const size_t bytes = n * (size_t)iq.plan.sample_bytes;
    const uint64_t first = B.next_index;
    B.next_index += n;
    if (bytes + LR20XX_LINK_RX_HDR > sizeof msg ||
        xMessageBufferSpacesAvailable(B.ring) < bytes + LR20XX_LINK_RX_HDR + sizeof(size_t)) {
        B.link_drops++;
        B.gap_pending = true;
        return;
    }
    lr20xx_link_rx_hdr_t h = {.first = first, .t_us = t_us, .format = (uint8_t)iq.plan.format,
                              .flags = (flags & LR20XX_IQ_GAP) || B.gap_pending ? LR20XX_IQ_GAP : 0,
                              .bits = iq.plan.format == LR20XX_IQ_CS8 ? 8 : 16}; /* packed on the way out */
    lr20xx_link_put_rx_hdr(msg, &h);
    memcpy(msg + LR20XX_LINK_RX_HDR, data, bytes);
    if (xMessageBufferSend(B.ring, msg, LR20XX_LINK_RX_HDR + bytes, 0) == 0) {
        B.link_drops++;
        B.gap_pending = true;
        return;
    }
    B.gap_pending = false;
    size_t used = RING_BYTES - xMessageBufferSpacesAvailable(B.ring);
    if (used > B.ring_peak)
        B.ring_peak = (uint32_t)used;
}

static size_t tx_pull(void *ctx, uint8_t *buf, size_t max)
{
    size_t n = B.tx_need - B.job.pos;
    if (n > max)
        n = max;
    memcpy(buf, B.tx_buf + B.job.pos, n);
    B.job.pos += n;
    return n;
}

static void run_tx(void)
{
    lock();
    B.job.pos = 0;
    memset(&B.job.stats, 0, sizeof B.job.stats);
    B.job.e = lr20xx_iq_transmit(&iq, &B.tx, tx_pull, NULL, &B.job.stats);
    const lr20xx_tx_stream_stats_t *s = &B.job.stats;
    ESP_LOGI(TAG, "tx: %d runs, %u refills, restart %" PRIu32 " us, gap %" PRIu32 " us, FIFO at TX_DONE %d, flags 0x%02x",
             s->runs, s->refills, s->restart_us, s->gap_us, s->done_level, s->tx_flags);
    if (B.streaming)
        B.gap_pending = true; /* no samples while it transmitted */
    unlock();
    B.job.pending = false;
    xSemaphoreGive(B.job.done);
}

static void reader_task(void *arg)
{
    for (;;) {
        if (B.job.pending) {
            run_tx();
            continue;
        }
        if (!B.streaming) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
            continue;
        }
        if (B.wanting)
            vTaskDelay(1); /* the mutex alone would go straight back to the reader */
        lock();
        if (!B.streaming || B.job.pending) {
            unlock();
            continue;
        }
        int r = lr20xx_iq_poll(&iq, sink, NULL, 2);
        if (r < 0) {
            B.streaming = false;
            B.rx_e = r;
            snprintf(B.rx_err, sizeof B.rx_err, "%s", *iq.error ? iq.error : lr20xx_err_name(r));
            B.rx_failed = true;
        }
        unlock();
        if (r < 0)
            ESP_LOGW(TAG, "receiving stopped: %s", B.rx_err);
    }
}

/* Every 20 s while receiving, from esp_timer's task (a log line in the reader would stall it past the FIFO). */
static void status_line(void *arg)
{
    static int64_t t0;
    static uint32_t run0;
    int64_t now = esp_timer_get_time();
    uint32_t run = 0;
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    TaskStatus_t ts;
    vTaskGetInfo(B.reader, &ts, pdFALSE, eRunning);
    run = (uint32_t)ts.ulRunTimeCounter; /* esp_timer us */
#endif
    if (B.streaming && t0) {
        const lr20xx_iq_stats_t s = iq.stats;
        ESP_LOGI(TAG,
                 "rx %.0f S/s %s at %.5f MHz: %" PRIu64 " samples, %" PRIu32 " losses, %" PRIu32 " restarts, %" PRIu32
                 " errors, %" PRIu32 " IRQs late; buffer peak %" PRIu32 " KB, %" PRIu32 " link drops, %" PRIu32
                 " SPI bounces; reader %.1f%% of core 0; internal RAM free %u",
                 iq.plan.rate, iq.plan.format == LR20XX_IQ_CS8 ? "cs8" : "cs16", iq.cfg.center_hz / 1e6, s.samples,
                 s.losses, s.restarts, s.errors, s.irq_late, B.ring_peak / 1024, B.link_drops,
                 lr20xx_hal_esp_bounces(&B.hal), (run - run0) / ((now - t0) / 100.0),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    }
    t0 = now;
    run0 = run;
}

void bridge_init(void)
{
    B.lock = xSemaphoreCreateMutex();
    B.job.done = xSemaphoreCreateBinary();
    /* in PSRAM where there's some; a chip without (C3, C6) has smaller ones in internal RAM (Kconfig's defaults) */
#if CONFIG_SPIRAM
    const uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
#else
    const uint32_t caps = MALLOC_CAP_8BIT;
#endif
    uint8_t *ring_mem = heap_caps_malloc(RING_BYTES + 1, caps);
    B.carry = heap_caps_malloc(MSG_MAX, caps);
    B.tx_buf = heap_caps_malloc(TX_BYTES, caps);
    if (!ring_mem || !B.carry || !B.tx_buf) {
        ESP_LOGE(TAG, "no room for the buffers (%u KB and %u KB)", CONFIG_LR20XX_RING_KB, CONFIG_LR20XX_TX_BUF_KB);
        abort();
    }
    B.ring = xMessageBufferCreateStatic(RING_BYTES, ring_mem, &B.ring_struct);

    char err[96];
    if (lr20xx_board_parse(CONFIG_LR20XX_BOARD, &B.board, err, sizeof err)) {
        ESP_LOGE(TAG, "board \"%s\": %s; using \"generic\"", CONFIG_LR20XX_BOARD, err);
        lr20xx_board_parse("generic", &B.board, err, sizeof err);
    }

    const lr20xx_esp_cfg_t hc = {
        .spi_host = CONFIG_LR20XX_SPI_HOST,
        .sck = CONFIG_LR20XX_SCK,
        .mosi = CONFIG_LR20XX_MOSI,
        .miso = CONFIG_LR20XX_MISO,
        .nss = CONFIG_LR20XX_NSS,
        .busy = CONFIG_LR20XX_BUSY,
        .nreset = CONFIG_LR20XX_NRESET,
        .irq = CONFIG_LR20XX_IRQ_GPIO,
        .pa_en = CONFIG_LR20XX_PA_EN_GPIO,
        .clock_hz = CONFIG_LR20XX_SPI_HZ,
    };
    lr20xx_err_t e = lr20xx_hal_esp_open(&hc, &B.hal);
    if (e == LR20XX_OK) {
        /* the handle's 1 KB SPI frames: internal, DMA-capable, aligned to 4 */
        void *mem = heap_caps_aligned_calloc(4, 1, lr20xx_handle_size(), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const lr20xx_config_t c = {.hal = B.hal, .mem = mem};
        e = mem ? lr20xx_open(&c, &B.dev) : LR20XX_ERR_NO_MEM;
        if (e != LR20XX_OK)
            snprintf(B.chip_err, sizeof B.chip_err, "no chip: %s", lr20xx_err_name(e));
    } else {
        snprintf(B.chip_err, sizeof B.chip_err, "SPI and pins: %s", lr20xx_err_name(e));
    }
    if (e == LR20XX_OK) {
        lr20xx_version(B.dev, &B.rom_major, &B.rom_minor);
        e = lr20xx_iq_init(&iq, B.dev, &B.board.b, ALLOW_UNTESTED);
        if (e != LR20XX_OK)
            snprintf(B.chip_err, sizeof B.chip_err, "%s", *iq.error ? iq.error : lr20xx_err_name(e));
    }
    B.chip_e = e;
    B.chip_ok = e == LR20XX_OK;
    if (B.chip_ok)
        ESP_LOGI(TAG, "LR20xx ROM %u.%u on board \"%s\" (SPI %d MHz, IRQ GPIO %d)", B.rom_major, B.rom_minor,
                 B.board.b.name, CONFIG_LR20XX_SPI_HZ / 1000000, CONFIG_LR20XX_IRQ_GPIO);
    else
        ESP_LOGE(TAG, "%s", B.chip_err);

    xTaskCreatePinnedToCore(reader_task, "lr20xx", 6144, NULL, READER_PRIO, &B.reader, 0);
    esp_timer_handle_t t;
    const esp_timer_create_args_t a = {.callback = status_line, .name = "lr20xx status"};
    if (esp_timer_create(&a, &t) == ESP_OK)
        esp_timer_start_periodic(t, STATUS_US);
}

/* ── receiving ──────────────────────────────────────────────────────────── */

static void drop_buffered(void)
{
    xMessageBufferReset(B.ring);
    B.carry_len = 0;
}

static void rx_stop(void)
{
    B.streaming = false;
    lock();
    if (iq.in_session)
        lr20xx_iq_stop(&iq);
    unlock();
    drop_buffered();
}

static lr20xx_err_t rx_start(void)
{
    lock();
    drop_buffered();
    B.next_index = 0;
    B.gap_pending = false;
    B.rx_failed = false;
    B.ring_peak = 0;
    lr20xx_err_t e = lr20xx_iq_start(&iq, &B.cfg);
    if (e == LR20XX_OK)
        B.streaming = true;
    unlock();
    if (e == LR20XX_OK)
        xTaskNotifyGive(B.reader);
    return e;
}

bool bridge_pause(void)
{
    bool was = B.streaming;
    if (was)
        rx_stop();
    B.paused = true;
    return was;
}

void bridge_resume(bool was_receiving)
{
    B.paused = false;
    if (was_receiving && B.chip_ok)
        rx_start();
}

/* ── the link ───────────────────────────────────────────────────────────── */

size_t bridge_hello(uint8_t *p)
{
    lr20xx_link_hello_t h = {
        .version = LR20XX_LINK_VERSION,
        .rom_major = B.rom_major,
        .rom_minor = B.rom_minor,
        .rom_known = lr20xx_rom_find(B.rom_major, B.rom_minor) != NULL,
        .caps = B.chip_ok ? LR20XX_LINK_CAP_RX | LR20XX_LINK_CAP_TX |
                                (CONFIG_LR20XX_IRQ_GPIO >= 0 ? LR20XX_LINK_CAP_IRQ : 0)
                          : 0,
        .tx_max_samples = (uint32_t)(TX_BYTES / 2), /* I/Q's 2 bytes a sample; FREQ and PHASE_STEP take twice */
        .freq_min = B.board.b.freq_min,
        .freq_max = B.board.b.freq_max,
        .tx_power_max = B.board.b.tx_power_max,
    };
    snprintf(h.board, sizeof h.board, "%s", B.board.b.name ? B.board.b.name : "?");
    snprintf(h.firmware, sizeof h.firmware, "bridge %.16s", esp_app_get_description()->version);
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(h.serial, sizeof h.serial, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return lr20xx_link_put_hello(p, &h);
}

static int send_frame(bridge_link_t *l, size_t n) { return n ? l->send(l, l->out, n) : -1; }

static int send_event(bridge_link_t *l, uint16_t kind, int32_t err, const char *text)
{
    return send_frame(l, lr20xx_link_event(l->out, sizeof l->out, kind, err, text));
}

void bridge_link_open(bridge_link_t *l)
{
    lr20xx_link_parser_init(&l->parser);
    l->hello_sent_event = false;
}

void bridge_link_closed(bridge_link_t *l)
{
    if (B.streaming)
        rx_stop();
    B.tx_open = false;
}

static bool tx_mode_ok(uint8_t m)
{
    return m == LR20XX_TX_FREQ || m == LR20XX_TX_PHASE_STEP || m == LR20XX_TX_IQ_PHASE || m == LR20XX_TX_IQ_POLAR;
}

/* A command: its answer into ans (*n bytes), the error back. */
static lr20xx_err_t command(bridge_link_t *l, uint16_t cmd, const uint8_t *a, size_t an, uint8_t *ans, size_t *n)
{
    *n = 0;
    if (cmd == LR20XX_CMD_HELLO) {
        /* a new session: a serial port has no connection to say so (link_serial.c) */
        if (B.streaming)
            rx_stop();
        B.tx_open = false;
        *n = bridge_hello(ans);
        return LR20XX_OK;
    }
    if (cmd == LR20XX_CMD_STATS) {
        lock();
        const lr20xx_iq_stats_t s = iq.stats;
        unlock();
        const lr20xx_link_stats_t v = {.samples = s.samples, .losses = s.losses, .restarts = s.restarts,
                                       .errors = s.errors, .reads = s.reads, .irq_late = s.irq_late,
                                       .link_drops = B.link_drops, .ring_peak = B.ring_peak};
        *n = lr20xx_link_put_stats(ans, &v);
        return LR20XX_OK;
    }
    if (!B.chip_ok)
        return LR20XX_ERR_INVALID_STATE;
    lr20xx_err_t e = LR20XX_OK;
    switch (cmd) {
    case LR20XX_CMD_RX_CONFIG: {
        lr20xx_link_rx_config_t c;
        if (!lr20xx_link_get_rx_config(a, an, &c))
            return LR20XX_ERR_INVALID_SIZE;
        const lr20xx_iq_cfg_t cfg = {.rate = c.rate, .mode = (lr20xx_iq_mode_t)c.mode, .center_hz = c.center_hz,
                                     .ppm = c.ppm, .gain = c.gain};
        lr20xx_iq_plan_t plan;
        if (c.gain < 0 || c.gain > 13)
            return LR20XX_ERR_INVALID_ARG;
        if ((e = lr20xx_iq_plan(cfg.rate, cfg.mode, &plan)) != LR20XX_OK)
            return e;
        B.cfg = cfg;
        B.configured = true;
        if (B.streaming) {
            lock();
            e = lr20xx_iq_start(&iq, &B.cfg);
            if (e == LR20XX_OK) {
                plan = iq.plan;
                B.gap_pending = true;
            } else {
                B.streaming = false;
            }
            unlock();
        }
        const lr20xx_link_plan_t p = {.mode = (uint8_t)plan.mode, .format = (uint8_t)plan.format,
                                      .rate_num = plan.rate_num, .rate_den = plan.rate_den,
                                      .passband_hz = plan.passband_hz};
        *n = lr20xx_link_put_plan(ans, &p);
        return e;
    }
    case LR20XX_CMD_RX_START:
        if (!B.configured || B.paused)
            return LR20XX_ERR_INVALID_STATE;
        return B.streaming ? LR20XX_OK : rx_start();
    case LR20XX_CMD_RX_STOP:
        rx_stop();
        return LR20XX_OK;
    case LR20XX_CMD_SET_GAIN:
        if (an < 1 || (int8_t)a[0] < 0 || (int8_t)a[0] > 13)
            return LR20XX_ERR_INVALID_ARG;
        B.cfg.gain = (int8_t)a[0];
        lock();
        e = lr20xx_iq_set_gain(&iq, B.cfg.gain);
        unlock();
        return e;
    case LR20XX_CMD_TX_BEGIN: {
        lr20xx_link_tx_begin_t t;
        if (!lr20xx_link_get_tx_begin(a, an, &t))
            return LR20XX_ERR_INVALID_SIZE;
        if (!tx_mode_ok(t.mode))
            return LR20XX_ERR_NOT_SUPPORTED;
        const size_t sb = (size_t)lr20xx_tx_sample_bytes((lr20xx_tx_mode_t)t.mode);
        if (!t.samples || t.samples > TX_BYTES / sb)
            return LR20XX_ERR_INVALID_SIZE;
        double p = t.power_dbm > B.board.b.tx_power_max ? B.board.b.tx_power_max : t.power_dbm;
        B.tx = (lr20xx_iq_tx_t){.mode = (lr20xx_tx_mode_t)t.mode, .center_hz = t.center_hz, .ppm = t.ppm,
                                .rate = t.rate, .fdev_hz = t.fdev_hz, .power_dbm = p, .samples = t.samples};
        B.tx_need = (size_t)t.samples * sb;
        B.tx_have = 0;
        B.tx_over = false;
        B.tx_open = true;
        return LR20XX_OK;
    }
    case LR20XX_CMD_TX_COMMIT: {
        if (!B.tx_open)
            return LR20XX_ERR_INVALID_STATE;
        B.tx_open = false;
        if (B.tx_have != B.tx_need || B.tx_over)
            return LR20XX_ERR_INVALID_SIZE;
        B.job.pending = true;
        xTaskNotifyGive(B.reader);
        xSemaphoreTake(B.job.done, portMAX_DELAY);
        const lr20xx_tx_stream_stats_t *s = &B.job.stats;
        const lr20xx_link_tx_result_t r = {.runs = s->runs, .sent = (uint32_t)s->sent, .tx_flags = s->tx_flags,
                                           .early = s->early, .timed_out = s->timed_out};
        *n = lr20xx_link_put_tx_result(ans, &r);
        return B.job.e;
    }
    case LR20XX_CMD_TX_ABORT:
        B.tx_open = false;
        return LR20XX_OK;
    default:
        break;
    }
    /* The chip by hand: only while it isn't receiving. */
    if (B.streaming)
        return LR20XX_ERR_INVALID_STATE;
    switch (cmd) {
    case LR20XX_CMD_READ_REG: {
        if (an < 5 || a[4] < 1 || a[4] > 64)
            return LR20XX_ERR_INVALID_ARG;
        uint32_t w[64];
        lock();
        e = lr20xx_read_mem(B.dev, lr20xx_link_u32(a), w, a[4]);
        unlock();
        for (int i = 0; e == LR20XX_OK && i < a[4]; i++)
            lr20xx_link_put_u32(ans + 4 * i, w[i]);
        *n = e == LR20XX_OK ? 4u * a[4] : 0;
        return e;
    }
    case LR20XX_CMD_WRITE_REG: {
        if (an < 8)
            return LR20XX_ERR_INVALID_SIZE;
        uint8_t f[12];
        lock();
        e = lr20xx_write(B.dev, f, lr20xx_cmd_write_reg32(f, lr20xx_link_u32(a), lr20xx_link_u32(a + 4)));
        unlock();
        return e;
    }
    case LR20XX_CMD_RAW: {
        if (an < 4 || an - 2 > 260)
            return LR20XX_ERR_INVALID_SIZE;
        const size_t want = lr20xx_link_u16(a);
        if (want > 256)
            return LR20XX_ERR_INVALID_SIZE;
        lr20xx_status_t st = {0};
        lock();
        e = want ? lr20xx_read(B.dev, a + 2, an - 2, ans + 2, want) : lr20xx_send(B.dev, a + 2, an - 2);
        lr20xx_get_status(B.dev, &st);
        unlock();
        /* status: the command status (0 FAIL, 1 PERR, 2 OK, 3 DATA) and the chip's mode, as lr20xx_status gives */
        ans[0] = st.cmd;
        ans[1] = st.mode;
        *n = 2 + (e == LR20XX_OK ? want : 0);
        return e;
    }
    case LR20XX_CMD_TEMP: {
        uint8_t f[4], t[2] = {0};
        lock();
        e = lr20xx_read(B.dev, f, lr20xx_radio_get_temp(f, LR20XX_TEMP_VBE, true, 13), t, 2);
        unlock();
        if (e == LR20XX_OK) {
            lr20xx_link_put_f32(ans, (float)lr20xx_radio_temp_c(t, true));
            *n = 4;
        }
        return e;
    }
    default:
        return LR20XX_ERR_NOT_SUPPORTED;
    }
}

static int handle(bridge_link_t *l, const lr20xx_link_frame_t *f)
{
    if (f->type == LR20XX_LINK_TX_DATA) {
        if (B.tx_open) {
            size_t room = B.tx_need - B.tx_have, k = f->len < room ? f->len : room;
            memcpy(B.tx_buf + B.tx_have, f->payload, k);
            B.tx_have += k;
            B.tx_over |= k < f->len;
        }
        return 0;
    }
    uint16_t seq, cmd;
    const uint8_t *a;
    size_t an;
    if (!lr20xx_link_parse_req(f, &seq, &cmd, &a, &an))
        return 0; /* not for the device */
    /* the arguments live in the parser's buffer, which the answer doesn't touch */
    static uint8_t ans[300];
    size_t n = 0;
    lr20xx_err_t e = command(l, cmd, a, an, ans, &n);
    if (send_frame(l, lr20xx_link_rsp(l->out, sizeof l->out, seq, e, ans, n)) < 0)
        return -1;
    if (cmd == LR20XX_CMD_HELLO && !B.chip_ok && !l->hello_sent_event) {
        l->hello_sent_event = true;
        return send_event(l, LR20XX_EVENT_RX_FAILED, B.chip_e, B.chip_err);
    }
    return 0;
}

int bridge_link_input(bridge_link_t *l, const uint8_t *data, size_t n)
{
    while (n) {
        size_t took = lr20xx_link_feed(&l->parser, data, n);
        data += took;
        n -= took;
        lr20xx_link_frame_t f;
        while (lr20xx_link_next(&l->parser, &f))
            if (handle(l, &f) < 0)
                return -1;
    }
    return 0;
}

int bridge_link_pump(bridge_link_t *l, int wait_ms)
{
    if (B.rx_failed) {
        B.rx_failed = false;
        if (send_event(l, LR20XX_EVENT_RX_FAILED, B.rx_e, B.rx_err) < 0)
            return -1;
    }
    if (!B.streaming && !B.carry_len && !xStreamBufferNextMessageLengthBytes(B.ring))
        return 0;
    /* (xMessageBufferNextLengthBytes has a stray ';' in IDF 5.5's FreeRTOS: the stream buffer call it wraps is
     * used.) The frame's payload is built in place: the first message, then those that follow on without a gap. */
    uint8_t *p = l->out + LR20XX_LINK_HEADER;
    size_t len;
    if (B.carry_len) {
        memcpy(p, B.carry, B.carry_len);
        len = B.carry_len;
        B.carry_len = 0;
    } else {
        len = xMessageBufferReceive(B.ring, p, LR20XX_LINK_MAX_PAYLOAD, pdMS_TO_TICKS(wait_ms));
        if (len < LR20XX_LINK_RX_HDR)
            return 0;
    }
    lr20xx_link_rx_hdr_t h;
    lr20xx_link_get_rx_hdr(p, len, &h);
    const size_t sb = h.format == LR20XX_IQ_CS8 ? 2 : 4;
    uint64_t next = h.first + (len - LR20XX_LINK_RX_HDR) / sb;
    for (;;) {
        size_t m = xStreamBufferNextMessageLengthBytes(B.ring);
        if (m < LR20XX_LINK_RX_HDR || len + m - LR20XX_LINK_RX_HDR > LR20XX_LINK_MAX_PAYLOAD)
            break;
        m = xMessageBufferReceive(B.ring, B.carry, MSG_MAX, 0);
        lr20xx_link_rx_hdr_t h2;
        if (!lr20xx_link_get_rx_hdr(B.carry, m, &h2))
            break;
        if (h2.flags || h2.first != next || h2.format != h.format) {
            B.carry_len = m; /* starts the next frame */
            break;
        }
        memcpy(p + len, B.carry + LR20XX_LINK_RX_HDR, m - LR20XX_LINK_RX_HDR);
        len += m - LR20XX_LINK_RX_HDR;
        next = h2.first + (m - LR20XX_LINK_RX_HDR) / sb;
    }
    /* packed in place to the narrowest width that holds this frame's values (lr20xx_link_pack): over USB's ~1 MB/s,
     * int8 always takes 6 bits of 8 and int16 noise about 10 of 16 */
    uint8_t *samples = p + LR20XX_LINK_RX_HDR;
    const unsigned format = h.format == LR20XX_IQ_CS8;
    const size_t values = (len - LR20XX_LINK_RX_HDR) / (format ? 1 : 2);
    h.bits = (uint8_t)lr20xx_link_bits(samples, values, format);
    const size_t n = lr20xx_link_pack(samples, samples, values, format, h.bits);
    return send_frame(l, lr20xx_link_rx_data(l->out, sizeof l->out, &h, samples, n)) < 0 ? -1 : 1;
}

int bridge_status_json(char *buf, size_t n)
{
    const lr20xx_iq_stats_t s = iq.stats;
    return snprintf(buf, n,
                    "\"chip\":\"%s\",\"rom\":\"%u.%u\",\"board\":\"%s\",\"receiving\":%s,\"rate\":%.1f,"
                    "\"center_hz\":%.0f,\"samples\":%" PRIu64 ",\"losses\":%" PRIu32 ",\"restarts\":%" PRIu32
                    ",\"errors\":%" PRIu32 ",\"link_drops\":%" PRIu32 ",\"ring_peak\":%" PRIu32,
                    B.chip_ok ? "ok" : B.chip_err, B.rom_major, B.rom_minor, B.board.b.name ? B.board.b.name : "",
                    B.streaming ? "true" : "false", B.streaming ? iq.plan.rate : 0.0,
                    B.streaming ? iq.cfg.center_hz : 0.0, s.samples, s.losses, s.restarts, s.errors, B.link_drops,
                    B.ring_peak);
}
