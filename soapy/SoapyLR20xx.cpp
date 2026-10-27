// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/* The SoapySDR Device (SoapyLR20xx.hpp) and the module's registration. */
#include "SoapyLR20xx.hpp"

#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstring>
#include <unistd.h>

#include "lr20xx_cmd.h"
#include "lr20xx_tx.h"

namespace soapy_lr20xx {

namespace {

const char *const MODES[] = {"auto", "lora", "fsk16", "conv8"};
/* lr20xx_tx_mode_t's values for the tx_mode setting's names */
const struct {
    const char *name;
    int mode;
    int bytes;
} TX_MODES[] = {{"freq", LR20XX_TX_FREQ, 1}, {"phase_step", LR20XX_TX_PHASE_STEP, 1},
                {"iq_phase", LR20XX_TX_IQ_PHASE, 2}, {"iq_polar", LR20XX_TX_IQ_POLAR, 2}};
const char *tx_mode_name(int mode)
{
    for (auto &m : TX_MODES)
        if (m.mode == mode)
            return m.name;
    return "?";
}
const double TX_RATES[] = {31250, 38400, 48000, 62500, 76800, 96000, 125000, 250000}; /* docs/iq.md R3.10 */
constexpr double TX_POWER_MIN = -9.5; /* the LF PA's floor, 0.5 dB steps (DS 7.4.3) */

std::string rom_text(const lr20xx_link_hello_t &h)
{
    return std::to_string(h.rom_major) + "." + std::to_string(h.rom_minor);
}

bool truthy(const std::string &v) { return v == "true" || v == "1" || v == "yes"; }

/* A stream's calls under way (readStream, writeStream, readStreamStatus): closeStream from another thread waits
 * for them to leave before it frees the stream. */
struct Calls {
    int n = 0;
    std::mutex m;
    std::condition_variable cv;
    void wait_none()
    {
        std::unique_lock<std::mutex> l(m);
        cv.wait(l, [this] { return n == 0; });
    }
};

class InCall {
public:
    explicit InCall(Calls &c) : c_(c)
    {
        std::lock_guard<std::mutex> l(c_.m);
        c_.n++;
    }
    ~InCall()
    {
        std::lock_guard<std::mutex> l(c_.m);
        if (--c_.n == 0)
            c_.cv.notify_all();
    }

private:
    Calls &c_;
};

} // namespace

struct Device::RxStream {
    Calls calls;
    std::string format;
    Chunk cur;
    size_t off = 0;
    bool active = false;
    uint32_t rate_num = 1, rate_den = 1;
    /* The device's clock (ns) at sample index 0. Each chunk's t_us is when its read ended, so never before its last
     * sample: base_ns is the lowest such bound, let rise slowly (the chip's sample clock and the device's drift
     * apart by tens of ppm). After a loss (the index doesn't count the lost samples) it's set afresh from that one
     * chunk, less a read's usual lateness past the bound (late_ns). The times given follow it from applied_ns, down
     * by under a sample a chunk, so they never go back. */
    bool anchored = false;
    long double base_ns = 0, applied_ns = 0, late_ns = 0;
    long double cur_ns = 0; /* the chunk being read: its first sample's time */
};

struct Device::TxStream {
    Calls calls;
    std::string format;
    /* the burst under way: begun on the device, its samples converted and sent as they come */
    std::atomic<bool> open{false};
    std::atomic<bool> committing{false}; /* TX_COMMIT waits for the burst to go out */
    std::atomic<bool> closing{false};
    std::atomic<bool> aborted{false}; /* deactivateStream aborted the burst: its error is no news */
    int mode = 0;
    double rate = 0;
    uint64_t n = 0;                          /* samples written */
    std::vector<std::complex<float>> buf;    /* written, not yet converted (iq_polar: a dip held for what follows) */
    std::complex<float> ctx;                 /* the last sample converted (iq_phase: the last nonzero one) */
    bool has_ctx = false;
    float last_hz = 0;                       /* freq, phase_step: the last sample's */
    uint64_t over = 0, counted = 0;          /* past the mode's frequency limit, of those counted */
    std::deque<std::pair<int, long long>> status; /* (flags or an error code, the key-up's time or 0) */
    std::mutex m;
    std::condition_variable cv;
};

Device::Device(std::unique_ptr<Backend> b, const SoapySDR::Kwargs &args) : b_(std::move(b))
{
    hello_ = b_->hello();
    if (hello_.version != LR20XX_LINK_VERSION)
        SoapySDR::logf(SOAPY_SDR_WARNING, "LR20xx: link version %u, this module speaks %u", hello_.version,
                       LR20XX_LINK_VERSION);
    if (!hello_.rom_known)
        SoapySDR::logf(SOAPY_SDR_WARNING, "LR20xx: ROM %s hasn't been tested for raw I/Q (lr20xx_chips.h)",
                       rom_text(hello_).c_str());
    center_ = 915e6 >= hello_.freq_min && 915e6 <= hello_.freq_max ? 915e6 : hello_.freq_min;
    for (auto &kv : args)
        if (kv.first == "iq_mode" || kv.first == "tx_mode" || kv.first == "tx_fdev" || kv.first == "tx_rate" ||
            kv.first == "polar_rmin" || kv.first == "allow_untested")
            writeSetting(kv.first, kv.second);
}

Device::~Device()
{
    try {
        if (rx_ && rx_->active)
            b_->rx_stop();
    } catch (...) {
    }
    delete rx_;
    delete tx_;
}

std::string Device::getHardwareKey() const { return std::string("LR20xx ") + hello_.board; }

SoapySDR::Kwargs Device::getHardwareInfo() const
{
    return {{"board", hello_.board},
            {"rom", rom_text(hello_)},
            {"rom_known", hello_.rom_known ? "true" : "false"},
            {"firmware", hello_.firmware},
            {"serial", hello_.serial},
            {"link_version", std::to_string(hello_.version)},
            {"irq", hello_.caps & LR20XX_LINK_CAP_IRQ ? "true" : "false"},
            {"tx_max_samples", std::to_string(hello_.tx_max_samples)}};
}

/* ── Streams ── */

std::vector<std::string> Device::getStreamFormats(const int, const size_t) const
{
    return {SOAPY_SDR_CF32, SOAPY_SDR_CS16, SOAPY_SDR_CS8};
}

std::string Device::getNativeStreamFormat(const int dir, const size_t, double &fullScale) const
{
    if (dir == SOAPY_SDR_TX) {
        fullScale = 1.0; /* converted to the transmit mode's int8 samples at the end of a burst */
        return SOAPY_SDR_CF32;
    }
    std::lock_guard<std::mutex> l(m_);
    if (have_plan_ && plan_.format == 1) {
        fullScale = 32; /* cs8 clips at -32..+31 (docs/iq.md R2.26) */
        return SOAPY_SDR_CS8;
    }
    fullScale = 32768;
    return SOAPY_SDR_CS16;
}

SoapySDR::Stream *Device::setupStream(const int dir, const std::string &format, const std::vector<size_t> &channels,
                                      const SoapySDR::Kwargs &)
{
    if (channels.size() > 1 || (channels.size() == 1 && channels[0] != 0))
        throw std::runtime_error("LR20xx: one channel, 0");
    if (format != SOAPY_SDR_CF32 && format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CS8)
        throw std::runtime_error("LR20xx: formats CF32, CS16, CS8 (not " + format + ")");
    if (dir == SOAPY_SDR_RX) {
        if (rx_)
            throw std::runtime_error("LR20xx: one RX stream at a time");
        rx_ = new RxStream;
        rx_->format = format;
        return (SoapySDR::Stream *)rx_;
    }
    if (tx_)
        throw std::runtime_error("LR20xx: one TX stream at a time");
    if (!(hello_.caps & LR20XX_LINK_CAP_TX))
        throw std::runtime_error("LR20xx: this device doesn't transmit");
    tx_ = new TxStream;
    tx_->format = format;
    return (SoapySDR::Stream *)tx_;
}

void Device::closeStream(SoapySDR::Stream *s)
{
    /* another thread may be in the stream's calls: the stream is taken from them first, its burst aborted, and freed
     * once they have left */
    if ((void *)s == (void *)rx_) {
        deactivateStream(s, 0, 0);
        RxStream *r;
        {
            std::lock_guard<std::mutex> l(streams_m_);
            r = rx_;
            rx_ = nullptr;
        }
        r->calls.wait_none(); /* a readStream waiting for samples: up to its timeout */
        delete r;
    } else if ((void *)s == (void *)tx_) {
        TxStream *t;
        {
            std::lock_guard<std::mutex> l(streams_m_);
            t = tx_;
            tx_ = nullptr;
        }
        t->closing = true;
        if (t->open || t->committing)
            try {
                b_->tx_abort(); /* a burst never ended, or one still going out */
            } catch (...) {
            }
        {
            std::lock_guard<std::mutex> l(t->m);
            t->cv.notify_all(); /* readStreamStatus */
        }
        t->calls.wait_none();
        delete t;
    }
}

size_t Device::getStreamMTU(SoapySDR::Stream *s) const
{
    /* an RX_DATA frame holds up to 4078 bytes of samples */
    return (void *)s == (void *)tx_ ? 4096 : 1019;
}

int Device::activateStream(SoapySDR::Stream *s, const int flags, const long long, const size_t)
{
    if ((void *)s == (void *)tx_) {
        tx_->closing = false;
        return 0;
    }
    if ((void *)s != (void *)rx_)
        return SOAPY_SDR_STREAM_ERROR;
    if (flags & SOAPY_SDR_HAS_TIME)
        return SOAPY_SDR_NOT_SUPPORTED;
    std::lock_guard<std::mutex> l(m_);
    if (!have_plan_)
        configure_rx();
    b_->rx_start();
    rx_->cur = Chunk{};
    rx_->off = 0;
    rx_->anchored = false;
    rx_->rate_num = plan_.rate_num;
    rx_->rate_den = plan_.rate_den ? plan_.rate_den : 1;
    rx_->active = true;
    return 0;
}

int Device::deactivateStream(SoapySDR::Stream *s, const int, const long long)
{
    if ((void *)s == (void *)rx_ && rx_->active) {
        std::lock_guard<std::mutex> l(m_);
        rx_->active = false;
        b_->rx_stop();
    } else if ((void *)s == (void *)tx_ && (tx_->open || tx_->committing)) {
        /* a burst not ended, or one waiting on its time or going out (writeStream in another thread): aborted, and
         * that writeStream returns with STREAM_ERROR in readStreamStatus */
        tx_->aborted = true;
        tx_->open = false; /* the next write begins a new burst */
        try {
            b_->tx_abort();
        } catch (const std::exception &e) {
            SoapySDR::logf(SOAPY_SDR_ERROR, "%s", e.what());
        }
    }
    return 0;
}

int Device::readStream(SoapySDR::Stream *s, void *const *buffs, const size_t numElems, int &flags, long long &timeNs,
                       const long timeoutUs)
{
    RxStream *r = (RxStream *)s;
    flags = 0;
    std::unique_lock<std::mutex> sl(streams_m_);
    if ((void *)s != (void *)rx_ || !r->active)
        return SOAPY_SDR_STREAM_ERROR;
    InCall in(r->calls);
    sl.unlock();
    const size_t have = r->cur.data.size();
    if (r->off >= have) {
        if (!b_->queue().pop(r->cur, timeoutUs)) {
            if (!b_->failure().empty()) {
                SoapySDR::logf(SOAPY_SDR_ERROR, "%s", b_->failure().c_str());
                return SOAPY_SDR_STREAM_ERROR;
            }
            return SOAPY_SDR_TIMEOUT;
        }
        r->off = 0;
        /* the chunk's time on the device's clock (RxStream's base_ns) */
        const size_t n = r->cur.data.size() / (r->cur.format == 1 ? 2 : 4);
        const long double per = 1e9L * r->rate_den / r->rate_num;
        const long double bound = (long double)r->cur.t_us * 1000 - (long double)(r->cur.first + n) * per;
        if (!r->anchored) {
            r->base_ns = r->applied_ns = bound;
            r->late_ns = 0;
        } else if (r->cur.flags & LR20XX_LINK_GAP) {
            r->base_ns = r->applied_ns = std::max(bound - r->late_ns, r->applied_ns);
        } else {
            r->late_ns += (std::max(bound - r->base_ns, 0.0L) - r->late_ns) / 16;
            if (bound < r->base_ns)
                r->base_ns = bound;
            else
                r->base_ns += std::min(bound - r->base_ns, (long double)n * per * 200e-6L);
            r->applied_ns = r->base_ns >= r->applied_ns ? r->base_ns
                                                        : r->applied_ns - std::min(r->applied_ns - r->base_ns, per * 0.9L);
        }
        r->anchored = true;
        r->cur_ns = r->applied_ns + (long double)r->cur.first * per;
    }
    if (r->cur.flags & LR20XX_LINK_GAP) {
        /* samples were lost ahead of this chunk: said once, then the chunk is read */
        r->cur.flags &= (uint8_t)~LR20XX_LINK_GAP;
        return SOAPY_SDR_OVERFLOW;
    }
    const bool cs8 = r->cur.format == 1;
    const size_t sb = cs8 ? 2 : 4;
    const size_t avail = (r->cur.data.size() - r->off) / sb;
    const size_t n = std::min(numElems, avail);
    const uint8_t *p = r->cur.data.data() + r->off;
    for (size_t i = 0; i < 2 * n; i++) {
        /* one component: int16 little-endian or int8 */
        const int v = cs8 ? (int8_t)p[i] : (int16_t)(p[2 * i] | p[2 * i + 1] << 8);
        if (r->format == SOAPY_SDR_CF32)
            ((float *)buffs[0])[i] = cs8 ? v / 32.0f : v / 32768.0f;
        else if (r->format == SOAPY_SDR_CS16)
            ((int16_t *)buffs[0])[i] = (int16_t)(cs8 ? v * 1024 : v);
        else
            ((int8_t *)buffs[0])[i] = (int8_t)(cs8 ? v : v >> 8);
    }
    timeNs = (long long)(r->cur_ns + (long double)(r->off / sb) * 1e9L * r->rate_den / r->rate_num) + time_offset_;
    r->off += n * sb;
    flags |= SOAPY_SDR_HAS_TIME;
    if (r->off < r->cur.data.size())
        flags |= SOAPY_SDR_MORE_FRAGMENTS;
    return (int)n;
}

int Device::writeStream(SoapySDR::Stream *s, const void *const *buffs, const size_t numElems, int &flags,
                        const long long timeNs, const long)
{
    TxStream *t = (TxStream *)s;
    std::unique_lock<std::mutex> sl(streams_m_);
    if ((void *)s != (void *)tx_ || t->closing)
        return SOAPY_SDR_STREAM_ERROR;
    InCall in(t->calls);
    sl.unlock();
    const bool streams = hello_.caps & LR20XX_LINK_CAP_TX_STREAM;
    if (!streams && t->n + t->buf.size() + numElems > hello_.tx_max_samples) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "LR20xx: a burst of more than %u samples doesn't fit the device",
                       hello_.tx_max_samples);
        if (t->open)
            try {
                b_->tx_abort();
            } catch (...) {
            }
        t->open = false;
        t->buf.clear();
        return SOAPY_SDR_STREAM_ERROR;
    }
    if (!t->open) {
        /* a new burst: the settings now */
        lr20xx_link_tx_begin_t b{};
        b.mode = (uint8_t)tx_mode_;
        {
            std::lock_guard<std::mutex> l(m_);
            b.center_hz = tx_center_;
            b.ppm = ppm_;
        }
        b.rate = (uint32_t)std::lround(tx_rate_);
        b.fdev_hz = (uint32_t)std::lround(tx_fdev_);
        b.power_dbm = (float)tx_power_;
        if (flags & SOAPY_SDR_HAS_TIME) {
            /* the burst's start on the device's clock (1: a time already past, which comes back late) */
            if (!b_->timed()) {
                SoapySDR::logf(SOAPY_SDR_ERROR, "LR20xx: this device can't time a burst (update the bridge)");
                return SOAPY_SDR_NOT_SUPPORTED;
            }
            b.at_us = std::max<long long>(1, std::llround((long double)(timeNs - time_offset_) / 1000));
        }
        t->open = true; /* before TX_BEGIN: a closeStream from now on aborts it */
        try {
            b_->tx_begin(b);
        } catch (const std::exception &e) {
            t->open = false;
            SoapySDR::logf(SOAPY_SDR_ERROR, "%s", e.what());
            return SOAPY_SDR_STREAM_ERROR;
        }
        if (t->closing) { /* closed while it began: its abort may have gone first */
            t->open = false;
            try {
                b_->tx_abort();
            } catch (...) {
            }
            return SOAPY_SDR_STREAM_ERROR;
        }
        t->aborted = false;
        t->mode = tx_mode_;
        t->rate = tx_rate_;
        t->n = 0;
        t->buf.clear();
        t->has_ctx = false;
        t->last_hz = 0;
        t->over = t->counted = 0;
    }
    const bool end = flags & SOAPY_SDR_END_BURST;
    try {
        /* a bridge that streams gets the samples as they come, in pieces so the first is on its way sooner (a
         * timed burst starts on it); otherwise they're gathered for the end */
        const size_t piece = streams ? 2048 : numElems;
        size_t i = 0;
        do {
            /* a burst's first piece small (1 KB of I/Q, what the device prefills): a USB link sends it on its own */
            const size_t to = std::min(numElems, i + (streams && t->n == 0 && i == 0 ? 512 : piece));
            for (; i < to; i++) {
                std::complex<float> v;
                if (t->format == SOAPY_SDR_CF32)
                    v = {((const float *)buffs[0])[2 * i], ((const float *)buffs[0])[2 * i + 1]};
                else if (t->format == SOAPY_SDR_CS16)
                    v = {((const int16_t *)buffs[0])[2 * i] / 32768.0f,
                         ((const int16_t *)buffs[0])[2 * i + 1] / 32768.0f};
                else
                    v = {((const int8_t *)buffs[0])[2 * i] / 128.0f, ((const int8_t *)buffs[0])[2 * i + 1] / 128.0f};
                t->buf.push_back(v);
            }
            if (streams || (end && i == numElems))
                tx_flush(*t, end && i == numElems, i < numElems);
        } while (i < numElems);
    } catch (const std::exception &e) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "%s", e.what());
        try {
            b_->tx_abort();
        } catch (...) {
        }
        t->open = false;
        std::lock_guard<std::mutex> l(t->m);
        t->status.push_back({SOAPY_SDR_STREAM_ERROR, 0});
        t->cv.notify_all();
        return SOAPY_SDR_STREAM_ERROR;
    }
    if (end)
        tx_finish(*t);
    return (int)numElems;
}

/* The widest instantaneous frequency the transmit mode sends right: I/Q's phase path (R3.25), a frequency sample's
 * ±127 counts of fdev / 64, a phase step's ±127 of rate / 128. */
double Device::tx_freq_limit() const
{
    if (tx_mode_ == LR20XX_TX_IQ_POLAR || tx_mode_ == LR20XX_TX_IQ_PHASE)
        return lr20xx_iq_freq_limit((uint32_t)std::lround(tx_rate_));
    if (tx_mode_ == LR20XX_TX_FREQ)
        return 127.0 * tx_fdev_ / 64.0;
    return 127.0 * tx_rate_ / 128.0;
}

/* The samples written so far, converted to the transmit mode's and sent, with the last one converted before as
 * context (a phase to step from); iq_polar keeps a trailing dip back for the sample after it (its phase is drawn
 * across the dip), unless end. */
void Device::tx_flush(TxStream &t, bool end, bool more)
{
    const bool iq = t.mode == LR20XX_TX_IQ_POLAR || t.mode == LR20XX_TX_IQ_PHASE;
    const double rmin = iq ? std::max(polar_rmin_, 0.2) : 0;
    size_t n = t.buf.size();
    if (!end && t.mode == LR20XX_TX_IQ_POLAR && (more || n < 4096)) {
        while (n && std::abs(t.buf[n - 1]) < (float)polar_rmin_)
            n--;
    }
    if (!n)
        return;
    /* past the mode's frequency limit the chip sends something else (I/Q: counted where the magnitude is up,
     * since near zero any waveform's phase steps are large) */
    const double limit = tx_freq_limit();
    for (size_t i = 0; i < n; i++) {
        const std::complex<float> *p = i ? &t.buf[i - 1] : t.has_ctx ? &t.ctx : nullptr;
        if (!p || std::abs(t.buf[i]) < rmin || std::abs(*p) < rmin)
            continue;
        t.counted++;
        if (std::abs(std::arg(t.buf[i] * std::conj(*p))) * t.rate / (2 * M_PI) > limit * 1.02)
            t.over++;
    }
    const size_t c = t.has_ctx ? 1 : 0; /* the context in front, its output dropped */
    std::vector<uint8_t> data;
    if (iq) {
        std::vector<float> f(2 * (n + c));
        if (c) {
            f[0] = t.ctx.real();
            f[1] = t.ctx.imag();
        }
        for (size_t i = 0; i < n; i++) {
            f[2 * (i + c)] = t.buf[i].real();
            f[2 * (i + c) + 1] = t.buf[i].imag();
        }
        std::vector<int8_t> out(2 * (n + c));
        if (t.mode == LR20XX_TX_IQ_POLAR)
            lr20xx_polar_iq8(f.data(), n + c, 1.0f, (float)polar_rmin_, nullptr, out.data());
        else
            lr20xx_phase_iq8(f.data(), n + c, out.data());
        data.assign((const uint8_t *)out.data() + 2 * c, (const uint8_t *)out.data() + 2 * (n + c));
    } else {
        /* FREQ and PHASE_STEP: the input's instantaneous frequency, from its phase step a sample */
        const uint32_t rate = (uint32_t)std::lround(t.rate);
        data.resize(n);
        for (size_t i = 0; i < n; i++) {
            float hz = t.last_hz;
            const std::complex<float> *p = i ? &t.buf[i - 1] : t.has_ctx ? &t.ctx : nullptr;
            if (p)
                hz = std::arg(t.buf[i] * std::conj(*p)) * (float)t.rate / (float)(2 * M_PI);
            else if (n > 1)
                hz = std::arg(t.buf[1] * std::conj(t.buf[0])) * (float)t.rate / (float)(2 * M_PI);
            t.last_hz = hz;
            data[i] = (uint8_t)(t.mode == LR20XX_TX_FREQ ? lr20xx_raw_freq_sample(hz, (float)tx_fdev_)
                                                         : lr20xx_phase_step_sample(hz, rate));
        }
    }
    for (size_t i = 0; i < n; i++)
        if (t.mode != LR20XX_TX_IQ_PHASE || t.buf[i] != std::complex<float>(0, 0)) {
            t.ctx = t.buf[i];
            t.has_ctx = true;
        }
    t.buf.erase(t.buf.begin(), t.buf.begin() + (std::ptrdiff_t)n);
    t.n += n;
    b_->tx_data(data.data(), data.size());
}

/* The burst's end: TX_COMMIT, its outcome queued for readStreamStatus. */
void Device::tx_finish(TxStream &t)
{
    t.open = false;
    if (!warned_tx_limit_ && t.over * 1000 > t.counted && t.counted) {
        warned_tx_limit_ = true;
        SoapySDR::logf(SOAPY_SDR_WARNING, "LR20xx: %.1f%% of the burst is past %s's ±%.0f Hz, which it doesn't send right%s",
                       100.0 * (double)t.over / (double)t.counted, tx_mode_name(t.mode), tx_freq_limit(),
                       t.mode == LR20XX_TX_FREQ ? " (raise tx_fdev)" : "");
    }
    if (!t.n) {
        try {
            b_->tx_abort();
        } catch (...) {
        }
        return;
    }
    int status = SOAPY_SDR_END_BURST;
    long long key_ns = 0;
    t.committing = true;
    try {
        lr20xx_link_tx_result_t r = b_->tx_commit();
        if (r.key_us)
            key_ns = r.key_us * 1000 + time_offset_;
        if (r.late)
            status = SOAPY_SDR_TIME_ERROR;
        else if (r.tx_flags & LR20XX_FIFO_UNDERFLOW)
            status = SOAPY_SDR_UNDERFLOW;
        else if (r.timed_out)
            status = SOAPY_SDR_TIMEOUT;
        else if (r.early)
            status = SOAPY_SDR_STREAM_ERROR;
    } catch (const std::exception &e) {
        if (!t.closing && !t.aborted) /* (aborted on purpose: no news) */
            SoapySDR::logf(SOAPY_SDR_ERROR, "%s", e.what());
        status = SOAPY_SDR_STREAM_ERROR;
    }
    t.committing = false;
    std::lock_guard<std::mutex> l(t.m);
    t.status.push_back({status, key_ns});
    t.cv.notify_all();
}

int Device::readStreamStatus(SoapySDR::Stream *s, size_t &chanMask, int &flags, long long &timeNs,
                             const long timeoutUs)
{
    TxStream *t = (TxStream *)s;
    std::unique_lock<std::mutex> sl(streams_m_);
    if ((void *)s != (void *)tx_)
        return SOAPY_SDR_NOT_SUPPORTED;
    InCall in(t->calls);
    sl.unlock();
    std::unique_lock<std::mutex> l(t->m);
    if (!t->cv.wait_for(l, std::chrono::microseconds(timeoutUs), [t] { return !t->status.empty() || t->closing; }))
        return SOAPY_SDR_TIMEOUT;
    if (t->status.empty())
        return SOAPY_SDR_STREAM_ERROR; /* closed */
    auto st = t->status.front();
    t->status.pop_front();
    chanMask = 1;
    timeNs = st.second;
    flags = st.second ? SOAPY_SDR_HAS_TIME : 0;
    if (st.first == SOAPY_SDR_END_BURST) {
        flags |= SOAPY_SDR_END_BURST; /* the time: when it keyed up (the signal follows by the chip's start-up) */
        return 0;
    }
    return st.first;
}

/* ── Time: the device's clock (the bridge's since its boot), offset by setHardwareTime ── */

bool Device::hasHardwareTime(const std::string &what) const { return what.empty() && b_->timed(); }

long long Device::getHardwareTime(const std::string &what) const
{
    if (!hasHardwareTime(what))
        return SoapySDR::Device::getHardwareTime(what);
    /* read over the link: taken as of halfway through the round trip */
    const auto t0 = std::chrono::steady_clock::now();
    const int64_t us = b_->now_us();
    const auto rtt = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0);
    return us * 1000 + rtt.count() / 2 + time_offset_;
}

void Device::setHardwareTime(const long long timeNs, const std::string &what)
{
    if (!hasHardwareTime(what))
        return SoapySDR::Device::setHardwareTime(timeNs, what);
    time_offset_ += timeNs - getHardwareTime(what);
}

/* ── Gain ── */

std::vector<std::string> Device::listGains(const int dir, const size_t) const
{
    return {dir == SOAPY_SDR_TX ? "PA" : "LNA"};
}

bool Device::hasGainMode(const int dir, const size_t) const { return dir == SOAPY_SDR_RX; }

void Device::setGainMode(const int dir, const size_t, const bool automatic)
{
    if (dir != SOAPY_SDR_RX)
        return;
    std::lock_guard<std::mutex> l(m_);
    gain_set_ = true;
    if (automatic && step_)
        last_step_ = step_;
    step_ = automatic ? 0 : last_step_;
    b_->set_gain(step_);
}

bool Device::getGainMode(const int dir, const size_t) const
{
    std::lock_guard<std::mutex> l(m_);
    return dir == SOAPY_SDR_RX && step_ == 0;
}

void Device::setGain(const int dir, const size_t, const double value)
{
    std::lock_guard<std::mutex> l(m_);
    if (dir == SOAPY_SDR_TX) {
        tx_power_ = std::max(TX_POWER_MIN, std::min(hello_.tx_power_max, value));
        return;
    }
    /* the nearest of the 13 steps */
    int best = 1;
    for (int i = 1; i < 13; i++)
        if (std::fabs(lr20xx_iq_gain_db[i] - value) < std::fabs(lr20xx_iq_gain_db[best - 1] - value))
            best = i + 1;
    step_ = last_step_ = best;
    gain_set_ = true;
    b_->set_gain(step_);
}

void Device::setGain(const int dir, const size_t ch, const std::string &, const double value) { setGain(dir, ch, value); }

double Device::getGain(const int dir, const size_t) const
{
    std::lock_guard<std::mutex> l(m_);
    if (dir == SOAPY_SDR_TX)
        return tx_power_;
    return lr20xx_iq_gain_db[(step_ ? step_ : last_step_) - 1];
}

double Device::getGain(const int dir, const size_t ch, const std::string &) const { return getGain(dir, ch); }

SoapySDR::Range Device::getGainRange(const int dir, const size_t) const
{
    if (dir == SOAPY_SDR_TX)
        return SoapySDR::Range(TX_POWER_MIN, hello_.tx_power_max, 0.5);
    return SoapySDR::Range(lr20xx_iq_gain_db[0], lr20xx_iq_gain_db[12]);
}

SoapySDR::Range Device::getGainRange(const int dir, const size_t ch, const std::string &) const
{
    return getGainRange(dir, ch);
}

/* ── Frequency ── */

void Device::configure_rx()
{
    lr20xx_link_rx_config_t c{};
    c.rate = rate_;
    c.mode = (uint8_t)mode_;
    /* int8 has 6 bits, about 20 dB above the noise (docs/iq.md R2.33): at the int16 default (53.5 dB) or under the
     * AGC the noise alone clips it. Until the user picks a gain, int8 gets 9 dB (step 4, noise at rms ~5). */
    lr20xx_iq_plan_t p{};
    const bool int8 = lr20xx_iq_plan(rate_, (lr20xx_iq_mode_t)mode_, &p) == LR20XX_OK && p.format == LR20XX_IQ_CS8;
    if (!gain_set_)
        step_ = last_step_ = int8 ? 4 : 13;
    else if (int8 && step_ == 0 && !warned_agc8_) {
        warned_agc8_ = true;
        SoapySDR::log(SOAPY_SDR_WARNING, "LR20xx: the AGC clips int8 (above 260 kS/s): a fixed gain near 9 dB keeps it "
                                         "in range (docs/iq.md R2.33)");
    }
    c.gain = (int8_t)step_;
    c.center_hz = center_;
    c.ppm = ppm_;
    plan_ = b_->rx_config(c);
    have_plan_ = true;
    if (rx_ && rx_->active) {
        const uint32_t den = plan_.rate_den ? plan_.rate_den : 1;
        /* the sample count still holds the old rate's samples: the next chunk anchors the time afresh */
        if ((uint64_t)plan_.rate_num * rx_->rate_den != (uint64_t)rx_->rate_num * den)
            rx_->anchored = false;
        rx_->rate_num = plan_.rate_num;
        rx_->rate_den = den;
    }
}

void Device::setFrequency(const int dir, const size_t, const double frequency, const SoapySDR::Kwargs &)
{
    if (frequency < hello_.freq_min || frequency > hello_.freq_max)
        throw std::runtime_error("LR20xx: " + std::to_string(frequency) + " Hz is outside the board's range");
    std::lock_guard<std::mutex> l(m_);
    if (dir == SOAPY_SDR_TX) {
        tx_center_ = frequency;
        return;
    }
    center_ = frequency;
    configure_rx();
}

void Device::setFrequency(const int dir, const size_t ch, const std::string &, const double frequency,
                          const SoapySDR::Kwargs &args)
{
    setFrequency(dir, ch, frequency, args);
}

double Device::getFrequency(const int dir, const size_t) const
{
    std::lock_guard<std::mutex> l(m_);
    return dir == SOAPY_SDR_TX && tx_center_ ? tx_center_ : center_;
}

double Device::getFrequency(const int dir, const size_t ch, const std::string &) const { return getFrequency(dir, ch); }

SoapySDR::RangeList Device::getFrequencyRange(const int, const size_t) const
{
    return {SoapySDR::Range(hello_.freq_min, hello_.freq_max)};
}

SoapySDR::RangeList Device::getFrequencyRange(const int dir, const size_t ch, const std::string &) const
{
    return getFrequencyRange(dir, ch);
}

void Device::setFrequencyCorrection(const int, const size_t, const double value)
{
    std::lock_guard<std::mutex> l(m_);
    ppm_ = value;
    if (have_plan_)
        configure_rx();
}

double Device::getFrequencyCorrection(const int, const size_t) const
{
    std::lock_guard<std::mutex> l(m_);
    return ppm_;
}

/* ── Sample rate ── */

void Device::setSampleRate(const int dir, const size_t, const double rate)
{
    std::lock_guard<std::mutex> l(m_);
    if (dir == SOAPY_SDR_TX) {
        tx_rate_ = rate;
        return;
    }
    /* the converter makes whole S/s (CONV8, and AUTO past FSK16's top) */
    const bool conv = mode_ == LR20XX_IQ_CONV8 || (mode_ == LR20XX_IQ_AUTO && rate > LR20XX_IQ_FSK16_MAX);
    rate_ = conv ? std::round(rate) : rate;
    configure_rx();
}

double Device::getSampleRate(const int dir, const size_t) const
{
    std::lock_guard<std::mutex> l(m_);
    if (dir == SOAPY_SDR_TX)
        return tx_rate_;
    return have_plan_ && plan_.rate_den ? (double)plan_.rate_num / plan_.rate_den : rate_;
}

/* A mode's rates (lr20xx_iq_rates): a list for LORA and FSK16, a range for CONV8. */
/* baud=, for a UART bridge: 0 (the port's own) when not given */
static unsigned baud_arg(const SoapySDR::Kwargs &args)
{
    auto it = args.find("baud");
    return it == args.end() ? 0 : (unsigned)std::stoul(it->second);
}

static std::vector<double> rate_list(int mode)
{
    double r[64];
    int n = lr20xx_iq_rates((lr20xx_iq_mode_t)mode, r, 64, nullptr, nullptr);
    return std::vector<double>(r, r + n);
}

std::vector<double> Device::listSampleRates(const int dir, const size_t) const
{
    if (dir == SOAPY_SDR_TX)
        return std::vector<double>(std::begin(TX_RATES), std::end(TX_RATES));
    std::lock_guard<std::mutex> l(m_);
    if (mode_ == LR20XX_IQ_LORA)
        return rate_list(LR20XX_IQ_LORA);
    std::vector<double> r;
    if (mode_ == LR20XX_IQ_AUTO || mode_ == LR20XX_IQ_FSK16)
        r = rate_list(LR20XX_IQ_FSK16);
    if (mode_ == LR20XX_IQ_AUTO || mode_ == LR20XX_IQ_CONV8)
        for (double v : {288000.0, 384000.0, 480000.0, 576000.0})
            r.push_back(v);
    return r;
}

SoapySDR::RangeList Device::getSampleRateRange(const int dir, const size_t ch) const
{
    SoapySDR::RangeList out;
    if (dir == SOAPY_SDR_TX || mode_ == LR20XX_IQ_LORA || mode_ == LR20XX_IQ_FSK16) {
        for (double v : listSampleRates(dir, ch))
            out.push_back(SoapySDR::Range(v, v));
        return out;
    }
    double lo = 0, hi = 0;
    lr20xx_iq_rates(LR20XX_IQ_CONV8, nullptr, 0, &lo, &hi);
    if (mode_ == LR20XX_IQ_AUTO) {
        for (double v : rate_list(LR20XX_IQ_FSK16))
            out.push_back(SoapySDR::Range(v, v));
        lo = std::max(lo, LR20XX_IQ_FSK16_MAX);
    }
    out.push_back(SoapySDR::Range(lo, hi));
    return out;
}

double Device::getBandwidth(const int dir, const size_t) const
{
    std::lock_guard<std::mutex> l(m_);
    if (dir == SOAPY_SDR_TX)
        return 2 * tx_freq_limit();
    return have_plan_ ? plan_.passband_hz : 0;
}

std::vector<double> Device::listBandwidths(const int dir, const size_t ch) const
{
    double b = getBandwidth(dir, ch);
    return b > 0 ? std::vector<double>{b} : std::vector<double>{};
}

/* ── Sensors, registers ── */

SoapySDR::ArgInfo Device::getSensorInfo(const std::string &key) const
{
    SoapySDR::ArgInfo a;
    a.key = key;
    a.name = "Temperature";
    a.type = SoapySDR::ArgInfo::FLOAT;
    a.units = "C";
    a.description = "The chip's die (GetTemp, VBE); not while receiving";
    return a;
}

std::string Device::readSensor(const std::string &key) const
{
    if (key != "temperature")
        throw std::runtime_error("LR20xx: no sensor " + key);
    std::lock_guard<std::mutex> l(m_);
    return std::to_string(b_->temperature());
}

void Device::writeRegister(const std::string &name, const unsigned addr, const unsigned value)
{
    if (name != "LR20XX")
        throw std::runtime_error("LR20xx: no register interface " + name);
    std::lock_guard<std::mutex> l(m_);
    b_->write_reg(addr, value);
}

unsigned Device::readRegister(const std::string &name, const unsigned addr) const
{
    if (name != "LR20XX")
        throw std::runtime_error("LR20xx: no register interface " + name);
    std::lock_guard<std::mutex> l(m_);
    return b_->read_reg(addr, 1)[0];
}

/* ── Settings ── */

SoapySDR::ArgInfoList Device::getSettingInfo() const
{
    SoapySDR::ArgInfoList l;
    SoapySDR::ArgInfo a;
    a.key = "iq_mode";
    a.name = "Receive mode";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = "auto";
    a.options = {"auto", "lora", "fsk16", "conv8"};
    a.description = "auto: int16 FSK up to 260 kS/s, then int8 through the rate converter (docs/iq.md section 2)";
    l.push_back(a);
    a = {};
    a.key = "allow_untested";
    a.name = "Allow an untested ROM";
    a.type = SoapySDR::ArgInfo::BOOL;
    a.value = "false";
    a.description = "Run the raw I/Q paths on a ROM not in lr20xx_chips.h's table (local backend)";
    l.push_back(a);
    a = {};
    a.key = "tx_mode";
    a.name = "Transmit mode";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = "iq_polar";
    a.options = {"iq_polar", "iq_phase", "freq", "phase_step"};
    a.description = "iq_polar: phase and magnitude; iq_phase: phase at constant power; freq / phase_step: the "
                    "input's instantaneous frequency (its phase derivative)";
    l.push_back(a);
    a = {};
    a.key = "tx_fdev";
    a.name = "FREQ scale";
    a.type = SoapySDR::ArgInfo::FLOAT;
    a.value = "2048";
    a.units = "Hz";
    a.description = "freq mode: a count is fdev / 64 Hz, counts -128..127 (docs/iq.md R3.10)";
    l.push_back(a);
    a = {};
    a.key = "tx_rate";
    a.name = "Transmit rate";
    a.type = SoapySDR::ArgInfo::FLOAT;
    a.value = "48000";
    a.units = "S/s";
    l.push_back(a);
    a = {};
    a.key = "polar_rmin";
    a.name = "Polar minimum magnitude";
    a.type = SoapySDR::ArgInfo::FLOAT;
    a.value = "0.2";
    a.description = "iq_polar: magnitudes under this share of full scale are pushed out to it (docs/iq.md R3.27)";
    l.push_back(a);
    return l;
}

void Device::writeSetting(const std::string &key, const std::string &value)
{
    std::unique_lock<std::mutex> l(m_);
    if (key == "iq_mode") {
        for (int i = 0; i < 4; i++)
            if (value == MODES[i]) {
                mode_ = i;
                if (have_plan_)
                    configure_rx();
                return;
            }
        throw std::runtime_error("LR20xx: iq_mode is auto, lora, fsk16 or conv8");
    } else if (key == "allow_untested") {
        allow_untested_ = truthy(value);
        b_->allow_untested(allow_untested_);
    } else if (key == "tx_mode") {
        for (auto &m : TX_MODES)
            if (value == m.name) {
                tx_mode_ = m.mode;
                return;
            }
        throw std::runtime_error("LR20xx: tx_mode is iq_polar, iq_phase, freq or phase_step");
    } else if (key == "tx_fdev") {
        tx_fdev_ = std::stod(value);
    } else if (key == "tx_rate") {
        tx_rate_ = std::stod(value);
    } else if (key == "polar_rmin") {
        polar_rmin_ = std::stod(value);
    } else {
        throw std::runtime_error("LR20xx: no setting " + key);
    }
}

std::string Device::readSetting(const std::string &key) const
{
    std::lock_guard<std::mutex> l(m_);
    if (key == "iq_mode")
        return MODES[mode_];
    if (key == "allow_untested")
        return allow_untested_ ? "true" : "false";
    if (key == "tx_mode") {
        for (auto &m : TX_MODES)
            if (m.mode == tx_mode_)
                return m.name;
    }
    if (key == "tx_fdev")
        return std::to_string(tx_fdev_);
    if (key == "tx_rate")
        return std::to_string(tx_rate_);
    if (key == "polar_rmin")
        return std::to_string(polar_rmin_);
    throw std::runtime_error("LR20xx: no setting " + key);
}

} // namespace soapy_lr20xx

/* ── Registration ── */

namespace {

SoapySDR::Kwargs result(const lr20xx_link_hello_t &h, const std::string &key, const std::string &where)
{
    SoapySDR::Kwargs r;
    r["driver"] = "lr20xx";
    r[key] = where;
    r["board"] = h.board;
    r["rom"] = std::to_string(h.rom_major) + "." + std::to_string(h.rom_minor);
    if (key != "serial") /* there, serial= is the port */
        r["serial"] = h.serial;
    r["label"] = std::string("LR20xx ") + h.board + " (" + where + ")";
    return r;
}

SoapySDR::KwargsList find_lr20xx(const SoapySDR::Kwargs &args)
{
    SoapySDR::KwargsList out;
    SoapySDR::logf(SOAPY_SDR_DEBUG, "LR20xx find: %s", SoapySDR::KwargsToString(args).c_str());
    auto it = args.find("driver");
    if (it != args.end() && it->second != "lr20xx")
        return out;
    try {
        if (args.count("host")) {
            auto b = soapy_lr20xx::open_link_tcp(args.at("host"), 1.0);
            out.push_back(result(b->hello(), "host", args.at("host")));
        } else if (args.count("serial") && args.at("serial").find('/') != std::string::npos) {
            auto b = soapy_lr20xx::open_link_serial(args.at("serial"), 1.0, soapy_lr20xx::baud_arg(args));
            out.push_back(result(b->hello(), "serial", args.at("serial")));
        } else if (args.count("spidev")) {
            /* listed only where the device is; make says why it won't open */
            if (soapy_lr20xx::have_local() && ::access(args.at("spidev").c_str(), F_OK) == 0) {
                SoapySDR::Kwargs r = args;
                r["driver"] = "lr20xx";
                r["label"] = "LR20xx on " + args.at("spidev");
                out.push_back(r);
            }
        } else {
            const std::string addr = args.count("discover") ? args.at("discover") : "";
            for (auto &f : soapy_lr20xx::discover(0.3, addr))
                out.push_back(result(f.hello, "host", f.host));
        }
    } catch (const std::exception &e) {
        /* a device named outright that doesn't answer is worth saying; discovery finding nothing isn't */
        const bool named = args.count("host") || args.count("serial");
        SoapySDR::logf(named ? SOAPY_SDR_WARNING : SOAPY_SDR_DEBUG, "LR20xx find: %s", e.what());
    }
    return out;
}

SoapySDR::Device *make_lr20xx(const SoapySDR::Kwargs &args)
{
    std::unique_ptr<soapy_lr20xx::Backend> b;
    if (args.count("host"))
        b = soapy_lr20xx::open_link_tcp(args.at("host"));
    else if (args.count("serial") && args.at("serial").find('/') != std::string::npos)
        b = soapy_lr20xx::open_link_serial(args.at("serial"), 3.0, soapy_lr20xx::baud_arg(args));
    else if (args.count("spidev"))
        b = soapy_lr20xx::open_local(args);
    else {
        auto found = soapy_lr20xx::discover(0.3, args.count("discover") ? args.at("discover") : "");
        if (found.empty())
            throw std::runtime_error("LR20xx: no bridge answered; give host=, serial= or spidev=");
        b = soapy_lr20xx::open_link_tcp(found[0].host);
    }
    return new soapy_lr20xx::Device(std::move(b), args);
}

SoapySDR::Registry registration("lr20xx", &find_lr20xx, &make_lr20xx, SOAPY_SDR_ABI_VERSION);

} // namespace
