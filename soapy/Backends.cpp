// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/* The module's backends (SoapyLR20xx.hpp): a bridge over TCP or serial, a chip on Linux's spidev, and discovery. */
#include "SoapyLR20xx.hpp"

#include <SoapySDR/Logger.hpp>

#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <map>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <sys/ioctl.h>
#if defined(__APPLE__)
#include <IOKit/serial/ioss.h>
#endif
#include <thread>
#include <unistd.h>

#include "lr20xx_cmd.h"

#ifdef LR20XX_HAL_LINUX
#include <pthread.h>
#include <sched.h>

#include "lr20xx_hal_linux.h"
#include "lr20xx_iq.h"
#include "lr20xx_port.h"
#include "lr20xx_radio.h"
#endif

namespace soapy_lr20xx {

/* ── ChunkQueue ── */

void ChunkQueue::push(Chunk &&c)
{
    {
        std::lock_guard<std::mutex> l(m_);
        if (bytes_ + c.data.size() > max_bytes_) {
            drops_++;
            gap_ = true;
            return;
        }
        if (gap_) {
            c.flags |= LR20XX_LINK_GAP;
            gap_ = false;
        }
        bytes_ += c.data.size();
        q_.push_back(std::move(c));
    }
    cv_.notify_one();
}

bool ChunkQueue::pop(Chunk &out, long timeout_us)
{
    std::unique_lock<std::mutex> l(m_);
    if (!cv_.wait_for(l, std::chrono::microseconds(timeout_us), [this] { return !q_.empty(); }))
        return false;
    out = std::move(q_.front());
    q_.pop_front();
    bytes_ -= out.data.size();
    return true;
}

void ChunkQueue::clear()
{
    std::lock_guard<std::mutex> l(m_);
    q_.clear();
    bytes_ = 0;
    gap_ = false;
}

/* ── The link: TCP or serial ── */

namespace {

double now_s()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

class LinkBackend : public Backend {
public:
    LinkBackend(int fd, std::string where) : fd_(fd), where_(std::move(where))
    {
        lr20xx_link_parser_init(&parser_);
        reader_ = std::thread([this] { read_loop(); });
    }

    ~LinkBackend() override
    {
        /* shutdown wakes a TCP reader; a serial port's notices quit_ at its next poll. Joined before the close, so
         * the fd can't be reused under it. */
        quit_ = true;
        ::shutdown(fd_, SHUT_RDWR);
        if (reader_.joinable())
            reader_.join();
        ::close(fd_);
    }

    lr20xx_link_hello_t hello() override
    {
        /* again if no answer: a serial port can lose the bridge's first bytes after it opens (a USB CDC port under
         * a VM's passthrough lost its first two transfers) */
        std::vector<uint8_t> a;
        for (int tries = 3;; tries--) {
            try {
                a = request(LR20XX_CMD_HELLO, nullptr, 0, 1.0);
                break;
            } catch (const Error &e) {
                if (tries <= 1 || e.code != LR20XX_ERR_TIMEOUT)
                    throw;
            }
        }
        lr20xx_link_hello_t h{};
        if (!lr20xx_link_get_hello(a.data(), a.size(), &h))
            throw Error(-1, "LR20xx: short HELLO answer from " + where_);
        caps_ = h.caps;
        return h;
    }

    lr20xx_link_plan_t rx_config(const lr20xx_link_rx_config_t &c) override
    {
        uint8_t p[LR20XX_LINK_RX_CONFIG_SIZE];
        lr20xx_link_put_rx_config(p, &c);
        auto a = request(LR20XX_CMD_RX_CONFIG, p, sizeof p);
        lr20xx_link_plan_t plan{};
        if (!lr20xx_link_get_plan(a.data(), a.size(), &plan))
            throw Error(-1, "LR20xx: short RX_CONFIG answer");
        return plan;
    }

    void rx_start() override
    {
        q_.clear();
        {
            std::lock_guard<std::mutex> l(m_);
            failure_.clear();
        }
        request(LR20XX_CMD_RX_START, nullptr, 0);
        receiving_ = true;
    }

    void rx_stop() override
    {
        request(LR20XX_CMD_RX_STOP, nullptr, 0);
        receiving_ = false;
    }

    void set_gain(int step) override
    {
        int8_t s = (int8_t)step;
        request(LR20XX_CMD_SET_GAIN, &s, 1);
    }

    ChunkQueue &queue() override { return q_; }

    lr20xx_link_tx_result_t transmit(const lr20xx_link_tx_begin_t &b, const std::vector<uint8_t> &data) override
    {
        uint8_t p[LR20XX_LINK_TX_BEGIN_SIZE];
        lr20xx_link_put_tx_begin(p, &b);
        request(LR20XX_CMD_TX_BEGIN, p, sizeof p);
        try {
            std::vector<uint8_t> f(LR20XX_LINK_HEADER + LR20XX_LINK_MAX_PAYLOAD);
            for (size_t off = 0; off < data.size(); off += LR20XX_LINK_MAX_PAYLOAD) {
                size_t n = std::min<size_t>(LR20XX_LINK_MAX_PAYLOAD, data.size() - off);
                send(f.data(), lr20xx_link_tx_data(f.data(), f.size(), data.data() + off, n));
            }
        } catch (...) {
            try {
                request(LR20XX_CMD_TX_ABORT, nullptr, 0);
            } catch (...) {
            }
            throw;
        }
        /* the answer comes once it has gone out: its length plus a margin */
        const double secs = b.rate ? (double)b.samples / b.rate : 0;
        auto a = request(LR20XX_CMD_TX_COMMIT, nullptr, 0, secs + wait_s(b) + (receiving_ ? 10 : 5));
        lr20xx_link_tx_result_t r{};
        if (!lr20xx_link_get_tx_result(a.data(), a.size(), &r))
            throw Error(-1, "LR20xx: short TX_COMMIT answer");
        return r;
    }

    /* Streamed: TX_BEGIN with 0 samples, TX_DATA as it comes (the bridge keys up once it has enough, and the link's
     * flow control holds this side back while its buffer is full), TX_COMMIT at the end */
    bool streams() const { return caps_ & LR20XX_LINK_CAP_TX_STREAM; }

    void tx_begin(const lr20xx_link_tx_begin_t &b) override
    {
        if (!streams())
            return Backend::tx_begin(b);
        tx_b_ = b;
        tx_b_.samples = 0;
        tx_sent_ = 0;
        uint8_t p[LR20XX_LINK_TX_BEGIN_SIZE];
        lr20xx_link_put_tx_begin(p, &tx_b_);
        request(LR20XX_CMD_TX_BEGIN, p, sizeof p);
    }

    void tx_data(const uint8_t *p, size_t n) override
    {
        if (!streams())
            return Backend::tx_data(p, n);
        std::vector<uint8_t> f(LR20XX_LINK_HEADER + LR20XX_LINK_MAX_PAYLOAD);
        for (size_t off = 0; off < n; off += LR20XX_LINK_MAX_PAYLOAD) {
            size_t k = std::min<size_t>(LR20XX_LINK_MAX_PAYLOAD, n - off);
            send(f.data(), lr20xx_link_tx_data(f.data(), f.size(), p + off, k));
        }
        tx_sent_ += n;
    }

    lr20xx_link_tx_result_t tx_commit() override
    {
        if (!streams())
            return Backend::tx_commit();
        /* the answer comes once it has gone out: at most all of it still to send, plus a margin */
        const double bps = (double)tx_b_.rate * (tx_b_.mode == LR20XX_TX_FREQ || tx_b_.mode == LR20XX_TX_PHASE_STEP ? 1 : 2);
        auto a = request(LR20XX_CMD_TX_COMMIT, nullptr, 0,
                         (bps > 0 ? (double)tx_sent_ / bps : 0) + wait_s(tx_b_) + (receiving_ ? 10 : 5));
        lr20xx_link_tx_result_t r{};
        if (!lr20xx_link_get_tx_result(a.data(), a.size(), &r))
            throw Error(-1, "LR20xx: short TX_COMMIT answer");
        return r;
    }

    void tx_abort() override
    {
        if (!streams())
            return Backend::tx_abort();
        request(LR20XX_CMD_TX_ABORT, nullptr, 0);
    }

    std::vector<uint32_t> read_reg(uint32_t addr, int n) override
    {
        uint8_t p[5];
        lr20xx_link_put_u32(p, addr);
        p[4] = (uint8_t)n;
        auto a = request(LR20XX_CMD_READ_REG, p, sizeof p);
        if (a.size() < 4u * n)
            throw Error(-1, "LR20xx: short READ_REG answer");
        std::vector<uint32_t> w(n);
        for (int i = 0; i < n; i++)
            w[i] = lr20xx_link_u32(a.data() + 4 * i);
        return w;
    }

    void write_reg(uint32_t addr, uint32_t value) override
    {
        uint8_t p[8];
        lr20xx_link_put_u32(p, addr);
        lr20xx_link_put_u32(p + 4, value);
        request(LR20XX_CMD_WRITE_REG, p, sizeof p);
    }

    bool timed() override { return caps_ & LR20XX_LINK_CAP_TIME; }

    int64_t now_us() override
    {
        if (!timed())
            return Backend::now_us();
        auto a = request(LR20XX_CMD_TIME, nullptr, 0);
        if (a.size() < 8)
            throw Error(-1, "LR20xx: short TIME answer");
        return (int64_t)lr20xx_link_u64(a.data());
    }

    /* A timed burst's wait before it starts, s (0: untimed) */
    double wait_s(const lr20xx_link_tx_begin_t &b)
    {
        return b.at_us && timed() ? std::max(0.0, (double)(b.at_us - now_us()) / 1e6) : 0;
    }

    float temperature() override
    {
        auto a = request(LR20XX_CMD_TEMP, nullptr, 0);
        if (a.size() < 4)
            throw Error(-1, "LR20xx: short TEMP answer");
        return lr20xx_link_f32(a.data());
    }

    lr20xx_link_stats_t stats() override
    {
        auto a = request(LR20XX_CMD_STATS, nullptr, 0);
        lr20xx_link_stats_t s{};
        if (!lr20xx_link_get_stats(a.data(), a.size(), &s))
            throw Error(-1, "LR20xx: short STATS answer");
        s.link_drops += q_.drops();
        return s;
    }

    std::string failure() override
    {
        std::lock_guard<std::mutex> l(m_);
        return failure_;
    }

private:
    struct Pending {
        bool done = false;
        int32_t err = 0;
        std::vector<uint8_t> ans;
    };

    void send(const uint8_t *p, size_t n)
    {
        std::lock_guard<std::mutex> l(send_m_);
        while (n) {
            ssize_t w = ::write(fd_, p, n);
            if (w < 0 && errno == EINTR)
                continue;
            if (w <= 0)
                throw Error(-1, "LR20xx: link to " + where_ + " broke: " + std::strerror(errno));
            p += w;
            n -= (size_t)w;
        }
    }

    /* timeout_s < 0: 3 s, or 10 s while receiving, when the answer queues behind the samples already sent and a
     * stalled Wi-Fi link can hold them for seconds (a late answer is dropped by its seq) */
    std::vector<uint8_t> request(uint16_t cmd, const void *args, size_t n, double timeout_s = -1)
    {
        if (timeout_s < 0)
            timeout_s = receiving_ ? 10.0 : 3.0;
        uint8_t f[LR20XX_LINK_HEADER + 64];
        uint16_t seq;
        {
            std::lock_guard<std::mutex> l(m_);
            if (closed_)
                throw Error(-1, "LR20xx: link to " + where_ + " is closed");
            seq = ++seq_;
            pending_[seq] = Pending{};
        }
        size_t len = lr20xx_link_req(f, sizeof f, seq, cmd, args, n);
        try {
            send(f, len);
        } catch (...) {
            std::lock_guard<std::mutex> l(m_);
            pending_.erase(seq);
            throw;
        }
        std::unique_lock<std::mutex> l(m_);
        bool ok = cv_.wait_for(l, std::chrono::duration<double>(timeout_s),
                               [&] { return pending_[seq].done || closed_; });
        Pending p = std::move(pending_[seq]);
        pending_.erase(seq);
        if (!p.done && ok)
            throw Error(-1, "LR20xx: link to " + where_ + " closed");
        if (!p.done)
            throw Error(LR20XX_ERR_TIMEOUT, "LR20xx: no answer from " + where_ + " (command 0x" + hex(cmd) + ")");
        if (p.err) {
            std::string why(p.ans.begin(), p.ans.end());
            lr20xx_link_tx_result_t r{};
            if (cmd == LR20XX_CMD_TX_COMMIT && lr20xx_link_get_tx_result(p.ans.data(), p.ans.size(), &r)) {
                char b[160]; /* the transmission's stats, not text */
                std::snprintf(b, sizeof b, "%d runs, %u bytes in, TX FIFO flags 0x%02x%s%s", (int)r.runs,
                              (unsigned)r.sent, r.tx_flags, r.early ? ", a run ended early" : "",
                              r.timed_out ? ", timed out" : "");
                why = b;
            }
            throw Error(p.err, "LR20xx: command 0x" + hex(cmd) + " refused (" + std::to_string(p.err) + ")" +
                                   (why.empty() ? "" : ": " + why));
        }
        return p.ans;
    }

    static std::string hex(unsigned v)
    {
        char b[8];
        std::snprintf(b, sizeof b, "%04x", v);
        return b;
    }

    void read_loop()
    {
        std::vector<uint8_t> buf(16384);
        while (!quit_) {
            pollfd p{fd_, POLLIN, 0};
            const int r = ::poll(&p, 1, 100);
            if (r < 0 && errno == EINTR)
                continue;
            if (r == 0)
                continue;
            ssize_t n = ::read(fd_, buf.data(), buf.size());
            if (n < 0 && errno == EINTR)
                continue;
            if (n <= 0)
                break;
            size_t off = 0;
            while (off < (size_t)n) {
                off += lr20xx_link_feed(&parser_, buf.data() + off, (size_t)n - off);
                lr20xx_link_frame_t f;
                while (lr20xx_link_next(&parser_, &f))
                    handle(f);
            }
        }
        std::lock_guard<std::mutex> l(m_);
        closed_ = true;
        if (!quit_ && failure_.empty())
            failure_ = "LR20xx: link to " + where_ + " closed";
        cv_.notify_all();
    }

    void handle(const lr20xx_link_frame_t &f)
    {
        if (f.type == LR20XX_LINK_RSP) {
            uint16_t seq;
            int32_t err;
            const uint8_t *a;
            size_t n;
            if (!lr20xx_link_parse_rsp(&f, &seq, &err, &a, &n))
                return;
            std::lock_guard<std::mutex> l(m_);
            auto it = pending_.find(seq);
            if (it == pending_.end())
                return; /* timed out already */
            it->second.done = true;
            it->second.err = err;
            it->second.ans.assign(a, a + n);
            cv_.notify_all();
        } else if (f.type == LR20XX_LINK_RX_DATA) {
            lr20xx_link_rx_hdr_t h;
            if (!lr20xx_link_get_rx_hdr(f.payload, f.len, &h))
                return;
            Chunk c;
            c.first = h.first;
            c.t_us = h.t_us;
            c.format = h.format;
            c.flags = h.flags;
            /* back to the chip's own format (lr20xx_link_pack), which readStream converts from */
            const uint8_t *in = f.payload + LR20XX_LINK_RX_HDR;
            const size_t n = f.len - LR20XX_LINK_RX_HDR;
            if (h.bits == (h.format ? 8 : 16)) {
                c.data.assign(in, in + n);
            } else {
                const size_t values = lr20xx_link_packed_values(n, h.bits);
                unpacked_.resize(values);
                lr20xx_link_unpack(unpacked_.data(), values, in, n, h.bits);
                c.data.resize(values * (h.format ? 1 : 2));
                for (size_t i = 0; i < values; i++) {
                    if (h.format) {
                        c.data[i] = (uint8_t)unpacked_[i];
                    } else {
                        c.data[2 * i] = (uint8_t)unpacked_[i];
                        c.data[2 * i + 1] = (uint8_t)((uint16_t)unpacked_[i] >> 8);
                    }
                }
            }
            q_.push(std::move(c));
        } else if (f.type == LR20XX_LINK_EVENT && f.len >= 6) {
            const uint16_t kind = lr20xx_link_u16(f.payload);
            const int32_t err = (int32_t)lr20xx_link_u32(f.payload + 2);
            std::string text((const char *)f.payload + 6, f.len - 6u);
            if (kind == LR20XX_EVENT_RX_FAILED) {
                SoapySDR::logf(SOAPY_SDR_ERROR, "LR20xx: receiving stopped (%d): %s", err, text.c_str());
                std::lock_guard<std::mutex> l(m_);
                failure_ = text.empty() ? "LR20xx: receiving stopped" : text;
                cv_.notify_all();
            } else {
                SoapySDR::logf(SOAPY_SDR_DEBUG, "LR20xx %s: %s", where_.c_str(), text.c_str());
            }
        }
    }

    int fd_;
    std::string where_;
    uint8_t caps_ = 0;
    std::atomic<bool> receiving_{false};
    uint64_t tx_sent_ = 0; /* a streamed transmission's bytes so far */
    lr20xx_link_parser_t parser_;
    std::thread reader_;
    std::atomic<bool> quit_{false};
    std::mutex m_, send_m_;
    std::condition_variable cv_;
    std::map<uint16_t, Pending> pending_;
    std::vector<int16_t> unpacked_; /* the reader thread's scratch for RX_DATA */
    uint16_t seq_ = 0;
    bool closed_ = false;
    std::string failure_;
    ChunkQueue q_;
};

} // namespace

std::unique_ptr<Backend> open_link_tcp(const std::string &where, double timeout_s)
{
    std::string host = where, port = std::to_string(LR20XX_LINK_TCP_PORT);
    const size_t colon = where.rfind(':');
    if (colon != std::string::npos && where.find(':') == colon) { /* host:port, not a bare IPv6 address */
        host = where.substr(0, colon);
        port = where.substr(colon + 1);
    }
    addrinfo hints{}, *res = nullptr;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res) || !res)
        throw Error(LR20XX_LINK_TCP_PORT, "LR20xx: can't resolve " + host);
    int fd = -1;
    std::string why = "no address";
    for (addrinfo *a = res; a && fd < 0; a = a->ai_next) {
        fd = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (fd < 0)
            continue;
        /* connect with a timeout */
        int fl = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, fl | O_NONBLOCK);
        int r = ::connect(fd, a->ai_addr, a->ai_addrlen);
        if (r < 0 && errno == EINPROGRESS) {
            pollfd p{fd, POLLOUT, 0};
            r = ::poll(&p, 1, (int)(timeout_s * 1000)) == 1 ? 0 : -1;
            int e = 0;
            socklen_t el = sizeof e;
            if (r == 0 && (getsockopt(fd, SOL_SOCKET, SO_ERROR, &e, &el) || e))
                errno = e, r = -1;
            else if (r < 0)
                errno = ETIMEDOUT;
        }
        if (r < 0) {
            why = std::strerror(errno);
            ::close(fd);
            fd = -1;
            continue;
        }
        fcntl(fd, F_SETFL, fl);
    }
    freeaddrinfo(res);
    if (fd < 0)
        throw Error(-1, "LR20xx: can't connect to " + where + ": " + why);
    int one = 1, big = 4 << 20;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &big, sizeof big);
#ifdef SO_NOSIGPIPE
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
    return std::unique_ptr<Backend>(new LinkBackend(fd, host + ":" + port));
}

/* Any rate: Linux's termios has constants for the usual fast ones, macOS sets any rate by IOSSIOSPEED. */
static bool set_baud(int fd, termios &t, unsigned baud)
{
#if defined(__APPLE__)
    if (tcsetattr(fd, TCSANOW, &t))
        return false;
    speed_t s = baud;
    return ioctl(fd, IOSSIOSPEED, &s) == 0;
#else
    static const struct {
        unsigned baud;
        speed_t b;
    } rates[] = {{9600, B9600}, {19200, B19200}, {38400, B38400}, {57600, B57600}, {115200, B115200},
                 {230400, B230400}, {460800, B460800}, {500000, B500000}, {921600, B921600}, {1000000, B1000000},
                 {1500000, B1500000}, {2000000, B2000000}, {2500000, B2500000}, {3000000, B3000000},
                 {4000000, B4000000}};
    for (const auto &r : rates)
        if (r.baud == baud)
            return cfsetspeed(&t, r.b) == 0 && tcsetattr(fd, TCSANOW, &t) == 0;
    return false;
#endif
}

std::unique_ptr<Backend> open_link_serial(const std::string &path, double, unsigned baud)
{
    int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0)
        throw Error(-1, "LR20xx: can't open " + path + ": " + std::strerror(errno));
    termios t{};
    if (tcgetattr(fd, &t) == 0) {
        cfmakeraw(&t);
        t.c_cflag |= CLOCAL | CREAD;
        t.c_cflag &= ~(tcflag_t)(HUPCL | CRTSCTS); /* no reset on close, no flow control */
        t.c_cc[VMIN] = 1;
        t.c_cc[VTIME] = 0;
        if (!baud) {
            cfsetspeed(&t, B115200); /* a USB port ignores it */
            tcsetattr(fd, TCSANOW, &t);
        } else if (!set_baud(fd, t, baud)) {
            ::close(fd);
            throw Error(-1, "LR20xx: " + path + " can't run at " + std::to_string(baud) + " baud");
        }
        tcflush(fd, TCIOFLUSH);
    }
    return std::unique_ptr<Backend>(new LinkBackend(fd, path));
}

/* ── Discovery ── */

/* Where the discovery goes: addr alone, or with addr empty each up IPv4 interface's broadcast address and
 * 255.255.255.255. The last leaves by the default route only, so a bridge on a second interface (a USB NCM link)
 * answers only to its interface's own broadcast. */
static std::vector<in_addr> discovery_targets(const std::string &addr)
{
    std::vector<in_addr> to;
    in_addr a{};
    if (!addr.empty()) {
        if (inet_pton(AF_INET, addr.c_str(), &a) == 1)
            to.push_back(a);
        return to;
    }
    a.s_addr = htonl(INADDR_BROADCAST);
    to.push_back(a);
    ifaddrs *ifs = nullptr;
    if (getifaddrs(&ifs) != 0)
        return to;
    for (ifaddrs *i = ifs; i; i = i->ifa_next) {
        const unsigned want = IFF_UP | IFF_RUNNING | IFF_BROADCAST;
        if (!i->ifa_addr || i->ifa_addr->sa_family != AF_INET || (i->ifa_flags & want) != want ||
            (i->ifa_flags & IFF_LOOPBACK) || !i->ifa_broadaddr || i->ifa_broadaddr->sa_family != AF_INET)
            continue;
        a = ((const sockaddr_in *)i->ifa_broadaddr)->sin_addr;
        if (std::none_of(to.begin(), to.end(), [&](const in_addr &t) { return t.s_addr == a.s_addr; }))
            to.push_back(a);
    }
    freeifaddrs(ifs);
    return to;
}

std::vector<Found> discover(double timeout_s, const std::string &addr)
{
    std::vector<Found> out;
    const std::vector<in_addr> targets = discovery_targets(addr);
    if (targets.empty())
        return out;
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
        return out;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof one);
    static const char ask[] = LR20XX_LINK_DISCOVERY;
    for (const in_addr &t : targets) {
        sockaddr_in to{};
        to.sin_family = AF_INET;
        to.sin_port = htons(LR20XX_LINK_DISCOVERY_PORT);
        to.sin_addr = t;
        ::sendto(fd, ask, sizeof ask - 1, 0, (sockaddr *)&to, sizeof to); /* an interface that can't: skipped */
    }
    const double end = now_s() + timeout_s;
    uint8_t buf[1500];
    for (double left; (left = end - now_s()) > 0;) {
        pollfd p{fd, POLLIN, 0};
        if (::poll(&p, 1, (int)(left * 1000) + 1) != 1)
            break;
        sockaddr_in from{};
        socklen_t fl = sizeof from;
        ssize_t n = ::recvfrom(fd, buf, sizeof buf, 0, (sockaddr *)&from, &fl);
        if (n <= 0)
            continue;
        /* the answer: one RSP frame, seq 0, err 0, the HELLO answer */
        if (n < LR20XX_LINK_HEADER || buf[0] != LR20XX_LINK_MAGIC)
            continue;
        lr20xx_link_frame_t f{buf[1], lr20xx_link_u16(buf + 2), buf + LR20XX_LINK_HEADER};
        const uint8_t *h;
        size_t hn;
        uint16_t seq;
        int32_t err;
        if (f.len + LR20XX_LINK_HEADER > n || !lr20xx_link_parse_rsp(&f, &seq, &err, &h, &hn) || seq || err)
            continue;
        Found fd_{};
        if (!lr20xx_link_get_hello(h, hn, &fd_.hello))
            continue;
        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &from.sin_addr, ip, sizeof ip);
        fd_.host = ip;
        /* one bridge on two interfaces (Wi-Fi and USB) answers twice: the first answer is kept */
        fd_.hello.serial[sizeof fd_.hello.serial - 1] = 0;
        auto same = [&](const Found &f) {
            return fd_.hello.serial[0] ? std::strcmp(f.hello.serial, fd_.hello.serial) == 0 : f.host == fd_.host;
        };
        if (std::none_of(out.begin(), out.end(), same))
            out.push_back(fd_);
    }
    ::close(fd);
    return out;
}

/* ── Local: a chip on spidev (Linux) ── */

#ifdef LR20XX_HAL_LINUX

namespace {

class LocalBackend : public Backend {
public:
    explicit LocalBackend(const SoapySDR::Kwargs &args)
    {
        auto get = [&](const char *k, const std::string &d) {
            auto it = args.find(k);
            return it == args.end() ? d : it->second;
        };
        auto num = [&](const char *k, int d) {
            auto it = args.find(k);
            return it == args.end() ? d : (int)std::strtol(it->second.c_str(), nullptr, 0);
        };
        spidev_ = get("spidev", "/dev/spidev0.0");
        gpiochip_ = get("gpiochip", "/dev/gpiochip0");
        if (!args.count("busy") || !args.count("nreset"))
            throw Error(LR20XX_ERR_INVALID_ARG, "LR20xx local: busy= and nreset= (GPIO line offsets) are needed");
        lr20xx_linux_cfg_t c{};
        c.spidev = spidev_.c_str();
        c.speed_hz = num("speed", 0);
        c.gpiochip = gpiochip_.c_str();
        c.busy = num("busy", -1);
        c.nreset = num("nreset", -1);
        c.irq = num("irq", -1);
        c.pa_en = num("pa_en", -1);
        char err[160] = "";
        const std::string btext = get("board", "generic");
        if (lr20xx_board_parse(btext.c_str(), &board_, err, sizeof err))
            throw Error(LR20XX_ERR_INVALID_ARG, std::string("LR20xx local: board: ") + err);
        lr20xx_hal_t hal{};
        lr20xx_err_t e = lr20xx_hal_linux_open(&c, &hal, err, sizeof err);
        if (e != LR20XX_OK)
            throw Error(e, std::string("LR20xx local: ") + err);
        lr20xx_config_t dc{};
        dc.hal = hal;
        if ((e = lr20xx_open(&dc, &dev_)) != LR20XX_OK)
            throw Error(e, std::string("LR20xx local: the chip didn't answer (") + lr20xx_err_name(e) + ")");
        /* the destructor doesn't run for a throw from here: release the chip and its fds */
        try {
            iq_.reset(new lr20xx_iq_t());
            allow_ = get("allow_untested", "false") == "true";
            init();
            reader_ = std::thread([this] { read_loop(); });
        } catch (...) {
            lr20xx_close(dev_);
            throw;
        }
    }

    ~LocalBackend() override
    {
        {
            std::lock_guard<std::mutex> l(m_);
            quit_ = true;
            streaming_ = false;
        }
        cv_.notify_all();
        if (reader_.joinable())
            reader_.join();
        if (iq_->in_session)
            lr20xx_iq_stop(iq_.get());
        lr20xx_close(dev_);
    }

    lr20xx_link_hello_t hello() override
    {
        std::lock_guard<std::mutex> l(m_);
        lr20xx_link_hello_t h{};
        h.version = LR20XX_LINK_VERSION;
        h.rom_major = iq_->rom_major;
        h.rom_minor = iq_->rom_minor;
        h.rom_known = iq_->rom != nullptr;
        h.caps = LR20XX_LINK_CAP_RX | LR20XX_LINK_CAP_TX | (lr20xx_has_irq(dev_) ? LR20XX_LINK_CAP_IRQ : 0);
        h.tx_max_samples = 1u << 24;
        h.freq_min = board_.b.freq_min;
        h.freq_max = board_.b.freq_max;
        h.tx_power_max = board_.b.tx_power_max;
        std::snprintf(h.board, sizeof h.board, "%s", board_.b.name ? board_.b.name : "generic");
        std::snprintf(h.firmware, sizeof h.firmware, "local");
        std::snprintf(h.serial, sizeof h.serial, "%s", spidev_.c_str());
        return h;
    }

    lr20xx_link_plan_t rx_config(const lr20xx_link_rx_config_t &c) override
    {
        lr20xx_iq_plan_t plan;
        lr20xx_err_t e = lr20xx_iq_plan(c.rate, (lr20xx_iq_mode_t)c.mode, &plan);
        if (e != LR20XX_OK)
            throw Error(e, "LR20xx: no receive mode for that rate");
        std::lock_guard<std::mutex> l(m_);
        cfg_.rate = c.rate;
        cfg_.mode = (lr20xx_iq_mode_t)c.mode;
        cfg_.center_hz = c.center_hz;
        cfg_.ppm = c.ppm;
        cfg_.gain = c.gain;
        if (streaming_)
            check(lr20xx_iq_start(iq_.get(), &cfg_), "start");
        lr20xx_link_plan_t p{};
        p.mode = (uint8_t)plan.mode;
        p.format = (uint8_t)plan.format;
        p.rate_num = plan.rate_num;
        p.rate_den = plan.rate_den;
        p.passband_hz = plan.passband_hz;
        return p;
    }

    void rx_start() override
    {
        q_.clear();
        std::lock_guard<std::mutex> l(m_);
        if (!inited_)
            init();
        failure_.clear();
        delivered_ = 0;
        check(lr20xx_iq_start(iq_.get(), &cfg_), "start");
        streaming_ = true;
        cv_.notify_all();
    }

    void rx_stop() override
    {
        std::lock_guard<std::mutex> l(m_);
        streaming_ = false;
        if (iq_->in_session)
            lr20xx_iq_stop(iq_.get());
    }

    void set_gain(int step) override
    {
        std::lock_guard<std::mutex> l(m_);
        cfg_.gain = step;
        if (inited_)
            check(lr20xx_iq_set_gain(iq_.get(), step), "gain");
    }

    ChunkQueue &queue() override { return q_; }

    bool timed() override { return true; }
    int64_t now_us() override { return lr20xx_now_us(); }

    lr20xx_link_tx_result_t transmit(const lr20xx_link_tx_begin_t &b, const std::vector<uint8_t> &data) override
    {
        /* timed: receiving goes on until 10 ms ahead (the set-up), then the library waits for the time */
        if (b.at_us && b.at_us - lr20xx_now_us() > 10000)
            std::this_thread::sleep_for(std::chrono::microseconds(b.at_us - lr20xx_now_us() - 10000));
        std::lock_guard<std::mutex> l(m_);
        if (!inited_)
            init();
        lr20xx_iq_tx_t t{};
        t.mode = (lr20xx_tx_mode_t)b.mode;
        t.center_hz = b.center_hz;
        t.ppm = b.ppm;
        t.rate = b.rate;
        t.fdev_hz = b.fdev_hz;
        t.power_dbm = b.power_dbm;
        t.samples = b.samples;
        t.at_us = b.at_us;
        struct Src {
            const std::vector<uint8_t> *d;
            size_t off;
        } src{&data, 0};
        auto pull = [](void *ctx, uint8_t *buf, size_t max) -> size_t {
            Src *s = (Src *)ctx;
            size_t n = std::min(max, s->d->size() - s->off);
            std::memcpy(buf, s->d->data() + s->off, n);
            s->off += n;
            return n;
        };
        lr20xx_tx_stream_stats_t st{};
        lr20xx_err_t e = lr20xx_iq_transmit(iq_.get(), &t, pull, &src, &st);
        lr20xx_link_tx_result_t r{};
        r.runs = st.runs;
        r.sent = (uint32_t)st.sent;
        r.tx_flags = st.tx_flags;
        r.early = st.early;
        r.timed_out = st.timed_out;
        r.key_us = st.late ? 0 : st.key_us;
        r.late = st.late;
        if (e != LR20XX_OK && !st.timed_out && !st.early)
            check(e, "transmit");
        return r;
    }

    std::vector<uint32_t> read_reg(uint32_t addr, int n) override
    {
        std::lock_guard<std::mutex> l(m_);
        idle("READ_REG");
        std::vector<uint32_t> w(n);
        check(lr20xx_read_mem(dev_, addr, w.data(), n), "ReadRegMem32");
        return w;
    }

    void write_reg(uint32_t addr, uint32_t value) override
    {
        std::lock_guard<std::mutex> l(m_);
        idle("WRITE_REG");
        uint8_t f[12];
        check(lr20xx_write(dev_, f, lr20xx_cmd_write_reg32(f, addr, value)), "WriteRegMem32");
    }

    float temperature() override
    {
        std::lock_guard<std::mutex> l(m_);
        idle("TEMP");
        uint8_t f[4], a[2];
        check(lr20xx_read(dev_, f, lr20xx_radio_get_temp(f, LR20XX_TEMP_VBE, true, 13), a, 2), "GetTemp");
        return (float)lr20xx_radio_temp_c(a, true);
    }

    lr20xx_link_stats_t stats() override
    {
        std::lock_guard<std::mutex> l(m_);
        lr20xx_link_stats_t s{};
        s.samples = iq_->stats.samples;
        s.losses = iq_->stats.losses;
        s.restarts = iq_->stats.restarts;
        s.errors = iq_->stats.errors;
        s.reads = iq_->stats.reads;
        s.irq_late = iq_->stats.irq_late;
        s.link_drops = q_.drops();
        return s;
    }

    std::string failure() override
    {
        std::lock_guard<std::mutex> l(m_);
        return failure_;
    }

    void allow_untested(bool on) override
    {
        std::lock_guard<std::mutex> l(m_);
        allow_ = on;
        if (!inited_)
            init_quiet();
    }

private:
    void check(lr20xx_err_t e, const char *what)
    {
        if (e != LR20XX_OK)
            throw Error(e, std::string("LR20xx ") + what + ": " +
                               (iq_->error[0] ? iq_->error : lr20xx_last_error(dev_)) + " (" + lr20xx_err_name(e) +
                               ")");
    }

    void idle(const char *what)
    {
        if (streaming_)
            throw Error(LR20XX_ERR_INVALID_STATE, std::string("LR20xx: ") + what + " isn't allowed while receiving");
    }

    void init_quiet() { inited_ = lr20xx_iq_init(iq_.get(), dev_, &board_.b, allow_) == LR20XX_OK; }

    void init()
    {
        lr20xx_err_t e = lr20xx_iq_init(iq_.get(), dev_, &board_.b, allow_);
        inited_ = e == LR20XX_OK;
        if (!inited_ && e != LR20XX_ERR_NOT_SUPPORTED)
            check(e, "init");
        if (!inited_)
            SoapySDR::logf(SOAPY_SDR_WARNING, "LR20xx: %s (allow_untested=true to try it)", iq_->error);
    }

    static void sink(void *ctx, const uint8_t *data, size_t n, unsigned flags, int64_t t_us)
    {
        LocalBackend *b = (LocalBackend *)ctx;
        Chunk c;
        c.first = b->delivered_;
        c.t_us = t_us;
        c.format = (uint8_t)b->iq_->plan.format;
        c.flags = (uint8_t)flags;
        c.data.assign(data, data + n * (size_t)b->iq_->plan.sample_bytes);
        b->delivered_ += n;
        b->q_.push(std::move(c));
    }

    void read_loop()
    {
        /* real time where the user may: the FIFO lasts 2 ms */
        sched_param sp{};
        sp.sched_priority = 50;
        if (int e = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
            SoapySDR::logf(SOAPY_SDR_WARNING,
                           "LR20xx local: no real-time priority for the reader (%s): samples may be lost under load; "
                           "allow it with an rtprio limit (/etc/security/limits.d) or CAP_SYS_NICE",
                           std::strerror(e));
        std::unique_lock<std::mutex> l(m_);
        while (!quit_) {
            if (!streaming_) {
                cv_.wait(l, [this] { return quit_ || streaming_; });
                continue;
            }
            int r = lr20xx_iq_poll(iq_.get(), sink, this, 2);
            if (r < 0) {
                failure_ = std::string("LR20xx: receiving stopped: ") + iq_->error;
                SoapySDR::logf(SOAPY_SDR_ERROR, "%s", failure_.c_str());
                streaming_ = false;
            }
            /* let a control call in between reads */
            l.unlock();
            std::this_thread::yield();
            l.lock();
        }
    }

    std::string spidev_, gpiochip_;
    lr20xx_board_buf_t board_{};
    lr20xx_t *dev_ = nullptr;
    std::unique_ptr<lr20xx_iq_t> iq_;
    lr20xx_iq_cfg_t cfg_{250000, LR20XX_IQ_AUTO, 915e6, 0, 13};
    bool inited_ = false, allow_ = false;
    std::mutex m_;
    std::condition_variable cv_;
    std::thread reader_;
    bool quit_ = false, streaming_ = false;
    uint64_t delivered_ = 0;
    std::string failure_;
    ChunkQueue q_;
};

} // namespace

std::unique_ptr<Backend> open_local(const SoapySDR::Kwargs &args) { return std::unique_ptr<Backend>(new LocalBackend(args)); }
bool have_local() { return true; }

#else

std::unique_ptr<Backend> open_local(const SoapySDR::Kwargs &)
{
    throw Error(-1, "LR20xx: spidev= needs Linux (this build has no local backend)");
}
bool have_local() { return false; }

#endif

} // namespace soapy_lr20xx
