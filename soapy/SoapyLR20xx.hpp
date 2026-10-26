// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * The SoapySDR module (driver=lr20xx): one Device over two backends. The link backend talks to a bridge
 * (firmware/) over TCP or a USB serial port, in proto/include/lr20xx_link.h's frames; the local backend (Linux
 * only) drives a chip on spidev itself, through lib/'s lr20xx_iq.h. README.md lists the arguments and settings.
 */
#pragma once

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Types.hpp>

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "lr20xx_iq.h"
#include "lr20xx_link.h"

namespace soapy_lr20xx {

/* A command the device refused: its lr20xx_err_t and why. */
struct Error : std::runtime_error {
    int code;
    Error(int code, const std::string &what) : std::runtime_error(what), code(code) {}
};

/* Samples as they came from the chip: in the plan's format, the first one's index in the stream. */
struct Chunk {
    uint64_t first = 0;
    int64_t t_us = 0;
    uint8_t format = 0; /* lr20xx_iq_format_t: 0 cs16, 1 cs8 */
    uint8_t flags = 0;  /* LR20XX_IQ_GAP */
    std::vector<uint8_t> data;
};

/* What a reader thread hands the stream: chunks up to a byte budget; past it new ones are dropped and the next one
 * kept carries a gap. */
class ChunkQueue {
public:
    explicit ChunkQueue(size_t max_bytes = 8u << 20) : max_bytes_(max_bytes) {}
    void push(Chunk &&c);
    /* The next chunk, waiting up to timeout_us; false if none came. */
    bool pop(Chunk &out, long timeout_us);
    void clear();
    uint32_t drops() const { return drops_; }

private:
    std::mutex m_;
    std::condition_variable cv_;
    std::deque<Chunk> q_;
    size_t bytes_ = 0, max_bytes_;
    bool gap_ = false;
    uint32_t drops_ = 0;
};

/* What every backend does. Each throws Error on a refusal or a broken connection. */
class Backend {
public:
    virtual ~Backend() = default;
    virtual lr20xx_link_hello_t hello() = 0;
    virtual lr20xx_link_plan_t rx_config(const lr20xx_link_rx_config_t &c) = 0;
    virtual void rx_start() = 0;
    virtual void rx_stop() = 0;
    virtual void set_gain(int step) = 0;
    /* The samples rx_start started. */
    virtual ChunkQueue &queue() = 0;
    virtual lr20xx_link_tx_result_t transmit(const lr20xx_link_tx_begin_t &b, const std::vector<uint8_t> &data) = 0;
    virtual std::vector<uint32_t> read_reg(uint32_t addr, int n) = 0;
    virtual void write_reg(uint32_t addr, uint32_t value) = 0;
    virtual float temperature() = 0;
    virtual lr20xx_link_stats_t stats() = 0;
    /* Set when the session failed on its own (the device's EVENT, or the local reader); empty otherwise. */
    virtual std::string failure() = 0;
    /* The local backend's allow_untested setting (re-checks the ROM); the link backend's device decides. */
    virtual void allow_untested(bool) {}
};

/* A bridge over TCP ("host[:port]") or a serial port. */
std::unique_ptr<Backend> open_link_tcp(const std::string &host, double timeout_s = 3.0);
/* baud 0: leave the port's rate alone (a USB port ignores it); a UART needs the bridge's (CONFIG_LR20XX_SERIAL_BAUD). */
std::unique_ptr<Backend> open_link_serial(const std::string &path, double timeout_s = 3.0, unsigned baud = 0);
/* A chip on this machine's spidev and GPIO (Linux; throws elsewhere). */
std::unique_ptr<Backend> open_local(const SoapySDR::Kwargs &args);
bool have_local();

/* Bridges on the local network that answer the UDP discovery within timeout_s, with their address, one per serial.
 * addr empty: a broadcast on every interface; else that address alone. */
struct Found {
    std::string host;
    lr20xx_link_hello_t hello;
};
std::vector<Found> discover(double timeout_s, const std::string &addr = "");

class Device : public SoapySDR::Device {
public:
    Device(std::unique_ptr<Backend> b, const SoapySDR::Kwargs &args);
    ~Device() override;

    std::string getDriverKey() const override { return "lr20xx"; }
    std::string getHardwareKey() const override;
    SoapySDR::Kwargs getHardwareInfo() const override;

    size_t getNumChannels(const int) const override { return 1; }
    bool getFullDuplex(const int, const size_t) const override { return false; }

    std::vector<std::string> getStreamFormats(const int dir, const size_t ch) const override;
    std::string getNativeStreamFormat(const int dir, const size_t ch, double &fullScale) const override;
    SoapySDR::ArgInfoList getStreamArgsInfo(const int, const size_t) const override { return {}; }
    SoapySDR::Stream *setupStream(const int dir, const std::string &format, const std::vector<size_t> &channels,
                                  const SoapySDR::Kwargs &args) override;
    void closeStream(SoapySDR::Stream *s) override;
    size_t getStreamMTU(SoapySDR::Stream *s) const override;
    int activateStream(SoapySDR::Stream *s, const int flags, const long long timeNs, const size_t numElems) override;
    int deactivateStream(SoapySDR::Stream *s, const int flags, const long long timeNs) override;
    int readStream(SoapySDR::Stream *s, void *const *buffs, const size_t numElems, int &flags, long long &timeNs,
                   const long timeoutUs) override;
    int writeStream(SoapySDR::Stream *s, const void *const *buffs, const size_t numElems, int &flags,
                    const long long timeNs, const long timeoutUs) override;
    int readStreamStatus(SoapySDR::Stream *s, size_t &chanMask, int &flags, long long &timeNs,
                         const long timeoutUs) override;

    std::vector<std::string> listAntennas(const int, const size_t) const override { return {"RX/TX"}; }
    std::string getAntenna(const int, const size_t) const override { return "RX/TX"; }
    void setAntenna(const int, const size_t, const std::string &) override {}

    std::vector<std::string> listGains(const int dir, const size_t) const override;
    bool hasGainMode(const int dir, const size_t) const override;
    void setGainMode(const int dir, const size_t, const bool automatic) override;
    bool getGainMode(const int dir, const size_t) const override;
    void setGain(const int dir, const size_t ch, const double value) override;
    void setGain(const int dir, const size_t ch, const std::string &name, const double value) override;
    double getGain(const int dir, const size_t ch) const override;
    double getGain(const int dir, const size_t ch, const std::string &name) const override;
    SoapySDR::Range getGainRange(const int dir, const size_t ch) const override;
    SoapySDR::Range getGainRange(const int dir, const size_t ch, const std::string &name) const override;

    void setFrequency(const int dir, const size_t ch, const double frequency, const SoapySDR::Kwargs &args) override;
    void setFrequency(const int dir, const size_t ch, const std::string &name, const double frequency,
                      const SoapySDR::Kwargs &args) override;
    double getFrequency(const int dir, const size_t ch) const override;
    double getFrequency(const int dir, const size_t ch, const std::string &name) const override;
    std::vector<std::string> listFrequencies(const int, const size_t) const override { return {"RF"}; }
    SoapySDR::RangeList getFrequencyRange(const int dir, const size_t ch) const override;
    SoapySDR::RangeList getFrequencyRange(const int dir, const size_t ch, const std::string &name) const override;
    bool hasFrequencyCorrection(const int, const size_t) const override { return true; }
    void setFrequencyCorrection(const int dir, const size_t ch, const double value) override;
    double getFrequencyCorrection(const int dir, const size_t ch) const override;

    void setSampleRate(const int dir, const size_t ch, const double rate) override;
    double getSampleRate(const int dir, const size_t ch) const override;
    std::vector<double> listSampleRates(const int dir, const size_t ch) const override;
    SoapySDR::RangeList getSampleRateRange(const int dir, const size_t ch) const override;
    double getBandwidth(const int dir, const size_t ch) const override;
    std::vector<double> listBandwidths(const int dir, const size_t ch) const override;

    std::vector<std::string> listSensors() const override { return {"temperature"}; }
    SoapySDR::ArgInfo getSensorInfo(const std::string &key) const override;
    std::string readSensor(const std::string &key) const override;

    std::vector<std::string> listRegisterInterfaces() const override { return {"LR20XX"}; }
    void writeRegister(const std::string &name, const unsigned addr, const unsigned value) override;
    unsigned readRegister(const std::string &name, const unsigned addr) const override;
    void writeRegister(const unsigned addr, const unsigned value) override { writeRegister("LR20XX", addr, value); }
    unsigned readRegister(const unsigned addr) const override { return readRegister("LR20XX", addr); }

    SoapySDR::ArgInfoList getSettingInfo() const override;
    void writeSetting(const std::string &key, const std::string &value) override;
    std::string readSetting(const std::string &key) const override;

private:
    struct RxStream;
    struct TxStream;

    void configure_rx(); /* RX_CONFIG with the current settings */
    void send_tx(TxStream &t);

    std::unique_ptr<Backend> b_;
    lr20xx_link_hello_t hello_{};
    mutable std::mutex m_; /* the control calls */
    double center_ = 0, ppm_ = 0, rate_ = 250000;
    int mode_ = 0;   /* lr20xx_iq_mode_t */
    int step_ = 13;  /* 1-13, 0 the AGC */
    int last_step_ = 13;
    bool gain_set_ = false;     /* the user chose a gain or the AGC: no default by format then (configure_rx) */
    bool warned_agc8_ = false;
    bool warned_tx_limit_ = false;
    double tx_freq_limit() const;
    lr20xx_link_plan_t plan_{};
    bool have_plan_ = false;
    bool allow_untested_ = false;
    /* transmit */
    int tx_mode_ = 4; /* lr20xx_tx_mode_t IQ_POLAR */
    double tx_rate_ = 48000, tx_fdev_ = 2048, tx_power_ = 0, tx_center_ = 0, polar_rmin_ = 0.2;
    RxStream *rx_ = nullptr;
    TxStream *tx_ = nullptr;
};

} // namespace soapy_lr20xx
