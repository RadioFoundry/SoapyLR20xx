#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Spencer Fowler
"""The module end to end against tools/fake_bridge: discovery, the probe's calls, a capture (the tone's frequency,
the timestamps, the overflow a loss gives), the sensor and registers, and a transmit burst.

    test_fake_bridge.py --module-dir build/soapy --bridge tools/fake_bridge/fake_bridge.py

Needs SoapySDR's Python bindings; no numpy (array.array buffers). Exits 77 (skipped) without the bindings.
"""
import argparse
import array
import cmath
import math
import os
import subprocess
import sys
import tempfile
import time

ap = argparse.ArgumentParser()
ap.add_argument("--module-dir", required=True)
ap.add_argument("--bridge", required=True)
args = ap.parse_args()
os.environ["SOAPY_SDR_PLUGIN_PATH"] = os.path.abspath(args.module_dir)
try:
    import SoapySDR
    from SoapySDR import SOAPY_SDR_CF32, SOAPY_SDR_END_BURST, SOAPY_SDR_OVERFLOW, SOAPY_SDR_RX, SOAPY_SDR_TX
except ImportError:
    print("no SoapySDR Python bindings: skipped")
    sys.exit(77)

checks = fails = 0


def check(cond, what):
    global checks, fails
    checks += 1
    if not cond:
        fails += 1
        print("FAIL", what)


TONE = 10000.0
dump = tempfile.NamedTemporaryFile(delete=False, suffix=".tx")
dump.close()
bridge = subprocess.Popen([sys.executable, args.bridge, "--port", "0", "--tone", str(TONE), "--gap-every", "20",
                           "--tx-dump", dump.name] + os.environ.get("LR20XX_FAKE_ARGS", "").split(),
                          stdout=subprocess.PIPE, text=True)
try:
    port = int(bridge.stdout.readline().split()[-1])
    host = "127.0.0.1:%d" % port

    # Discovery on loopback (a broadcast may also find the same fake on another interface)
    found = SoapySDR.Device.enumerate({"driver": "lr20xx", "discover": "127.0.0.1"})
    check(any(f["board"] == "fake" and f["rom"] == "1.24" for f in found), "discovery: %s" % list(found))
    found = SoapySDR.Device.enumerate({"driver": "lr20xx", "host": host})
    check(len(found) == 1 and found[0]["host"] == host, "find by host")

    # A string, not a dict: SoapySDR 0.8.1's Python Device() on ARM loses a dict's values (SoapySDR#472, fixed in 0.9.0)
    sdr = SoapySDR.Device("driver=lr20xx,host=" + host)
    info = sdr.getHardwareInfo()
    check(info["board"] == "fake" and info["rom"] == "1.24" and info["rom_known"] == "true", "hardware info")
    check(abs(float(sdr.readSensor("temperature")) - 31.5) < 1e-3, "temperature")
    check(sdr.readRegister("LR20XX", 0xF30028) == 0xF30028, "register read")
    sdr.writeRegister("LR20XX", 0xF30030, 0x3E8)
    check(sdr.readRegister("LR20XX", 0xF30030) == 0x3E8, "register write")

    sdr.setSampleRate(SOAPY_SDR_RX, 0, 250000)
    rate = sdr.getSampleRate(SOAPY_SDR_RX, 0)
    check(abs(rate - 250000) < 1, "rate %f" % rate)
    sdr.setFrequency(SOAPY_SDR_RX, 0, 915e6)
    sdr.setGain(SOAPY_SDR_RX, 0, 30)
    check(sdr.getGain(SOAPY_SDR_RX, 0) == 28 or sdr.getGain(SOAPY_SDR_RX, 0) == 32.5, "gain to a step")
    try:
        sdr.setFrequency(SOAPY_SDR_RX, 0, 2.0e9)
        check(False, "out of range frequency refused")
    except Exception:
        pass

    rx = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    sdr.activateStream(rx)
    buf = array.array("f", [0.0]) * (2 * 1024)
    x = []
    times = []
    overflows = 0
    end = time.time() + 1.5
    while time.time() < end and len(x) < 200000:
        r = sdr.readStream(rx, [buf], 1024, timeoutUs=500000)
        if r.ret == SOAPY_SDR_OVERFLOW:
            overflows += 1
            continue
        check(r.ret > 0, "readStream %d" % r.ret)
        if r.ret <= 0:
            break
        times.append((r.timeNs, r.ret))
        x.extend(complex(buf[2 * i], buf[2 * i + 1]) for i in range(r.ret))
    try:
        sdr.readSensor("temperature")
        check(False, "temperature refused while receiving")
    except Exception:
        pass
    sdr.deactivateStream(rx)
    sdr.closeStream(rx)

    check(len(x) > 50000, "samples %d" % len(x))
    check(overflows >= 1, "a loss reported as an overflow (%d)" % overflows)
    # the tone: the mean phase step a sample
    acc = sum(x[i] * x[i - 1].conjugate() for i in range(1, len(x)))
    f = cmath.phase(acc) * rate / (2 * math.pi)
    check(abs(f - TONE) < 200, "tone %.0f Hz, want %.0f" % (f, TONE))
    check(abs(abs(x[100]) - 2000 / 32768) < 0.01, "level %f" % abs(x[100]))
    # the timestamps rise by the samples between, jumping where samples were lost
    ok = all(t1 >= t0 + n0 * 1e9 / rate - 1 for (t0, n0), (t1, _) in zip(times, times[1:]))
    check(ok, "timestamps rise")
    jumps = sum(1 for (t0, n0), (t1, _) in zip(times, times[1:]) if t1 > t0 + n0 * 1e9 / rate + 1000)
    check(jumps >= 1, "a gap shows in the timestamps (%d)" % jumps)

    # int8 through the converter
    sdr.writeSetting("iq_mode", "conv8")
    sdr.setSampleRate(SOAPY_SDR_RX, 0, 384000)
    check(abs(sdr.getSampleRate(SOAPY_SDR_RX, 0) - 384000) < 1, "conv8 rate")
    try:
        native = sdr.getNativeStreamFormat(SOAPY_SDR_RX, 0).format
    except TypeError:  # SoapySDR 0.8.1's bindings (Ubuntu 22.04) want a fullScale they can't return
        native = None
    if native is not None:
        check(native == "CS8", "conv8 is cs8")
    sdr.writeSetting("iq_mode", "auto")

    # transmit: a tone burst in freq mode, 4800 samples
    sdr.writeSetting("tx_mode", "freq")
    sdr.writeSetting("tx_fdev", "2048")
    sdr.setSampleRate(SOAPY_SDR_TX, 0, 48000)
    sdr.setFrequency(SOAPY_SDR_TX, 0, 915e6)
    # the widest the mode sends right: freq's ±127 counts of fdev / 64, I/Q's phase path (R3.25)
    bw_freq = sdr.getBandwidth(SOAPY_SDR_TX, 0)
    sdr.writeSetting("tx_mode", "iq_polar")
    bw_iq = sdr.getBandwidth(SOAPY_SDR_TX, 0)
    sdr.writeSetting("tx_mode", "freq")
    check(abs(bw_freq - 2 * 127 * float(sdr.readSetting("tx_fdev")) / 64) < 1 and abs(bw_iq - 2 * 0.31 * 48000) < 1,
          "TX bandwidth %.0f / %.0f" % (bw_freq, bw_iq))
    tx = sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32)
    sdr.activateStream(tx)
    n = 4800
    w = 2 * math.pi * 1000 / 48000
    tb = array.array("f", [v for k in range(n) for v in (0.5 * math.cos(w * k), 0.5 * math.sin(w * k))])
    r = sdr.writeStream(tx, [tb], n, SOAPY_SDR_END_BURST, timeoutUs=1000000)
    check(r.ret == n, "writeStream %d" % r.ret)
    st = sdr.readStreamStatus(tx, timeoutUs=3000000)
    check(st.ret == 0 and st.flags & SOAPY_SDR_END_BURST, "burst done: %d %d" % (st.ret, st.flags))
    sdr.deactivateStream(tx)
    sdr.closeStream(tx)
    with open(dump.name, "rb") as fh:
        d = fh.read()
    check(len(d) == 1 + n and d[0] == 1, "TX burst: %d bytes, mode %d" % (len(d), d[0] if d else -1))
    # 1 kHz at 2048 / 64 = 32 Hz a count: 31 counts
    counts = [b - 256 if b > 127 else b for b in d[1:]]
    check(all(abs(c - 31) <= 1 for c in counts[1:]), "FREQ samples %s" % counts[:5])

    # a streamed burst: written in parts, it must go out as the same burst written whole, in each mode (iq_polar's
    # dips under rmin cross the parts' edges)
    def burst(mode, parts):
        sdr.writeSetting("tx_mode", mode)
        tx = sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32)
        sdr.activateStream(tx)
        n = 9000
        w = 2 * math.pi * 1000 / 48000
        x = [cmath.rect(0.5 * (1 + math.cos(2 * math.pi * k / 700)), w * k) for k in range(n)]
        cuts = [0] + [n * i // parts + 37 * i for i in range(1, parts)] + [n]
        for i in range(parts):
            part = x[cuts[i]:cuts[i + 1]]
            b = array.array("f", [v for z in part for v in (z.real, z.imag)])
            r = sdr.writeStream(tx, [b], len(part), SOAPY_SDR_END_BURST if i == parts - 1 else 0, timeoutUs=1000000)
            check(r.ret == len(part), "%s part %d: writeStream %d" % (mode, i, r.ret))
        st = sdr.readStreamStatus(tx, timeoutUs=3000000)
        check(st.ret == 0 and st.flags & SOAPY_SDR_END_BURST, "%s in %d parts: done %d %d" % (mode, parts, st.ret,
                                                                                            st.flags))
        sdr.deactivateStream(tx)
        sdr.closeStream(tx)
        with open(dump.name, "rb") as fh:
            return fh.read()

    for mode in ("iq_polar", "iq_phase", "freq", "phase_step"):
        whole, parts = burst(mode, 1), burst(mode, 7)
        check(len(whole) > 9000 and whole == parts, "%s: written in 7 parts as written whole (%d, %d bytes, first "
              "difference at %s)" % (mode, len(whole), len(parts),
                                     next((i for i in range(min(len(whole), len(parts))) if whole[i] != parts[i]),
                                          None)))
    sdr.writeSetting("tx_mode", "freq")
    del sdr
finally:
    bridge.terminate()
    bridge.wait()
    os.unlink(dump.name)

print("soapy fake bridge: %d checks, %d failures" % (checks, fails))
sys.exit(1 if fails else 0)
