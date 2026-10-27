#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Spencer Fowler
"""A fake LR20xx bridge: speaks proto/include/lr20xx_link.h over TCP like firmware/ does, with no chip behind it.

Receiving streams a tone (--tone Hz off the center) in the planned format at the planned rate, paced in real time,
and drops a run of samples every --gap-every frames as the chip loses them: flagged LR20XX_IQ_GAP, the index not
counting them, the clock (t_us) going on. A transmission is checked for its length, kept to TX_BEGIN's at_us, and
written to --tx-dump. It answers the UDP discovery on port 7374.

    tools/fake_bridge/fake_bridge.py [--port 7373] [--tone 10000]

Python 3 standard library only.
"""
import argparse
import math
import os
import random
import socket
import struct
import sys
import threading
import time

MAGIC = 0xA5
REQ, RSP, RX_DATA, TX_DATA, EVENT = 0x01, 0x02, 0x10, 0x11, 0x12
HELLO, RX_CONFIG, RX_START, RX_STOP, SET_GAIN = 0x0001, 0x0010, 0x0011, 0x0012, 0x0013
TX_BEGIN, TX_COMMIT, TX_ABORT = 0x0020, 0x0021, 0x0022
READ_REG, WRITE_REG, RAW, STATS, TEMP, TIME = 0x0030, 0x0031, 0x0032, 0x0040, 0x0041, 0x0042
ERR_INVALID_ARG, ERR_INVALID_STATE, ERR_NOT_SUPPORTED = 0x102, 0x103, 0x106
MAX_PAYLOAD = 4096
GAP = 0x1
TX_BYTES = {1: 1, 2: 1, 3: 2, 4: 2}  # lr20xx_tx_mode_t: FREQ, PHASE_STEP, IQ_PHASE, IQ_POLAR
TX_MAX = 1 << 20


def pack(values):
    """The link's packing (lr20xx_link_pack): the narrowest width of 4 or more bits that holds every value, each
    value LSB first into a little-endian bit stream."""
    m = 0
    for v in values:
        m |= v if v >= 0 else ~v
    bits = max(4, m.bit_length() + 1)
    acc = have = 0
    out = bytearray()
    for v in values:
        acc |= (v & ((1 << bits) - 1)) << have
        have += bits
        while have >= 8:
            out.append(acc & 0xFF)
            acc >>= 8
            have -= 8
    if have:
        out.append(acc)
    return bits, bytes(out)


def frame(t, payload):
    return struct.pack("<BBH", MAGIC, t, len(payload)) + payload


def hello(args):
    # lr20xx_link_put_hello: 106 bytes
    caps = 0x03 if args.no_tx_stream else 0x1B  # RX, TX (, TX_STREAM, TIME)
    return struct.pack("<HBBBBIddd24s24s24s", 1, 0x01, 0x18, 1, caps, TX_MAX, 150e6, 960e6, 22.0,
                       b"fake", b"fake_bridge 0.1", args.serial.encode())


def fsk_rates():
    """FSK rx_bw code rates: 32 MHz / (mode x decimation), as lr20xx_fsk_rx_rate (every code, listed or not)."""
    return sorted({32e6 / (4 << d) / m for d in range(8) for m in range(1, 9)})


def plan(rate, mode):
    """(mode, format, num, den, passband) like lr20xx_iq_plan, near enough for a fake."""
    if mode == 0:
        mode = 2 if rate <= 260000 else 3
    if mode == 1:
        r = 125000 if abs(rate - 125000) <= abs(rate - 250000) else 250000
        return 1, 0, r, 1, r / 4
    if mode == 2:
        for r in fsk_rates():
            if r >= rate and r <= 260000:
                # 32e6 / (4<<d) / m as a fraction
                for d in range(8):
                    for m in range(1, 9):
                        if abs(32e6 / (4 << d) / m - r) < 1e-6:
                            return 2, 0, int(32e6) // (4 << d), m, 0.37 * r
        return None
    if mode == 3:
        if not 50000 <= rate <= 576000:
            return None
        return 3, 1, int(round(rate)), 1, 0.37 * rate
    return None


class Client:
    def __init__(self, sock, args):
        self.s = sock
        self.args = args
        self.send_lock = threading.Lock()
        self.cfg = (250000.0, 0, 13, 915e6, 0.0)
        self.plan = plan(250000.0, 0)
        self.streaming = False
        self.thread = None
        self.stats = dict(samples=0, losses=0)
        self.regs = {}
        self.tx = None
        self.index = 0  # the stream's sample index: what was delivered since RX_START
        self.gap = False  # the next frame after a pause (a transmission) carries a gap
        self.tx_stop = None  # set: the transmission under way (TX_COMMIT waiting) ends

    def send(self, t, payload):
        with self.send_lock:
            self.s.sendall(frame(t, payload))

    def rsp(self, seq, err=0, ans=b""):
        self.send(RSP, struct.pack("<Hi", seq, err) + ans)

    def stream(self):
        mode, fmt, num, den, _ = self.plan
        rate = num / den
        sb = 4 if fmt == 0 else 2
        per = (MAX_PAYLOAD - 18) // sb
        amp = 2000 if fmt == 0 else 20
        w = 2 * math.pi * self.args.tone / rate
        c = 0  # the chip's samples since this start, lost ones too: its clock
        n_frame = 0
        t0 = time.monotonic()
        while self.streaming:
            n_frame += 1
            flags = GAP if self.gap else 0
            self.gap = False
            if self.args.gap_every and n_frame % self.args.gap_every == 0:
                c += per // 2  # lost: the time goes on, the index doesn't
                flags = GAP
                self.stats["losses"] += 1
            values = [v for k in range(c, c + per)
                      for v in (int(amp * math.cos(w * k)), int(amp * math.sin(w * k)))]
            bits, data = pack(values)
            c += per
            # real time: read once the frame's last sample is in; stamped as a read ending 50-400 us after it
            ahead = t0 + c / rate - time.monotonic()
            if ahead > 0:
                time.sleep(ahead)
            t_us = int((t0 + c / rate + random.uniform(50e-6, 400e-6)) * 1e6)
            hdr = struct.pack("<QqBBB", self.index, t_us, fmt, flags, bits)
            try:
                self.send(RX_DATA, hdr + data)
            except OSError:
                return
            self.index += per
            self.stats["samples"] = self.index

    def stop(self):
        self.streaming = False
        if self.thread:
            self.thread.join()
            self.thread = None

    def request(self, seq, cmd, a):
        if cmd == HELLO:
            return self.rsp(seq, 0, hello(self.args))
        if cmd == RX_CONFIG:
            if len(a) < 26:
                return self.rsp(seq, ERR_INVALID_ARG)
            rate, mode, gain, center, ppm = struct.unpack("<dBbdd", a[:26])
            p = plan(rate, mode)
            if not p or not 150e6 <= center <= 960e6:
                return self.rsp(seq, ERR_NOT_SUPPORTED, b"no mode for that rate or frequency")
            restart = self.streaming
            self.stop()
            self.cfg, self.plan = (rate, mode, gain, center, ppm), p
            if restart:
                self.start()
            return self.rsp(seq, 0, struct.pack("<BBIId", *p))
        if cmd == RX_START:
            self.stop()
            self.index = 0
            self.start()
            return self.rsp(seq)
        if cmd == RX_STOP:
            self.stop()
            return self.rsp(seq)
        if cmd == SET_GAIN:
            return self.rsp(seq)
        if cmd == STATS:
            return self.rsp(seq, 0, struct.pack("<Q7I", self.stats["samples"], self.stats["losses"], 0, 0, 0, 0, 0,
                                                0))
        if cmd == TEMP:
            if self.streaming:
                return self.rsp(seq, ERR_INVALID_STATE, b"not while receiving")
            return self.rsp(seq, 0, struct.pack("<f", 31.5))
        if cmd == READ_REG:
            if self.streaming:
                return self.rsp(seq, ERR_INVALID_STATE, b"not while receiving")
            addr, n = struct.unpack("<IB", a[:5])
            return self.rsp(seq, 0, b"".join(struct.pack("<I", self.regs.get(addr + 4 * i, addr + 4 * i))
                                             for i in range(n)))
        if cmd == WRITE_REG:
            addr, v = struct.unpack("<II", a[:8])
            self.regs[addr] = v
            return self.rsp(seq)
        if cmd == TX_BEGIN:
            mode, center, ppm, rate, fdev, power, n = struct.unpack("<BddIIfQ", a[:37])
            at = struct.unpack("<q", a[37:45])[0] if len(a) >= 45 and not self.args.no_tx_stream else 0
            if mode not in TX_BYTES or n > TX_MAX or (n == 0 and self.args.no_tx_stream):
                return self.rsp(seq, ERR_INVALID_ARG)
            self.tx = dict(mode=mode, rate=rate, n=n, at=at, data=bytearray())
            return self.rsp(seq)
        if cmd == TX_ABORT:
            self.tx = None
            if self.tx_stop:
                self.tx_stop.set()
            return self.rsp(seq)
        if cmd == TX_COMMIT:
            tx, self.tx = self.tx, None
            if not tx:
                return self.rsp(seq, ERR_INVALID_STATE, b"no TX_BEGIN")
            if tx["n"] == 0:  # streamed: as long as it came
                tx["n"] = len(tx["data"]) // TX_BYTES[tx["mode"]]
            want = tx["n"] * TX_BYTES[tx["mode"]]
            if len(tx["data"]) != want or not want:
                return self.rsp(seq, ERR_INVALID_ARG, b"%d bytes, not %d" % (len(tx["data"]), want))
            # answered when it has gone out, as the bridge does: the link goes on meanwhile (TX_ABORT, TIME, samples)
            self.tx_stop = threading.Event()
            threading.Thread(target=self.transmit, args=(seq, tx, self.tx_stop), daemon=True).start()
            return None
        if cmd == TIME and not self.args.no_tx_stream:
            return self.rsp(seq, 0, struct.pack("<q", int(time.monotonic() * 1e6)))
        return self.rsp(seq, ERR_NOT_SUPPORTED)

    def transmit(self, seq, tx, stop):
        def now():
            return int(time.monotonic() * 1e6)
        # late if the data came after its time (LR20XX_TX_LATE_US), decided now: a sleep on a busy runner overshoots
        late = int(bool(tx["at"]) and now() > tx["at"] + 100)
        # timed: receiving goes on until the set-up's time ahead (5 ms here); an abort before then leaves it alone
        if tx["at"] and not late and stop.wait(max(0.0, (tx["at"] - 5000 - now()) / 1e6)):
            return self.rsp(seq, ERR_INVALID_STATE, b"aborted")
        restart = self.streaming
        self.stop()  # half duplex
        key = now()
        if tx["at"]:
            if tx["at"] > key:
                time.sleep((tx["at"] - key) / 1e6)
            key = 0 if late else now()
        aborted = not late and stop.wait(min(1.0, tx["n"] / tx["rate"]))
        if aborted:
            if restart:
                self.gap = True
                self.start()
            return self.rsp(seq, ERR_INVALID_STATE, b"aborted")
        if self.args.tx_dump:
            with open(self.args.tx_dump, "wb") as f:
                f.write(bytes([tx["mode"]]) + bytes(tx["data"]))
        if restart:
            self.gap = True
            self.start()
        runs = 0 if late else -(-tx["n"] // 65535)
        sent = 0 if late else len(tx["data"])
        return self.rsp(seq, 0, struct.pack("<iIBBBqB", runs, sent, 0, 0, 0, key, late))

    def start(self):
        self.streaming = True
        self.thread = threading.Thread(target=self.stream, daemon=True)
        self.thread.start()

    def run(self):
        buf = b""
        try:
            while True:
                d = self.s.recv(65536)
                if not d:
                    break
                buf += d
                while len(buf) >= 4:
                    if buf[0] != MAGIC:
                        buf = buf[1:]
                        continue
                    t, n = buf[1], struct.unpack("<H", buf[2:4])[0]
                    if len(buf) < 4 + n:
                        break
                    p, buf = buf[4:4 + n], buf[4 + n:]
                    if t == REQ:
                        seq, cmd = struct.unpack("<HH", p[:4])
                        self.request(seq, cmd, p[4:])
                    elif t == TX_DATA and self.tx is not None:
                        self.tx["data"] += p
        except OSError:
            pass
        finally:
            if self.tx_stop:
                self.tx_stop.set()
            self.stop()
            self.s.close()


def discovery(args):
    u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    u.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        u.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    try:
        u.bind(("", 7374))
    except OSError as e:
        print("fake_bridge: no discovery (%s)" % e, file=sys.stderr)
        return
    while True:
        d, addr = u.recvfrom(64)
        if d.startswith(b"LR20XX?"):
            u.sendto(frame(RSP, struct.pack("<Hi", 0, 0) + hello(args)), addr)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=7373, help="TCP port (0: any; the port is printed)")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--tone", type=float, default=10000.0, help="the tone's offset from the center, Hz")
    ap.add_argument("--gap-every", type=int, default=40, help="a loss every this many frames (0: none)")
    ap.add_argument("--serial", default="FAKE0001", help="the serial HELLO and discovery give")
    ap.add_argument("--tx-dump", help="write each transmission here (its mode byte, then its samples)")
    ap.add_argument("--no-tx-stream", action="store_true", help="as a bridge before streamed transmissions (and the clock)")
    args = ap.parse_args()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.port))
    srv.listen(1)
    print("fake_bridge: port %d" % srv.getsockname()[1], flush=True)
    threading.Thread(target=discovery, args=(args,), daemon=True).start()
    while True:
        c, _ = srv.accept()
        c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        threading.Thread(target=Client(c, args).run, daemon=True).start()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        os._exit(0)
