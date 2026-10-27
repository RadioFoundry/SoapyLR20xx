// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! Frames and payloads, byte for byte as proto/include/lr20xx_link.h lays them out (all little-endian).

pub const MAGIC: u8 = 0xA5;
pub const VERSION: u16 = 1;
pub const HEADER: usize = 4;
pub const MAX_PAYLOAD: usize = 4096;
pub const TCP_PORT: u16 = 7373;
pub const DISCOVERY_PORT: u16 = 7374;
/// What a host sends to the discovery port.
pub const DISCOVERY: &str = "LR20XX?";

/// Frame types.
pub mod frame_type {
    pub const REQ: u8 = 0x01;
    pub const RSP: u8 = 0x02;
    pub const RX_DATA: u8 = 0x10;
    pub const TX_DATA: u8 = 0x11;
    pub const EVENT: u8 = 0x12;
}

/// Commands (a REQ's `cmd`).
pub mod cmd {
    pub const HELLO: u16 = 0x0001;
    pub const RX_CONFIG: u16 = 0x0010;
    pub const RX_START: u16 = 0x0011;
    pub const RX_STOP: u16 = 0x0012;
    pub const SET_GAIN: u16 = 0x0013;
    pub const TX_BEGIN: u16 = 0x0020;
    pub const TX_COMMIT: u16 = 0x0021;
    pub const TX_ABORT: u16 = 0x0022;
    pub const READ_REG: u16 = 0x0030;
    pub const WRITE_REG: u16 = 0x0031;
    pub const RAW: u16 = 0x0032;
    pub const STATS: u16 = 0x0040;
    pub const TEMP: u16 = 0x0041;
}

pub const CAP_RX: u8 = 0x01;
pub const CAP_TX: u8 = 0x02;
pub const CAP_IRQ: u8 = 0x04;
/// TX_BEGIN with 0 samples streams until TX_COMMIT; a burst of any length.
pub const CAP_TX_STREAM: u8 = 0x08;
/// RX_DATA flag: samples were lost ahead of these.
pub const FLAG_GAP: u8 = 0x01;
pub const EVENT_RX_FAILED: u16 = 1;
pub const EVENT_LOG: u16 = 2;

pub const HELLO_SIZE: usize = 106;
pub const RX_CONFIG_SIZE: usize = 26;
pub const PLAN_SIZE: usize = 18;
pub const RX_HDR_SIZE: usize = 19;
pub const TX_BEGIN_SIZE: usize = 37;
pub const TX_RESULT_SIZE: usize = 11;
pub const STATS_SIZE: usize = 36;

fn u16_at(p: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([p[o], p[o + 1]])
}
fn u32_at(p: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(p[o..o + 4].try_into().unwrap())
}
fn u64_at(p: &[u8], o: usize) -> u64 {
    u64::from_le_bytes(p[o..o + 8].try_into().unwrap())
}
fn f32_at(p: &[u8], o: usize) -> f32 {
    f32::from_bits(u32_at(p, o))
}
fn f64_at(p: &[u8], o: usize) -> f64 {
    f64::from_bits(u64_at(p, o))
}

/// A NUL-padded text field of n bytes (at most n - 1 characters kept).
fn put_str(out: &mut Vec<u8>, s: &str, n: usize) {
    let b = s.as_bytes();
    let k = b.len().min(n - 1);
    out.extend_from_slice(&b[..k]);
    out.resize(out.len() + n - k, 0);
}
fn get_str(p: &[u8]) -> String {
    let end = p.iter().position(|&c| c == 0).unwrap_or(p.len() - 1).min(p.len() - 1);
    String::from_utf8_lossy(&p[..end]).into_owned()
}

/// A payload struct's packed form.
pub trait Payload: Sized {
    const SIZE: usize;
    fn put(&self, out: &mut Vec<u8>);
    fn get(p: &[u8]) -> Option<Self>;
    fn to_bytes(&self) -> Vec<u8> {
        let mut v = Vec::with_capacity(Self::SIZE);
        self.put(&mut v);
        v
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Hello {
    pub version: u16,
    pub rom_major: u8,
    pub rom_minor: u8,
    pub rom_known: u8,
    pub caps: u8,
    pub tx_max_samples: u32,
    pub freq_min: f64,
    pub freq_max: f64,
    pub tx_power_max: f64,
    pub board: String,
    pub firmware: String,
    pub serial: String,
}

impl Payload for Hello {
    const SIZE: usize = HELLO_SIZE;
    fn put(&self, o: &mut Vec<u8>) {
        o.extend_from_slice(&self.version.to_le_bytes());
        o.extend_from_slice(&[self.rom_major, self.rom_minor, self.rom_known, self.caps]);
        o.extend_from_slice(&self.tx_max_samples.to_le_bytes());
        o.extend_from_slice(&self.freq_min.to_le_bytes());
        o.extend_from_slice(&self.freq_max.to_le_bytes());
        o.extend_from_slice(&self.tx_power_max.to_le_bytes());
        put_str(o, &self.board, 24);
        put_str(o, &self.firmware, 24);
        put_str(o, &self.serial, 24);
    }
    fn get(p: &[u8]) -> Option<Self> {
        if p.len() < Self::SIZE {
            return None;
        }
        Some(Hello {
            version: u16_at(p, 0),
            rom_major: p[2],
            rom_minor: p[3],
            rom_known: p[4],
            caps: p[5],
            tx_max_samples: u32_at(p, 6),
            freq_min: f64_at(p, 10),
            freq_max: f64_at(p, 18),
            tx_power_max: f64_at(p, 26),
            board: get_str(&p[34..58]),
            firmware: get_str(&p[58..82]),
            serial: get_str(&p[82..106]),
        })
    }
}

/// RX_CONFIG's arguments. `mode` is an lr20xx_iq_mode_t (0 auto, 1 LoRa, 2 FSK16, 3 CONV8).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RxConfig {
    pub rate: f64,
    pub mode: u8,
    pub gain: i8,
    pub center_hz: f64,
    pub ppm: f64,
}

impl Payload for RxConfig {
    const SIZE: usize = RX_CONFIG_SIZE;
    fn put(&self, o: &mut Vec<u8>) {
        o.extend_from_slice(&self.rate.to_le_bytes());
        o.push(self.mode);
        o.push(self.gain as u8);
        o.extend_from_slice(&self.center_hz.to_le_bytes());
        o.extend_from_slice(&self.ppm.to_le_bytes());
    }
    fn get(p: &[u8]) -> Option<Self> {
        if p.len() < Self::SIZE {
            return None;
        }
        Some(RxConfig {
            rate: f64_at(p, 0),
            mode: p[8],
            gain: p[9] as i8,
            center_hz: f64_at(p, 10),
            ppm: f64_at(p, 18),
        })
    }
}

/// RX_CONFIG's answer. `format` 0 is cs16, 1 cs8.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Plan {
    pub mode: u8,
    pub format: u8,
    pub rate_num: u32,
    pub rate_den: u32,
    pub passband_hz: f64,
}

impl Plan {
    pub fn rate(&self) -> f64 {
        self.rate_num as f64 / self.rate_den.max(1) as f64
    }
    /// Bytes a sample: 4 for cs16, 2 for cs8.
    pub fn sample_bytes(&self) -> usize {
        if self.format == 0 {
            4
        } else {
            2
        }
    }
}

impl Payload for Plan {
    const SIZE: usize = PLAN_SIZE;
    fn put(&self, o: &mut Vec<u8>) {
        o.extend_from_slice(&[self.mode, self.format]);
        o.extend_from_slice(&self.rate_num.to_le_bytes());
        o.extend_from_slice(&self.rate_den.to_le_bytes());
        o.extend_from_slice(&self.passband_hz.to_le_bytes());
    }
    fn get(p: &[u8]) -> Option<Self> {
        if p.len() < Self::SIZE {
            return None;
        }
        Some(Plan {
            mode: p[0],
            format: p[1],
            rate_num: u32_at(p, 2),
            rate_den: u32_at(p, 6),
            passband_hz: f64_at(p, 10),
        })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RxHeader {
    pub first: u64,
    pub t_us: i64,
    /// 0 int16, 1 int8 (lr20xx_iq_format_t).
    pub format: u8,
    pub flags: u8,
    /// Each value's width on the wire, [`BITS_MIN`] to 16 (8 for int8): see [`pack`].
    pub bits: u8,
}

impl Payload for RxHeader {
    const SIZE: usize = RX_HDR_SIZE;
    fn put(&self, o: &mut Vec<u8>) {
        o.extend_from_slice(&self.first.to_le_bytes());
        o.extend_from_slice(&self.t_us.to_le_bytes());
        o.extend_from_slice(&[self.format, self.flags, self.bits]);
    }
    fn get(p: &[u8]) -> Option<Self> {
        if p.len() < Self::SIZE {
            return None;
        }
        let (format, bits) = (p[16], p[18]);
        if bits < BITS_MIN || bits > if format != 0 { 8 } else { 16 } {
            return None;
        }
        Some(RxHeader { first: u64_at(p, 0), t_us: u64_at(p, 8) as i64, format, flags: p[17], bits })
    }
}

/// TX_BEGIN's arguments. `mode` is an lr20xx_tx_mode_t (0 CW, 1 FREQ, 2 PHASE_STEP, 3 IQ_PHASE, 4 IQ_POLAR).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TxBegin {
    pub mode: u8,
    pub center_hz: f64,
    pub ppm: f64,
    pub rate: u32,
    pub fdev_hz: u32,
    pub power_dbm: f32,
    pub samples: u64,
}

impl Payload for TxBegin {
    const SIZE: usize = TX_BEGIN_SIZE;
    fn put(&self, o: &mut Vec<u8>) {
        o.push(self.mode);
        o.extend_from_slice(&self.center_hz.to_le_bytes());
        o.extend_from_slice(&self.ppm.to_le_bytes());
        o.extend_from_slice(&self.rate.to_le_bytes());
        o.extend_from_slice(&self.fdev_hz.to_le_bytes());
        o.extend_from_slice(&self.power_dbm.to_le_bytes());
        o.extend_from_slice(&self.samples.to_le_bytes());
    }
    fn get(p: &[u8]) -> Option<Self> {
        if p.len() < Self::SIZE {
            return None;
        }
        Some(TxBegin {
            mode: p[0],
            center_hz: f64_at(p, 1),
            ppm: f64_at(p, 9),
            rate: u32_at(p, 17),
            fdev_hz: u32_at(p, 21),
            power_dbm: f32_at(p, 25),
            samples: u64_at(p, 29),
        })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TxResult {
    pub runs: i32,
    pub sent: u32,
    pub tx_flags: u8,
    pub early: u8,
    pub timed_out: u8,
}

impl Payload for TxResult {
    const SIZE: usize = TX_RESULT_SIZE;
    fn put(&self, o: &mut Vec<u8>) {
        o.extend_from_slice(&self.runs.to_le_bytes());
        o.extend_from_slice(&self.sent.to_le_bytes());
        o.extend_from_slice(&[self.tx_flags, self.early, self.timed_out]);
    }
    fn get(p: &[u8]) -> Option<Self> {
        if p.len() < Self::SIZE {
            return None;
        }
        Some(TxResult { runs: u32_at(p, 0) as i32, sent: u32_at(p, 4), tx_flags: p[8], early: p[9], timed_out: p[10] })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Stats {
    pub samples: u64,
    pub losses: u32,
    pub restarts: u32,
    pub errors: u32,
    pub reads: u32,
    pub irq_late: u32,
    pub link_drops: u32,
    pub ring_peak: u32,
}

impl Payload for Stats {
    const SIZE: usize = STATS_SIZE;
    fn put(&self, o: &mut Vec<u8>) {
        o.extend_from_slice(&self.samples.to_le_bytes());
        for w in [self.losses, self.restarts, self.errors, self.reads, self.irq_late, self.link_drops, self.ring_peak] {
            o.extend_from_slice(&w.to_le_bytes());
        }
    }
    fn get(p: &[u8]) -> Option<Self> {
        if p.len() < Self::SIZE {
            return None;
        }
        let w = |i: usize| u32_at(p, 8 + 4 * i);
        Some(Stats {
            samples: u64_at(p, 0),
            losses: w(0),
            restarts: w(1),
            errors: w(2),
            reads: w(3),
            irq_late: w(4),
            link_drops: w(5),
            ring_peak: w(6),
        })
    }
}

/// A whole frame around a payload. Panics on a payload past MAX_PAYLOAD (a caller's bug).
pub fn frame(ftype: u8, payload: &[u8]) -> Vec<u8> {
    assert!(payload.len() <= MAX_PAYLOAD, "link payload of {} bytes", payload.len());
    let mut v = Vec::with_capacity(HEADER + payload.len());
    v.push(MAGIC);
    v.push(ftype);
    v.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    v.extend_from_slice(payload);
    v
}

pub fn req(seq: u16, command: u16, args: &[u8]) -> Vec<u8> {
    let mut p = Vec::with_capacity(4 + args.len());
    p.extend_from_slice(&seq.to_le_bytes());
    p.extend_from_slice(&command.to_le_bytes());
    p.extend_from_slice(args);
    frame(frame_type::REQ, &p)
}

pub fn rsp(seq: u16, err: i32, ans: &[u8]) -> Vec<u8> {
    let mut p = Vec::with_capacity(6 + ans.len());
    p.extend_from_slice(&seq.to_le_bytes());
    p.extend_from_slice(&err.to_le_bytes());
    p.extend_from_slice(ans);
    frame(frame_type::RSP, &p)
}

pub fn event(kind: u16, err: i32, text: &str) -> Vec<u8> {
    let t = &text.as_bytes()[..text.len().min(MAX_PAYLOAD - 6)];
    let mut p = Vec::with_capacity(6 + t.len());
    p.extend_from_slice(&kind.to_le_bytes());
    p.extend_from_slice(&err.to_le_bytes());
    p.extend_from_slice(t);
    frame(frame_type::EVENT, &p)
}

pub fn rx_data(h: &RxHeader, samples: &[u8]) -> Vec<u8> {
    let mut p = h.to_bytes();
    p.extend_from_slice(samples);
    frame(frame_type::RX_DATA, &p)
}

pub fn tx_data(samples: &[u8]) -> Vec<u8> {
    frame(frame_type::TX_DATA, samples)
}

/// A frame as parsed: its type and payload.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub ftype: u8,
    pub payload: Vec<u8>,
}

impl Frame {
    /// A REQ's sequence number, command and arguments.
    pub fn as_req(&self) -> Option<(u16, u16, &[u8])> {
        (self.ftype == frame_type::REQ && self.payload.len() >= 4)
            .then(|| (u16_at(&self.payload, 0), u16_at(&self.payload, 2), &self.payload[4..]))
    }
    /// An RSP's sequence number, error (an lr20xx_err_t) and answer.
    pub fn as_rsp(&self) -> Option<(u16, i32, &[u8])> {
        (self.ftype == frame_type::RSP && self.payload.len() >= 6)
            .then(|| (u16_at(&self.payload, 0), u32_at(&self.payload, 2) as i32, &self.payload[6..]))
    }
    /// An RX_DATA's header and samples.
    pub fn as_rx_data(&self) -> Option<(RxHeader, &[u8])> {
        if self.ftype != frame_type::RX_DATA {
            return None;
        }
        RxHeader::get(&self.payload).map(|h| (h, &self.payload[RX_HDR_SIZE..]))
    }
    /// An EVENT's kind, error and text.
    pub fn as_event(&self) -> Option<(u16, i32, String)> {
        (self.ftype == frame_type::EVENT && self.payload.len() >= 6).then(|| {
            (
                u16_at(&self.payload, 0),
                u32_at(&self.payload, 2) as i32,
                String::from_utf8_lossy(&self.payload[6..]).into_owned(),
            )
        })
    }
}

fn known_type(t: u8) -> bool {
    matches!(t, frame_type::REQ | frame_type::RSP | frame_type::RX_DATA | frame_type::TX_DATA | frame_type::EVENT)
}

/// A byte-stream parser: feed bytes, take frames. Loses sync the same way the C parser does: it skips to the next
/// magic byte with a known type and a length in range.
#[derive(Default)]
pub struct Parser {
    buf: Vec<u8>,
    /// Bytes dropped to find a frame.
    pub skipped: u64,
}

impl Parser {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn feed(&mut self, data: &[u8]) {
        self.buf.extend_from_slice(data);
    }

    fn resync(&mut self) {
        let mut i = 0;
        while i < self.buf.len() {
            if self.buf[i] == MAGIC {
                if self.buf.len() - i < HEADER {
                    break;
                }
                if known_type(self.buf[i + 1]) && u16_at(&self.buf, i + 2) as usize <= MAX_PAYLOAD {
                    break;
                }
            }
            i += 1;
        }
        if i > 0 {
            self.buf.drain(..i);
            self.skipped += i as u64;
        }
    }

    /// The next whole frame, if one is in.
    pub fn next_frame(&mut self) -> Option<Frame> {
        self.resync();
        if self.buf.len() < HEADER {
            return None;
        }
        let want = HEADER + u16_at(&self.buf, 2) as usize;
        if self.buf.len() < want {
            return None;
        }
        let f = Frame { ftype: self.buf[1], payload: self.buf[HEADER..want].to_vec() };
        self.buf.drain(..want);
        Some(f)
    }
}

// ── Samples on the wire (RX_DATA) ──
// Each value (I, then Q) is a `bits`-bit two's complement integer, packed LSB first into a little-endian bit
// stream; the last byte's spare bits are 0. At full width it's the chip's format byte for byte (16: int16
// little-endian, 8: int8). The same as lr20xx_link.h's lr20xx_link_pack.

/// The narrowest width a packed value may have.
pub const BITS_MIN: u8 = 4;

fn chip_values(samples: &[u8], format: u8) -> impl Iterator<Item = i32> + '_ {
    let step = if format != 0 { 1 } else { 2 };
    samples
        .chunks_exact(step)
        .map(move |c| if step == 1 { c[0] as i8 as i32 } else { i16::from_le_bytes([c[0], c[1]]) as i32 })
}

/// The narrowest width (at least [`BITS_MIN`]) that holds every value in `samples` (the chip's format).
pub fn sample_bits(samples: &[u8], format: u8) -> u8 {
    let m = chip_values(samples, format).fold(0u32, |m, v| m | if v < 0 { !v as u32 } else { v as u32 });
    let bits = (33 - m.leading_zeros()) as u8; // a b-bit value holds m < 2^(b-1)
    bits.max(BITS_MIN)
}

/// `samples` (the chip's format) packed at `bits` a value.
pub fn pack(samples: &[u8], format: u8, bits: u8) -> Vec<u8> {
    let mask = (1u32 << bits) - 1;
    let (mut acc, mut have) = (0u32, 0u32);
    let mut out = Vec::with_capacity(samples.len());
    for v in chip_values(samples, format) {
        acc |= (v as u32 & mask) << have;
        have += bits as u32;
        while have >= 8 {
            out.push(acc as u8);
            acc >>= 8;
            have -= 8;
        }
    }
    if have > 0 {
        out.push(acc as u8);
    }
    out
}

/// How many values `n` bytes packed at `bits` hold (whole I/Q pairs).
pub fn packed_values(n: usize, bits: u8) -> usize {
    if !(BITS_MIN..=16).contains(&bits) {
        return 0;
    }
    (n * 8 / bits as usize) & !1
}

/// The values in `packed`, at `bits` each.
pub fn unpack(packed: &[u8], bits: u8) -> Vec<i16> {
    let n = packed_values(packed.len(), bits);
    let (mask, sign) = ((1u32 << bits) - 1, 1u32 << (bits - 1));
    let (mut acc, mut have, mut j) = (0u32, 0u32, 0usize);
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        while have < bits as u32 {
            acc |= (packed[j] as u32) << have;
            j += 1;
            have += 8;
        }
        let v = acc & mask;
        out.push(if v & sign != 0 { v as i32 - (mask as i32 + 1) } else { v as i32 } as i16);
        acc >>= bits;
        have -= bits as u32;
    }
    out
}

/// `packed` back in the chip's format (int16 little-endian, or int8).
pub fn unpack_to_chip(packed: &[u8], format: u8, bits: u8) -> Vec<u8> {
    if bits == if format != 0 { 8 } else { 16 } {
        return packed.to_vec();
    }
    let v = unpack(packed, bits);
    if format != 0 {
        v.iter().map(|&x| x as i8 as u8).collect()
    } else {
        v.iter().flat_map(|&x| x.to_le_bytes()).collect()
    }
}
