// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! A blocking client of the bridge link over any byte stream: a TcpStream, or a serial port the caller opens.

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::net::{Ipv4Addr, SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use crate::codec::*;

#[derive(Debug)]
pub enum Error {
    Io(io::Error),
    /// The device answered with an lr20xx_err_t other than 0.
    Device(i32),
    /// The device's answer didn't parse, or the stream closed.
    Protocol(&'static str),
    /// The device reported that receiving failed (an EVENT).
    RxFailed(i32, String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Io(e) => write!(f, "link: {e}"),
            Error::Device(e) => write!(f, "device error {e:#x}"),
            Error::Protocol(s) => write!(f, "link protocol: {s}"),
            Error::RxFailed(e, s) => write!(f, "receive failed ({e:#x}): {s}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Error::Io(e)
    }
}

pub type Result<T> = std::result::Result<T, Error>;

/// A block of received samples.
#[derive(Clone, Debug)]
pub struct RxBlock {
    pub header: RxHeader,
    /// In the chip's format (int16 little-endian, or int8: `header.format`), unpacked from the wire's.
    pub samples: Vec<u8>,
}

impl RxBlock {
    pub fn gap(&self) -> bool {
        self.header.flags & FLAG_GAP != 0
    }
}

/// One device on a stream. Answers come in order; RX_DATA that arrives while waiting for one is queued for
/// [`Client::next_block`].
pub struct Client<S: Read + Write> {
    stream: S,
    parser: Parser,
    seq: u16,
    blocks: VecDeque<RxBlock>,
    /// Log lines the device sent (EVENT_LOG), oldest first.
    pub log: VecDeque<String>,
    failed: Option<(i32, String)>,
    buf: Vec<u8>,
}

impl<S: Read + Write> Client<S> {
    pub fn new(stream: S) -> Self {
        Client {
            stream,
            parser: Parser::new(),
            seq: 0,
            blocks: VecDeque::new(),
            log: VecDeque::new(),
            failed: None,
            buf: vec![0; 16384],
        }
    }

    pub fn into_inner(self) -> S {
        self.stream
    }

    pub fn get_mut(&mut self) -> &mut S {
        &mut self.stream
    }

    /// Read until a frame is in, sorting RX_DATA and EVENTs aside; the frame if it's anything else.
    fn pump(&mut self) -> Result<Option<Frame>> {
        loop {
            if let Some(f) = self.parser.next_frame() {
                if let Some((h, s)) = f.as_rx_data() {
                    self.blocks.push_back(RxBlock { header: h, samples: unpack_to_chip(s, h.format, h.bits) });
                } else if let Some((kind, err, text)) = f.as_event() {
                    if kind == EVENT_RX_FAILED {
                        self.failed = Some((err, text));
                    } else {
                        if self.log.len() >= 256 {
                            self.log.pop_front();
                        }
                        self.log.push_back(text);
                    }
                } else {
                    return Ok(Some(f));
                }
                return Ok(None);
            }
            let n = self.stream.read(&mut self.buf)?;
            if n == 0 {
                return Err(Error::Protocol("stream closed"));
            }
            self.parser.feed(&self.buf[..n]);
        }
    }

    /// A command and its answer.
    pub fn request(&mut self, command: u16, args: &[u8]) -> Result<Vec<u8>> {
        self.seq = self.seq.wrapping_add(1);
        let seq = self.seq;
        self.stream.write_all(&req(seq, command, args))?;
        self.stream.flush()?;
        loop {
            if let Some(f) = self.pump()? {
                if let Some((s, err, ans)) = f.as_rsp() {
                    if s != seq {
                        continue; // a stale answer
                    }
                    if err != 0 {
                        return Err(Error::Device(err));
                    }
                    return Ok(ans.to_vec());
                }
            }
        }
    }

    pub fn hello(&mut self) -> Result<Hello> {
        let a = self.request(cmd::HELLO, &[])?;
        Hello::get(&a).ok_or(Error::Protocol("short HELLO"))
    }

    pub fn rx_config(&mut self, c: &RxConfig) -> Result<Plan> {
        let a = self.request(cmd::RX_CONFIG, &c.to_bytes())?;
        Plan::get(&a).ok_or(Error::Protocol("short plan"))
    }

    pub fn rx_start(&mut self) -> Result<()> {
        self.failed = None;
        self.request(cmd::RX_START, &[]).map(|_| ())
    }

    /// Stop receiving; blocks already queued are dropped.
    pub fn rx_stop(&mut self) -> Result<()> {
        self.request(cmd::RX_STOP, &[])?;
        self.blocks.clear();
        Ok(())
    }

    pub fn set_gain(&mut self, step: i8) -> Result<()> {
        self.request(cmd::SET_GAIN, &[step as u8]).map(|_| ())
    }

    /// The next block of samples, waiting for it.
    pub fn next_block(&mut self) -> Result<RxBlock> {
        loop {
            if let Some(b) = self.blocks.pop_front() {
                return Ok(b);
            }
            if let Some((e, t)) = self.failed.take() {
                return Err(Error::RxFailed(e, t));
            }
            self.pump()?; // an unexpected answer here is dropped
        }
    }

    /// Blocks as an iterator (ends on the first error).
    pub fn blocks(&mut self) -> impl Iterator<Item = Result<RxBlock>> + '_ {
        std::iter::from_fn(move || Some(self.next_block()))
    }

    /// One transmission: TX_BEGIN, the samples in TX_DATA frames, TX_COMMIT (which answers once it has gone out).
    /// `samples` is in the mode's FIFO format; `begin.samples` is set from its length.
    ///
    /// This client doesn't read while it writes: while receiving, the samples the bridge sends meanwhile wait in the
    /// stream's buffers, and a bridge that can't send for 2 s drops the host. Stop receiving first for a burst whose
    /// upload is long (over the stream's receive buffer at the receive rate), or read and write on separate threads.
    pub fn transmit(&mut self, begin: &TxBegin, samples: &[u8], bytes_per_sample: usize) -> Result<TxResult> {
        let mut b = *begin;
        b.samples = (samples.len() / bytes_per_sample.max(1)) as u64;
        self.request(cmd::TX_BEGIN, &b.to_bytes())?;
        let step = (MAX_PAYLOAD / bytes_per_sample.max(1)) * bytes_per_sample.max(1);
        for chunk in samples.chunks(step.max(1)) {
            self.stream.write_all(&tx_data(chunk))?;
        }
        let a = self.request(cmd::TX_COMMIT, &[])?;
        TxResult::get(&a).ok_or(Error::Protocol("short TX result"))
    }

    /// The device's clock, us (needs [`crate::CAP_TIME`] in the HELLO's caps).
    pub fn time_us(&mut self) -> Result<i64> {
        let a = self.request(cmd::TIME, &[])?;
        if a.len() < 8 {
            return Err(Error::Protocol("short TIME answer"));
        }
        Ok(i64::from_le_bytes(a[..8].try_into().unwrap()))
    }

    pub fn tx_abort(&mut self) -> Result<()> {
        self.request(cmd::TX_ABORT, &[]).map(|_| ())
    }

    pub fn read_reg(&mut self, addr: u32, n: u8) -> Result<Vec<u32>> {
        let mut a = addr.to_le_bytes().to_vec();
        a.push(n);
        let r = self.request(cmd::READ_REG, &a)?;
        if r.len() < 4 * n as usize {
            return Err(Error::Protocol("short register read"));
        }
        Ok(r.chunks_exact(4).take(n as usize).map(|c| u32::from_le_bytes(c.try_into().unwrap())).collect())
    }

    pub fn write_reg(&mut self, addr: u32, value: u32) -> Result<()> {
        let mut a = addr.to_le_bytes().to_vec();
        a.extend_from_slice(&value.to_le_bytes());
        self.request(cmd::WRITE_REG, &a).map(|_| ())
    }

    /// A raw command frame; returns the status bytes and `answer` bytes of answer.
    pub fn raw(&mut self, frame: &[u8], answer: u16) -> Result<([u8; 2], Vec<u8>)> {
        let mut a = answer.to_le_bytes().to_vec();
        a.extend_from_slice(frame);
        let r = self.request(cmd::RAW, &a)?;
        if r.len() < 2 {
            return Err(Error::Protocol("short raw answer"));
        }
        Ok(([r[0], r[1]], r[2..].to_vec()))
    }

    pub fn stats(&mut self) -> Result<Stats> {
        let a = self.request(cmd::STATS, &[])?;
        Stats::get(&a).ok_or(Error::Protocol("short stats"))
    }

    pub fn temp(&mut self) -> Result<f32> {
        let a = self.request(cmd::TEMP, &[])?;
        if a.len() < 4 {
            return Err(Error::Protocol("short temperature"));
        }
        Ok(f32::from_le_bytes(a[..4].try_into().unwrap()))
    }
}

/// Bridges answering UDP discovery within `timeout`, asked on every IPv4 interface that can broadcast (so a bridge
/// on a USB network link is found too, not only on the default route's), deduplicated by serial number.
pub fn discover(timeout: Duration) -> io::Result<Vec<(SocketAddr, Hello)>> {
    let mut to: Vec<Ipv4Addr> = if_addrs::get_if_addrs()
        .unwrap_or_default()
        .into_iter()
        .filter(|i| !i.is_loopback())
        .filter_map(|i| match i.addr {
            if_addrs::IfAddr::V4(a) => a.broadcast,
            _ => None,
        })
        .collect();
    to.push(Ipv4Addr::BROADCAST);
    to.sort();
    to.dedup();
    discover_to(&to, timeout)
}

/// Discovery asked of one address: a bridge's, or a subnet's broadcast address.
pub fn discover_at(addr: Ipv4Addr, timeout: Duration) -> io::Result<Vec<(SocketAddr, Hello)>> {
    discover_to(&[addr], timeout)
}

fn discover_to(to: &[Ipv4Addr], timeout: Duration) -> io::Result<Vec<(SocketAddr, Hello)>> {
    let sock = UdpSocket::bind("0.0.0.0:0")?;
    sock.set_broadcast(true)?;
    let mut sent = false;
    for a in to {
        // an interface that can't send (down since it was listed, say) is skipped
        sent |= sock.send_to(DISCOVERY.as_bytes(), (*a, DISCOVERY_PORT)).is_ok();
    }
    if !sent {
        return Err(io::Error::new(io::ErrorKind::NotConnected, "discovery: no address could be sent to"));
    }
    let end = Instant::now() + timeout;
    let mut found: Vec<(SocketAddr, Hello)> = Vec::new();
    let mut buf = [0u8; 2048];
    loop {
        let left = end.saturating_duration_since(Instant::now());
        if left.is_zero() {
            break;
        }
        sock.set_read_timeout(Some(left))?;
        let (n, from) = match sock.recv_from(&mut buf) {
            Ok(r) => r,
            Err(e) if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => break,
            Err(e) => return Err(e),
        };
        let mut p = Parser::new();
        p.feed(&buf[..n]);
        if let Some(f) = p.next_frame() {
            if let Some((_, 0, ans)) = f.as_rsp() {
                if let Some(h) = Hello::get(ans) {
                    // one bridge answers once for each interface the question reached it on
                    let seen =
                        found.iter().any(|(a, f)| if h.serial.is_empty() { *a == from } else { f.serial == h.serial });
                    if !seen {
                        found.push((from, h));
                    }
                }
            }
        }
    }
    Ok(found)
}
