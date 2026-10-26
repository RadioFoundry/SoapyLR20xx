// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! The link's encoding, the parser's resync, and the client against a scripted device.

use std::io::{self, Read, Write};

use lr20xx_link::*;

#[test]
fn sizes_match_the_header() {
    assert_eq!(Hello::default().to_bytes().len(), HELLO_SIZE);
    assert_eq!(RxConfig::default().to_bytes().len(), RX_CONFIG_SIZE);
    assert_eq!(Plan::default().to_bytes().len(), PLAN_SIZE);
    assert_eq!(RxHeader::default().to_bytes().len(), RX_HDR_SIZE);
    assert_eq!(TxBegin::default().to_bytes().len(), TX_BEGIN_SIZE);
    assert_eq!(TxResult::default().to_bytes().len(), TX_RESULT_SIZE);
    assert_eq!(Stats::default().to_bytes().len(), STATS_SIZE);
}

#[test]
fn frames() {
    assert_eq!(req(0x0102, cmd::RX_START, &[]), vec![0xA5, 0x01, 4, 0, 0x02, 0x01, 0x11, 0x00]);
    assert_eq!(rsp(7, -1, &[9]), vec![0xA5, 0x02, 7, 0, 7, 0, 0xFF, 0xFF, 0xFF, 0xFF, 9]);
    let h = Hello {
        version: 1,
        rom_major: 1,
        rom_minor: 0x18,
        board: "w12".into(),
        firmware: "x".repeat(40),
        ..Default::default()
    };
    let back = Hello::get(&h.to_bytes()).unwrap();
    assert_eq!(back.board, "w12");
    assert_eq!(back.firmware.len(), 23, "a long string is cut to 23 and a NUL");
    let t = TxBegin {
        mode: 4,
        center_hz: 920e6,
        ppm: -28.0,
        rate: 48000,
        fdev_hz: 2048,
        power_dbm: -9.5,
        samples: 1 << 40,
    };
    assert_eq!(TxBegin::get(&t.to_bytes()), Some(t));
    let p = Plan { mode: 2, format: 0, rate_num: 2_000_000, rate_den: 3, passband_hz: 1e5 };
    assert_eq!(Plan::get(&p.to_bytes()), Some(p));
    assert!(Plan::get(&p.to_bytes()[..17]).is_none());
}

#[test]
fn parser_resyncs() {
    let mut p = Parser::new();
    let a = rx_data(&RxHeader { first: 5, t_us: 6, format: 0, flags: FLAG_GAP, bits: 16 }, &[1, 2, 3, 4]);
    let b = rsp(3, 0, &[0xA5, 0x02, 0x00, 0x00]); // a payload that looks like a header
    let mut stream = vec![0x00, 0xA5, 0x77, 0xA5]; // junk: a magic with a bad type, then a magic at the end
    stream.extend_from_slice(&[0xA5, 0x01, 0xFF, 0xFF]); // a length past the maximum
    stream.extend_from_slice(&a);
    stream.extend_from_slice(&b);
    // a byte at a time
    let mut got = Vec::new();
    for c in &stream {
        p.feed(std::slice::from_ref(c));
        while let Some(f) = p.next_frame() {
            got.push(f);
        }
    }
    assert_eq!(got.len(), 2);
    let (h, s) = got[0].as_rx_data().unwrap();
    assert_eq!((h.first, h.t_us, h.flags, s), (5, 6, FLAG_GAP, &[1u8, 2, 3, 4][..]));
    assert_eq!(got[1].as_rsp(), Some((3, 0, &[0xA5u8, 0x02, 0, 0][..])));
    assert_eq!(p.skipped, 8);
}

/// A device that answers from a script: each request gets the next answer; RX_DATA can be queued ahead.
struct Scripted {
    out: Vec<u8>,
    sent: Vec<u8>,
    answers: Vec<(i32, Vec<u8>)>,
    extra: Vec<Vec<u8>>,
}

impl Read for Scripted {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let n = buf.len().min(self.out.len());
        buf[..n].copy_from_slice(&self.out[..n]);
        self.out.drain(..n);
        Ok(n)
    }
}

impl Write for Scripted {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.sent.extend_from_slice(buf);
        let mut p = Parser::new();
        p.feed(buf);
        while let Some(f) = p.next_frame() {
            if let Some((seq, _, _)) = f.as_req() {
                for e in self.extra.drain(..) {
                    self.out.extend_from_slice(&e);
                }
                let (err, ans) = self.answers.remove(0);
                self.out.extend_from_slice(&rsp(seq, err, &ans));
            }
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[test]
fn client() {
    let hello = Hello {
        version: 1,
        rom_major: 1,
        rom_minor: 0x18,
        rom_known: 1,
        caps: CAP_RX | CAP_TX,
        board: "w12".into(),
        ..Default::default()
    };
    let plan = Plan { mode: 2, format: 0, rate_num: 250_000, rate_den: 1, passband_hz: 9e4 };
    let dev = Scripted {
        out: Vec::new(),
        sent: Vec::new(),
        answers: vec![(0, hello.to_bytes()), (0, plan.to_bytes()), (0, vec![]), (0x107, vec![])],
        extra: vec![],
    };
    let mut c = Client::new(dev);
    assert_eq!(c.hello().unwrap().board, "w12");
    let got = c.rx_config(&RxConfig { rate: 250e3, mode: 0, gain: 13, center_hz: 915e6, ppm: 0.0 }).unwrap();
    assert_eq!(got.rate(), 250e3);
    // RX_DATA ahead of the answer is kept for next_block
    c.get_mut().extra.push(rx_data(&RxHeader { first: 0, t_us: 1, format: 0, flags: 0, bits: 16 }, &[0; 8]));
    c.rx_start().unwrap();
    let b = c.next_block().unwrap();
    assert_eq!((b.samples.len(), b.gap()), (8, false));
    match c.set_gain(5) {
        Err(Error::Device(0x107)) => {}
        r => panic!("expected the device's error, got {r:?}"),
    }
}
