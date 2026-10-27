// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! The safe API against a chip scripted in Rust: the C driver runs over a Rust Hal, the C builders' frames are
//! checked against the datasheet's bytes (as lib/test/test_lr20xx.c has them), and a receive session streams.

use std::sync::{Arc, Mutex};

use lr20xx::*;

/// What the scripted chip saw and answers.
#[derive(Default)]
struct Chip {
    frames: Vec<Vec<u8>>,
    nreset: Vec<bool>,
    last_op: u16,
    last_addr: u32,
    fifo_level: u16,
    fifo_next: u8,
    fail_status: bool,
}

#[derive(Clone, Default)]
struct Mock(Arc<Mutex<Chip>>);

impl Hal for Mock {
    fn xfer(&mut self, tx: &[u8], rx: Option<&mut [u8]>) -> Result<()> {
        let mut c = self.0.lock().unwrap();
        c.frames.push(tx.to_vec());
        let op = u16::from_be_bytes([tx[0], tx[1]]);
        let mut ans = vec![0u8; tx.len()];
        if op == 0x0001 {
            // a direct FIFO read: the samples follow the status bytes in the same frame
            ans[0] = 0x04;
            for b in ans[2..].iter_mut() {
                *b = c.fifo_next;
                c.fifo_next = c.fifo_next.wrapping_add(1);
            }
        } else if op == 0x0000 {
            // the answer to the command before (a read's second frame)
            ans[0] = 0x06;
            let body: Vec<u8> = match c.last_op {
                0x0101 => vec![0x01, 0x18],                    // GetVersion: ROM 1.24
                0x011C => c.fifo_level.to_be_bytes().to_vec(), // GetRxFifoLevel
                0x0106 => match c.last_addr {
                    0xF30028 => 0x0080_4400u32.to_be_bytes().to_vec(), // the 1 KB FIFO's base
                    0xF30030 => 0x0000_03E8u32.to_be_bytes().to_vec(), // and size
                    _ => vec![0; 4],
                },
                _ => vec![],
            };
            for (i, b) in body.iter().enumerate() {
                if 2 + i < ans.len() {
                    ans[2 + i] = *b;
                }
            }
        } else {
            // GetStatus (0x0100) answers the last command's status: OK, or FAIL when scripted
            if op != 0x0100 {
                c.last_op = op;
                if op == 0x0106 {
                    c.last_addr = u32::from_be_bytes([0, tx[2], tx[3], tx[4]]);
                }
            }
            ans[0] = if c.fail_status { 0x00 } else { 0x04 };
        }
        if let Some(r) = rx {
            let n = r.len();
            r.copy_from_slice(&ans[..n]);
        }
        Ok(())
    }
    fn busy(&mut self) -> Result<bool> {
        Ok(false)
    }
    fn nreset(&mut self, high: bool) -> Result<()> {
        self.0.lock().unwrap().nreset.push(high);
        Ok(())
    }
}

#[test]
fn builders_match_the_datasheet() {
    assert_eq!(cmd::op(cmd::LR20XX_GET_VERSION as u16), [0x01, 0x01]);
    assert_eq!(cmd::op_u8(cmd::LR20XX_SET_STANDBY as u16, 0), [0x01, 0x28, 0x00]);
    assert_eq!(cmd::set_rf_frequency(461137500), [0x02, 0x00, 0x1B, 0x7C, 0x66, 0x5C]);
    assert_eq!(cmd::set_rx(0xFFFFFF), [0x02, 0x0C, 0xFF, 0xFF, 0xFF]);
    assert_eq!(cmd::set_tx(0x1234567), [0x02, 0x0D, 0x23, 0x45, 0x67]);
    assert_eq!(cmd::set_gain(12), [0x02, 0x1A, 0x0C]);
    assert_eq!(cmd::set_gain(20), [0x02, 0x1A, 0x0D]);
    assert_eq!(cmd::write_reg32(0xF30028, 0x804400), [0x01, 0x04, 0xF3, 0x00, 0x28, 0x00, 0x80, 0x44, 0x00]);
    assert_eq!(cmd::read_reg32(0xF30030, 1), [0x01, 0x06, 0xF3, 0x00, 0x30, 0x01]);
}

#[test]
fn device_over_a_rust_hal() {
    let m = Mock::default();
    let mut d = Device::open(m.clone()).unwrap();
    assert_eq!(m.0.lock().unwrap().nreset, [false, true], "reset: NRESET low, then high");
    assert_eq!(d.version().unwrap(), (1, 0x18));
    m.0.lock().unwrap().fifo_level = 300;
    assert_eq!(d.rx_fifo_level().unwrap(), 300);
    let mut buf = [0u8; 8];
    d.read_fifo(&mut buf).unwrap();
    assert_eq!(buf, [0, 1, 2, 3, 4, 5, 6, 7]);
    d.write(&cmd::op_u8(cmd::LR20XX_SET_STANDBY as u16, 0)).unwrap();
    assert_eq!(d.read32(0xF30030).unwrap(), 0x3E8);
    {
        let c = m.0.lock().unwrap();
        let n = c.frames.len();
        assert_eq!(c.frames[n - 2], [0x01, 0x06, 0xF3, 0x00, 0x30, 0x01], "ReadRegMem32's frame");
    }
    // a command the chip refuses: the driver's text comes through
    m.0.lock().unwrap().fail_status = true;
    let e = d.write(&cmd::set_rx(0)).unwrap_err();
    assert_eq!(e.code(), Error::FAIL);
    assert!(e.detail().contains("failed"), "{e}");
    m.0.lock().unwrap().fail_status = false;
    drop(d);
    assert_eq!(m.0.lock().unwrap().nreset.last(), Some(&false), "closed: held in reset");
}

#[test]
fn open_failure_frees_the_hal() {
    struct Stuck(#[allow(dead_code)] Arc<()>);
    impl Hal for Stuck {
        fn xfer(&mut self, _: &[u8], _: Option<&mut [u8]>) -> Result<()> {
            Ok(())
        }
        fn busy(&mut self) -> Result<bool> {
            Err(Error::new(Error::FAIL, "no pin"))
        }
        fn nreset(&mut self, _: bool) -> Result<()> {
            Ok(())
        }
    }
    let token = Arc::new(());
    let e = Device::open(Stuck(token.clone())).err().unwrap();
    assert_eq!(e.code(), Error::FAIL);
    assert_eq!(Arc::strong_count(&token), 1, "the HAL was dropped by close");
}

#[test]
fn boards_and_roms() {
    let w12 = Board::find("w12").unwrap();
    assert_eq!(w12.name(), "w12");
    assert_eq!(w12.irq_dio(), Some(7));
    assert!(Board::find("nonesuch").is_none());
    let b = Board::parse("generic irq=5").unwrap();
    assert_eq!(b.irq_dio(), Some(5));
    assert!(Board::parse("generic bogus=1").is_err());
    let r = Rom::find(1, 0x18).unwrap();
    assert!(r.iq_rx);
    assert!(Rom::find(9, 9).is_none());
    assert!(!Rom::all().is_empty());
}

#[test]
fn plans() {
    let p = plan(250_000.0, IqMode::Auto).unwrap();
    assert_eq!((p.mode, p.format, p.sample_bytes), (IqMode::Fsk16, IqFormat::Cs16, 4));
    assert!(p.rate >= 250_000.0);
    let p = plan(576_000.0, IqMode::Conv8).unwrap();
    assert_eq!((p.format, p.sample_bytes, p.rate), (IqFormat::Cs8, 2, 576_000.0));
    let (lora, range) = rates(IqMode::Lora);
    assert_eq!(lora, [125_000.0, 250_000.0]);
    assert!(range.is_none());
    assert!(rates(IqMode::Conv8).1.is_some());
    assert!(plan(5e6, IqMode::Fsk16).is_err());
    assert_eq!(gain_db()[12], 53.5);
}

#[test]
fn receive_session() {
    let m = Mock::default();
    let mut d = Device::open(m.clone()).unwrap();
    let mut s = IqSession::new(&mut d, Board::generic(), false).unwrap();
    assert_eq!(s.rom_version(), (1, 0x18));
    let p = s.start(&IqConfig { rate: 250_000.0, mode: IqMode::Auto, center_hz: 915e6, ppm: 0.0, gain: 13 }).unwrap();
    assert_eq!(p.sample_bytes, 4);
    m.0.lock().unwrap().fifo_level = 700; // past the reader's 640-byte target: read on the first look, whatever the sleeps
    let mut got = Vec::new();
    let mut total = 0;
    for _ in 0..4 {
        total += s
            .poll(5, |b, flags, _t| {
                assert_eq!(b.len() % 4, 0, "whole samples");
                assert_eq!(flags & GAP, 0);
                got.extend_from_slice(b);
            })
            .unwrap();
    }
    assert!(total > 0 && got.len() == total * 4, "{total} samples, {} bytes", got.len());
    // the bytes come out in order, across reads that end mid-sample
    for (i, w) in got.windows(2).enumerate() {
        assert_eq!(w[1], w[0].wrapping_add(1), "byte {i}");
    }
    assert_eq!(s.stats().samples, total as u64);
    s.stop();
}
