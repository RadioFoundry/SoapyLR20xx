// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! The pure-Rust link (lr20xx-link) against the C encoder (proto/src/lr20xx_link.c): the same structs encode to
//! the same bytes, and the Rust parser reads frames the C wrote.

use link::Payload;
use lr20xx::sys;
use lr20xx_link as link;

fn c_bytes(n: usize, f: impl FnOnce(*mut u8) -> usize) -> Vec<u8> {
    let mut b = vec![0u8; n + 64];
    let k = f(b.as_mut_ptr());
    b.truncate(k);
    b
}

fn name(s: &str) -> [std::ffi::c_char; 24] {
    let mut a = [0 as std::ffi::c_char; 24];
    for (i, c) in s.bytes().take(23).enumerate() {
        a[i] = c as std::ffi::c_char;
    }
    a
}

#[test]
fn payloads_match_c() {
    let h = sys::lr20xx_link_hello_t {
        version: 1,
        rom_major: 1,
        rom_minor: 0x18,
        rom_known: 1,
        caps: 7,
        tx_max_samples: 4_000_000,
        freq_min: 150e6,
        freq_max: 960e6,
        tx_power_max: 22.0,
        board: name("w12"),
        firmware: name("0.1.0"),
        serial: name("A1B2C3"),
    };
    let c = c_bytes(link::HELLO_SIZE, |p| unsafe { sys::lr20xx_link_put_hello(p, &h) });
    let r = link::Hello {
        version: 1,
        rom_major: 1,
        rom_minor: 0x18,
        rom_known: 1,
        caps: 7,
        tx_max_samples: 4_000_000,
        freq_min: 150e6,
        freq_max: 960e6,
        tx_power_max: 22.0,
        board: "w12".into(),
        firmware: "0.1.0".into(),
        serial: "A1B2C3".into(),
    };
    assert_eq!(c, r.to_bytes());
    assert_eq!(link::Hello::get(&c), Some(r));

    let cfg = sys::lr20xx_link_rx_config_t { rate: 250e3, mode: 2, gain: -3, center_hz: 915e6, ppm: -28.5 };
    let c = c_bytes(link::RX_CONFIG_SIZE, |p| unsafe { sys::lr20xx_link_put_rx_config(p, &cfg) });
    assert_eq!(c, link::RxConfig { rate: 250e3, mode: 2, gain: -3, center_hz: 915e6, ppm: -28.5 }.to_bytes());

    let pl = sys::lr20xx_link_plan_t { mode: 3, format: 1, rate_num: 576000, rate_den: 1, passband_hz: 3e5 };
    let c = c_bytes(link::PLAN_SIZE, |p| unsafe { sys::lr20xx_link_put_plan(p, &pl) });
    assert_eq!(c, link::Plan { mode: 3, format: 1, rate_num: 576000, rate_den: 1, passband_hz: 3e5 }.to_bytes());

    let tb = sys::lr20xx_link_tx_begin_t {
        mode: 1,
        center_hz: 461.3375e6,
        ppm: 1.0,
        rate: 48000,
        fdev_hz: 2048,
        power_dbm: -9.5,
        samples: 123456789012,
    };
    let c = c_bytes(link::TX_BEGIN_SIZE, |p| unsafe { sys::lr20xx_link_put_tx_begin(p, &tb) });
    assert_eq!(
        c,
        link::TxBegin {
            mode: 1,
            center_hz: 461.3375e6,
            ppm: 1.0,
            rate: 48000,
            fdev_hz: 2048,
            power_dbm: -9.5,
            samples: 123456789012
        }
        .to_bytes()
    );

    let tr = sys::lr20xx_link_tx_result_t { runs: -2, sent: 70000, tx_flags: 0x20, early: 1, timed_out: 0 };
    let c = c_bytes(link::TX_RESULT_SIZE, |p| unsafe { sys::lr20xx_link_put_tx_result(p, &tr) });
    assert_eq!(c, link::TxResult { runs: -2, sent: 70000, tx_flags: 0x20, early: 1, timed_out: 0 }.to_bytes());

    let st = sys::lr20xx_link_stats_t {
        samples: 1 << 33,
        losses: 1,
        restarts: 2,
        errors: 3,
        reads: 4,
        irq_late: 5,
        link_drops: 6,
        ring_peak: 7,
    };
    let c = c_bytes(link::STATS_SIZE, |p| unsafe { sys::lr20xx_link_put_stats(p, &st) });
    let r = link::Stats {
        samples: 1 << 33,
        losses: 1,
        restarts: 2,
        errors: 3,
        reads: 4,
        irq_late: 5,
        link_drops: 6,
        ring_peak: 7,
    };
    assert_eq!(c, r.to_bytes());
}

#[test]
fn frames_match_c() {
    let args = [1u8, 2, 3];
    let c = c_bytes(16, |p| unsafe { sys::lr20xx_link_req(p, 16, 0xBEEF, 0x0010, args.as_ptr().cast(), 3) });
    assert_eq!(c, link::req(0xBEEF, 0x0010, &args));
    let c = c_bytes(16, |p| unsafe { sys::lr20xx_link_rsp(p, 16, 9, 0x107, args.as_ptr().cast(), 3) });
    assert_eq!(c, link::rsp(9, 0x107, &args));
    let text = std::ffi::CString::new("session failed").unwrap();
    let c = c_bytes(64, |p| unsafe { sys::lr20xx_link_event(p, 64, 1, -1, text.as_ptr()) });
    assert_eq!(c, link::event(1, -1, "session failed"));
    let h = sys::lr20xx_link_rx_hdr_t { first: 1000, t_us: -5, format: 0, flags: 1, bits: 16 };
    let s = [9u8; 8];
    let c = c_bytes(64, |p| unsafe { sys::lr20xx_link_rx_data(p, 64, &h, s.as_ptr(), s.len()) });
    assert_eq!(c, link::rx_data(&link::RxHeader { first: 1000, t_us: -5, format: 0, flags: 1, bits: 16 }, &s));

    // the Rust parser reads a stream of C frames with junk between
    let mut stream = vec![0x13, 0xA5];
    stream.extend_from_slice(&c);
    stream.extend_from_slice(&c_bytes(16, |p| unsafe { sys::lr20xx_link_req(p, 16, 1, 0x0001, std::ptr::null(), 0) }));
    let mut p = link::Parser::new();
    p.feed(&stream);
    let f = p.next_frame().unwrap();
    let (hdr, samples) = f.as_rx_data().unwrap();
    assert_eq!((hdr.first, hdr.t_us, samples), (1000, -5, &s[..]));
    assert_eq!(p.next_frame().unwrap().as_req(), Some((1, 0x0001, &[][..])));
    assert_eq!(p.skipped, 2);

    // and the C parser reads Rust frames
    let mut cp: Box<sys::lr20xx_link_parser_t> = Box::new(unsafe { std::mem::zeroed() });
    unsafe { sys::lr20xx_link_parser_init(&mut *cp) };
    let r = link::rsp(77, 0, &[5, 6]);
    let took = unsafe { sys::lr20xx_link_feed(&mut *cp, r.as_ptr(), r.len()) };
    assert_eq!(took, r.len());
    let mut f: sys::lr20xx_link_frame_t = unsafe { std::mem::zeroed() };
    assert!(unsafe { sys::lr20xx_link_next(&mut *cp, &mut f) });
    let (mut seq, mut err, mut ans, mut n) = (0u16, 0i32, std::ptr::null(), 0usize);
    assert!(unsafe { sys::lr20xx_link_parse_rsp(&f, &mut seq, &mut err, &mut ans, &mut n) });
    assert_eq!((seq, err, unsafe { std::slice::from_raw_parts(ans, n) }), (77, 0, &[5u8, 6][..]));
}

#[test]
fn packing_matches_c() {
    // a ramp through each width's whole range, int16 and int8, packed by C and by Rust
    for format in [0u8, 1] {
        let top = if format != 0 { 8 } else { 16 };
        for bits in link::BITS_MIN..=top {
            let lo = -(1i32 << (bits - 1));
            let span = 1i32 << bits;
            let values: Vec<i32> = (0..202).map(|i| lo + (i * 7919) % span).chain([lo, lo + span - 1]).collect();
            let chip: Vec<u8> = if format != 0 {
                values.iter().map(|&v| v as i8 as u8).collect()
            } else {
                values.iter().flat_map(|&v| (v as i16).to_le_bytes()).collect()
            };
            let n = values.len();
            let cb = unsafe { sys::lr20xx_link_bits(chip.as_ptr(), n, format as u32) };
            assert_eq!(cb as u8, link::sample_bits(&chip, format), "width, format {format} bits {bits}");
            let c = c_bytes(chip.len(), |p| unsafe {
                sys::lr20xx_link_pack(p, chip.as_ptr(), n, format as u32, bits as u32)
            });
            assert_eq!(c, link::pack(&chip, format, bits), "bytes, format {format} bits {bits}");
            let mut back = vec![0i16; n];
            let got = unsafe { sys::lr20xx_link_unpack(back.as_mut_ptr(), n, c.as_ptr(), c.len(), bits as u32) };
            assert_eq!(&back[..got], &link::unpack(&c, bits)[..], "values, format {format} bits {bits}");
            assert_eq!(
                link::unpack_to_chip(&c, format, bits),
                chip,
                "the chip's bytes back, format {format} bits {bits}"
            );
        }
    }
}
