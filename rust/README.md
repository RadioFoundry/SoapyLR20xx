# SoapyLR20xx for Rust

Rust crates for the LR20xx SDR library in this repository. The chip logic is written once, in C (`lib/`); Rust
binds it. The C is the reference: when the two disagree, the C is right and the binding is the bug.

| Crate | What |
|---|---|
| `lr20xx-sys` | Raw bindings to liblr20xx and the bridge link's C encoder. `build.rs` compiles `lib/` and `proto/` with the `cc` crate: the POSIX port, and on Linux the spidev / GPIO HAL. |
| `lr20xx` | A safe API over `lr20xx-sys`: `Device` (an open chip over your `Hal`, or Linux's spidev), `IqSession` (raw I/Q receive and sample transmit), `Board`, `Rom`, `cmd` (command frames). Feature `embedded-hal`: an adapter from embedded-hal 1.0's `SpiDevice` and pins. |
| `lr20xx-link` | The bridge link (`proto/include/lr20xx_link.h`, `docs/link.md`) in pure Rust, with no C: frames, payloads, a parser and a blocking `Client` over any `Read + Write` (a `TcpStream`, or a serial port you open), plus UDP discovery. For hosts that talk to the bridge firmware. |

`lr20xx-link` is the one place the Rust has its own code rather than a binding: it's a wire format, small, and a
host that only talks to a bridge shouldn't need a C compiler. Its tests encode the same structs with the C and the
Rust and compare the bytes, and each side parses the other's frames.

## Building

```sh
cd rust
CARGO_TARGET_DIR=../build/rust cargo test --all-features
```

A C compiler is needed for `lr20xx-sys` (cc finds the system's). libclang isn't: the bindings are generated ahead
of time and committed (`lr20xx-sys/src/bindings.rs`, and `bindings_linux.rs` for the Linux HAL). After a C header
changes, regenerate them with `tools/gen-bindings.sh` (needs `cargo install bindgen-cli` and libclang).

## Using it

```rust
use lr20xx::{Board, Device, IqConfig, IqMode, IqSession};

let mut dev = Device::open(my_hal)?;             // or Device::open_linux(&LinuxConfig { .. }) on Linux
let mut sdr = IqSession::new(&mut dev, Board::find("w12").unwrap(), false)?;
let plan = sdr.start(&IqConfig { rate: 250e3, mode: IqMode::Auto, center_hz: 915e6, ppm: 0.0, gain: 13 })?;
loop {
    sdr.poll(2, |samples, flags, t_us| { /* plan.sample_bytes a sample, I then Q, little-endian */ })?;
}
```

Run the reader loop on a thread of its own at a real-time priority: the chip's FIFO holds 2 ms of samples at
125 kS/s int16 (`docs/iq.md` R2.18). Every other call on the session must come from that thread or be serialized
with it.

## Limits

- Std hosts only. The C library's clock and waits come from `lib/port/posix.c`. A `no_std` or embedded target
  needs a port of its own (the four functions in `lib/include/lr20xx_port.h`); that isn't done.
- `Device::open_linux` and the Linux HAL are built and bound on Linux only, and haven't run on hardware yet.
- The tests run the C driver and receive session against a chip scripted in Rust; nothing here has run on a
  board.
