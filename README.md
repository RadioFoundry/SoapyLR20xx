# SoapyLR20xx

**A SoapySDR driver for the Semtech LR20xx: a LoRa transceiver used as a software-defined radio, on Linux and
macOS.**

The LR2021 has an undocumented path that streams its raw baseband, 16-bit I and Q, out of its FIFO, and another
that takes samples in for transmit. This project turns that into a SoapySDR device, so GNU Radio, SDR++, CubicSDR,
`rx_tools` and anything else on SoapySDR can use the chip as an SDR.

Status: experimental. The I/Q paths were found and measured on the LR2021 with ROM 1.24 (on a LilyGO T-Display-P4
and a Meshnology W12). Receiving works end to end on a W12 bridge: every rate up to 576 kS/s streams into SoapySDR
over USB without loss, on macOS and on Linux. Transmit, Wi-Fi streaming and the Linux SPI backend are not yet tried on hardware.

## How it fits together

The chip's receive FIFO holds 1 KB, 2 ms of samples at 250 kS/s. Whatever reads it has to sit on its SPI bus. So
there are two ways to reach it:

- **A bridge**: a microcontroller reads the chip and streams the samples to the host: over Wi-Fi, over USB as a
  network adapter (an ESP32-S3) or a serial port (any ESP32 with USB, or a UART). `firmware/` is that bridge; the
  Meshnology W12 is its first board.
- **Local**: the chip on a Linux machine's own SPI bus and GPIO lines (a Raspberry Pi, for one).

Both go through the same SoapySDR module and the same C library.

| Folder | What |
|---|---|
| `lib/` | `liblr20xx`: the chip's commands, an SPI driver over a small HAL, and the raw I/Q session. Portable C11 |
| `proto/` | the bridge link: the frames between host and bridge |
| `soapy/` | the SoapySDR module, `driver=lr20xx` (C++) |
| `firmware/` | the bridge firmware (ESP-IDF) |
| `rust/` | Rust crates: bindings to `lib/`, a safe API over it, and the link in pure Rust |
| `docs/` | `iq.md`: the hardware facts the code is written from; `link.md`: the bridge protocol |

The C library is the one implementation of the chip's logic. The SoapySDR module is C++ because SoapySDR's
module API is. The Rust crates bind the C rather than repeat it.

## What it can do

| | |
|---|---|
| **Frequency** | 150 to 960 MHz on the LF path (the datasheet's). The HF path (1.5 to 2.5 GHz) is untried for raw I/Q |
| **Receive** | CS16 at 125 to 260 kS/s, or CS8 at any rate up to 576 kS/s. See `docs/iq.md` section 2 |
| **Gain** | 13 steps, −8 to 53.5 dB, or the chip's AGC (not for CS8: R2.33) |
| **Over USB** | every receive rate without loss, on the W12 (`firmware/README.md`) |
| **Transmit** | half duplex, in bursts: polar I/Q (phase and amplitude), phase-only I/Q, or frequency samples. See `docs/iq.md` section 3 |

The limits that matter:

- The chip's int8 output clips at −32 and +31: 6 bits, about 20 dB above the noise. Use CS16 unless you need more
  than 260 kS/s. Above it the module starts at 9 dB of gain, since the AGC and the higher steps clip on the noise
  alone (R2.33).
- The chip's own filter passes about a third of the rate (CS16 at 250 kS/s passes about ±46 kHz).
- Transmit I/Q follows phase steps only up to about 20 kHz of instantaneous frequency (R3.25). Polar I/Q needs
  its magnitude kept off zero; the module does that (`polar_rmin`).
- One synthesizer: receiving stops while a burst goes out.

## Quick start

On a Mac or Linux, with SoapySDR installed:

```sh
cmake -S . -B build/host && cmake --build build/host && ctest --test-dir build/host
SOAPY_SDR_PLUGIN_PATH=$PWD/build/host/soapy SoapySDRUtil --find="driver=lr20xx"
```

Then flash a bridge (`firmware/README.md`) and plug it in or give it Wi-Fi; `--find` lists it. Or wire a chip to a
Linux board (`soapy/README.md`, arguments `spidev=`, `busy=`, `nreset=`).

## Chips and ROMs

GetVersion gives the ROM's version, not the part. `lib/src/lr20xx_chips.c` lists the versions the I/Q paths are
known on: today only 1.24, on the LR2021. Others are refused unless you pass `allow_untested=true`. If you run it on
another ROM, or on an LR2022 or LR2012, please open an issue with what you found.

## License

MIT (`LICENSE`). The documented commands are ported from Semtech's USP driver (Clear BSD), with its notice kept;
`PROVENANCE.md` lists what came from where. The raw I/Q facts were first found by SAMS0N1TE/lr2021-sdr; only facts
were taken from it, no code.
