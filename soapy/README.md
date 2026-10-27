# soapy

The SoapySDR module, `driver=lr20xx` (`lr20xxSupport`). One device class over two backends:

- **link**: a bridge (`firmware/`) that reads the chip and streams its samples, over TCP or a USB serial port, in
  the frames of `proto/include/lr20xx_link.h`. Any platform.
- **local**: a chip on this machine's spidev and GPIO lines, driven through `lib/` (`lr20xx_iq.h`) with a reader
  thread at `SCHED_FIFO` when allowed. Linux only.

| File | What |
|---|---|
| `SoapyLR20xx.hpp` | the Device, the backend interface, the sample queue |
| `SoapyLR20xx.cpp` | the Device and the module's registration |
| `Backends.cpp` | the link (TCP, serial), the local backend, UDP discovery, the rate lists |
| `test/test_fake_bridge.py` | end to end against `tools/fake_bridge/` (ctest `soapy_fake_bridge`) |

## Build

Part of the top-level build when SoapySDR is installed (Debian / Ubuntu `libsoapysdr-dev`, Homebrew `soapysdr`), or
on its own:

```sh
cmake -S soapy -B build/soapy && cmake --build build/soapy && ctest --test-dir build/soapy
SOAPY_SDR_PLUGIN_PATH=$PWD/build/soapy SoapySDRUtil --probe="driver=lr20xx,host=192.0.2.10"
```

`cmake --install` puts the module in SoapySDR's module directory.

## Arguments

| Argument | Backend | |
|---|---|---|
| `host=<addr>[:port]` | link | TCP, port 7373 by default |
| `serial=<path>` | link | a serial port (`/dev/ttyACM0`, `/dev/cu.usbmodem…`, `/dev/ttyUSB0`) |
| `baud=<rate>` | link | a UART's rate, the bridge's `CONFIG_LR20XX_SERIAL_BAUD` (a USB port needs none) |
| (none) | link | the first bridge that answers a UDP broadcast on port 7374 within 300 ms, sent on every interface |
| `discover=<addr>` | link | send the discovery there alone instead of the broadcasts |
| `spidev=<path>` | local | `/dev/spidev0.0` |
| `gpiochip=<path>` | local | `/dev/gpiochip0` by default |
| `busy=<line>`, `nreset=<line>` | local | GPIO line offsets, both needed |
| `irq=<line>`, `pa_en=<line>`, `speed=<Hz>` | local | optional: the IRQ DIO's line, a front-end supply driven high, the SPI clock |
| `board=<text>` | local | `lr20xx_board_parse` text: `w12`, `generic tcxo=3 irq=7`, … |
| `allow_untested=true` | local | run on a ROM not in `lr20xx_chips.h` |

Settings named in the arguments (`iq_mode=conv8`, `tx_mode=freq`, …) are applied when the device opens.

## Receive

The chip's own rate, not a resampled one: `setSampleRate` picks a mode and `getSampleRate` returns the rate it
gives (docs/iq.md section 2; `lr20xx_iq.h`'s table).

| `iq_mode` | Native format | Rates |
|---|---|---|
| `auto` (default) | CS16 up to 260 kS/s, then CS8 | `fsk16`'s list, then `conv8`'s range |
| `lora` | CS16 | 125 000, 250 000 |
| `fsk16` | CS16 | the FSK rate codes', up to 260 kS/s |
| `conv8` | CS8 | any rate the converter makes, up to 576 kS/s |

CS8's full scale is 32: the chip's int8 clips at −32..+31 (R2.26). CF32 is scaled by the native full scale.
`readStream` gives each buffer a time from its first sample's index in the stream (`SOAPY_SDR_HAS_TIME`), so a
loss shows as a jump; the loss itself is returned once as `SOAPY_SDR_OVERFLOW` before the samples after it.

Gain: one element, `LNA`, −8 to 53.5 dB in the chip's 13 steps (R2.25), set to the nearest; `setGainMode(true)` is
the chip's AGC. Until a gain or the AGC is set, it's 53.5 dB for CS16 and 9 dB for CS8: int8's 6 bits clip on the
noise alone from about 28 dB, and under the AGC (R2.33), which logs a warning in CS8. `setFrequencyCorrection` takes ppm. Sensor `temperature` and the `LR20XX` register interface
(ReadRegMem32 / WriteRegMem32) work only while not receiving.

## Transmit

Half duplex, by bursts ending in `SOAPY_SDR_END_BURST` (the receiver pauses for each). Each `writeStream` is
converted and sent as it comes: the bridge keys up once it has 0.2 s of samples (71 ms after the first over USB at
125 kS/s), and a burst can be any length, `writeStream` blocking while the bridge's buffer is full. The
`writeStream` with `SOAPY_SDR_END_BURST` returns once the burst has gone out; `readStreamStatus` then gives
`SOAPY_SDR_END_BURST`, `SOAPY_SDR_UNDERFLOW` if the chip's FIFO ran dry (R3.13: write faster than the rate), or
`SOAPY_SDR_TIMEOUT`. A bridge from before streamed transmissions gets the burst whole at the end, and refuses one
longer than it buffers (HELLO's `tx_max_samples`).

| `tx_mode` | Sent | |
|---|---|---|
| `iq_polar` (default) | int8 I/Q, phase and magnitude | `lr20xx_polar_iq8`, magnitudes under `polar_rmin` (0.2) pushed out to it (R3.27) |
| `iq_phase` | int8 I/Q, the phase at constant power | `lr20xx_phase_iq8` (R3.22) |
| `freq` | int8 frequency, a count `tx_fdev` / 64 Hz | the input's instantaneous frequency: its phase step a sample (R3.10) |
| `phase_step` | int8 phase steps, a count rate / 128 Hz | the same frequency (R3.18) |

The TX rate is `setSampleRate(TX)` or `tx_rate`, 31.25 to 250 kS/s (R3.10); the TX gain is the chip's PA power in
dBm, −9.5 up to the board's most. A board's own front end comes after it: the W12's GC1109 adds about 12 dB (up to
about +27 dBm at the antenna) between 860 and 930 MHz, and takes about 19 dB off elsewhere (R3.44).

What each mode sends right, as `getBandwidth(TX)` reports it (twice the limit), with a warning the first time a
burst goes past it:

- `iq_polar`, `iq_phase`: an instantaneous frequency up to ±min(20 kHz, 0.31 × rate) (R3.25); past it the chip
  sends some other frequency. Each sample is held, so a tone at f has images at f ± rate, about −23 to −29 dBc at
  48–96 kS/s (R3.26).
- `freq`: ±127 counts of `tx_fdev` / 64, ±4.06 kHz at the default 2048: raise `tx_fdev` for more.
- `phase_step`: ±127 / 128 of the rate.

A burst goes out as runs back to back, with a dip on air between them (R3.15, R3.44, R3.45): about 85 µs (iq_polar)
or 105–135 µs (iq_phase) on a board whose IRQ pin is wired (the W12), up to 0.5 ms without one. I/Q runs are at
most 32 767 samples (R3.43), the others 65 535.

## A USB bridge on Linux

The bridge's USB port gives a network (NCM), a serial port (CDC ACM), or both, as its firmware is built.

**Network.** The kernel's `cdc_ncm` driver makes it a USB Ethernet adapter, named `enx` and its MAC. The bridge
gives the host an address by DHCP on a /30 of its own in 10.73.0.0/16 (the bridge the first, the host the second),
with no router, so the default route stays where it was. On Ubuntu Desktop, NetworkManager does it unasked: a
"Wired connection" comes up with the address in a few seconds. Ubuntu Server leaves the adapter down; netplan brings
it up in about 5 s. The bridge's lease may name it as a DNS server, which answers nothing; `use-dns: false` keeps
the host from asking it (NetworkManager: `nmcli connection modify <name> ipv4.ignore-auto-dns yes`).

```yaml
# /etc/netplan/60-lr20xx.yaml (chmod 600, then sudo netplan apply)
network:
  version: 2
  ethernets:
    lr20xx:
      match:
        driver: cdc_ncm
      dhcp4: true
      dhcp4-overrides:
        use-dns: false
      dhcp6: false
      link-local: []
      optional: true
```

Discovery finds it with no arguments: it goes out on every interface. The bridge is the address below the host's
(`ip -4 addr show`): `host=10.73.x.y`. Its console's `status` prints it too.

**Serial.** `/dev/ttyACM0` and up, group `dialout` (`sudo usermod -aG dialout $USER`, then log in again). On a
machine with ModemManager (Ubuntu has it), a new ACM port next to a network port gets probed as a modem for a few
seconds. [`tools/udev/60-lr20xx.rules`](../tools/udev/60-lr20xx.rules) keeps ModemManager off Espressif's devices
(USB vendor 303a) and gives the logged-in user the serial ports:

```sh
sudo cp tools/udev/60-lr20xx.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger
```

Then `serial=/dev/ttyACM0`.

## Tested

Against `tools/fake_bridge/` (`ctest -R soapy_fake_bridge`), in CI on Ubuntu 24.04 and macOS: discovery, find,
probe, a capture (the tone, timestamps, the overflow), conv8's format, registers, the sensor, a freq-mode burst.
On Ubuntu 22.04 (arm64) also discovery of a fake bridge behind a second interface, given its address by
NetworkManager and by systemd-networkd from a DHCP server with no router.

Against a W12 bridge over USB (the network adapter, and the USB Serial/JTAG port), on macOS and on Ubuntu 22.04
(arm64): discovery, probe, and receive at 125, 250, 400 and 576 kS/s with every sample delivered; closing and
reopening, after a clean close, Ctrl-C or a kill. Over Wi-Fi from the Mac: 125 kS/s and 400 kS/s int8 whole; 250
kS/s int16 and 576 kS/s int8 past that link (`firmware/README.md`).

Transmit on the W12 from Linux, against the bench's receiver a room away (915 MHz, 0 dBm; 461.4625 MHz through the
board's 915 MHz filter at 22 dBm): every mode at 48–125 kS/s, tones where they belong inside the mode's limit (the
carrier on frequency with the board's ppm set), halved past it (R3.25), images at the rate (R3.26); bursts of 1–5 s
at 48–125 kS/s their exact length, with seams of about 85 µs between runs (R3.45). Not yet: the local backend (it needs a chip on a Linux
board's SPI bus).

From Python, SoapySDR 0.8.1 on ARM (Raspberry Pi OS / Debian 12, an Apple Silicon Mac) loses a dict's values in
`SoapySDR.Device({...})` ([SoapySDR#472](https://github.com/pothosware/SoapySDR/issues/472), fixed in 0.9.0);
there, pass a string: `SoapySDR.Device("driver=lr20xx,host=...")`.
