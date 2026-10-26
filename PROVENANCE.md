# Provenance

Where this project's code came from: what was taken from other projects and under which license. Keep a row per
use, and add one before copying or porting anything new. The license texts are in `LICENSES/`.

## Origin

The LR20xx library (`lib/`) began as the `lr2021` component of RadioFoundry Gateway
(github.com/RadioFoundry/RadioFoundryGateway, MIT), by the same author, imported on 2026-10-26 from commit
e2a85653aab8ef13a497ed2b8891f102fac711c4. `tools/upstream/files.txt` lists every file taken and how (renamed by a
script, or rewritten by hand); `tools/upstream/README.md` says how the two are kept in step. The raw I/Q session
(`lib/src/lr20xx_iq.c`) is ported from the gateway's LR2021 front end (`esp_idf/main/fe_lr2021.c`), and
`docs/iq.md` from its `docs/spec/lr2021-iq.md`.

The SoapySDR module (`soapy/`), the bridge link (`proto/`), the bridge firmware (`firmware/`), the Rust crates
(`rust/`) and the HAL, port and board code in `lib/` were written here.

## Third-party code

| Path | Upstream (repo @ commit) | License | What was taken |
|---|---|---|---|
| `lib/src/lr20xx_radio.c`, `lr20xx_lora.c`, `lr20xx_fsk.c`, `lr20xx_proto.c` and their headers (the documented commands) | Lora-net/usp @ 540bba008477a8969e554b8d10dbf90ff6160c8e (`smtc_rac_lib/radio_drivers/lr20xx_driver`) | Clear BSD (`LICENSES/BSD-3-Clause-Clear-Semtech-USP.txt`) | Command encodings, value tables and time-on-air formulas, ported (by way of RadioFoundry Gateway); the notice is at the top of each file. Checked against the datasheet (DS.LR20xx Rev. 2.2) |
| `lib/` raw I/Q receive and transmit (`lr20xx_cmd_raw_*`, `lr20xx_iq.c`, `lr20xx_tx.c`), `docs/iq.md` | SAMS0N1TE/lr2021-sdr @ 929d97a1b14b9563f418d6f4fd84a787a56e45aa | GPL-3.0 | Facts only (the hardware interface: opcodes, register addresses, argument values, their order, the sample format, rates, measured limits), cited in `docs/iq.md` as "SDR". No code, comments or code structure taken |

The LR2021's ROM has no license: nothing of it is here. `docs/iq.md` states facts about the chip learned from
measurement and from reading the ROM, as RadioFoundry Gateway's spec did.

## Dependencies

| What | License | Used by |
|---|---|---|
| SoapySDR (pothosware/SoapySDR) | Boost Software License 1.0 | `soapy/` links it; nothing copied |
| ESP-IDF (espressif/esp-idf) | Apache-2.0 | `firmware/`, `lib/hal/esp_idf.c`, `lib/port/esp_idf.c` link it; nothing copied |
| Rust crates in `rust/Cargo.lock` (cc, embedded-hal, if-addrs) | MIT / Apache-2.0 / BSD-3-Clause | `rust/`; `src/bindings*.rs` are bindgen's output from this project's headers |
| esp_tinyusb and TinyUSB (espressif/esp_tinyusb, hathach/tinyusb) | Apache-2.0 / MIT | `firmware/`'s USB links, from the component registry; nothing copied |
