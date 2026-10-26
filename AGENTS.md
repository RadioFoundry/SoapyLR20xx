# SoapyLR20xx

A SoapySDR driver for the Semtech LR20xx's undocumented raw I/Q paths: a C11 library (`lib/`), a bridge link
(`proto/`), the C++ SoapySDR module (`soapy/`), ESP32 bridge firmware (`firmware/`) and Rust crates (`rust/`).
[README.md](README.md) is the overview; [docs/iq.md](docs/iq.md) the hardware facts; [docs/link.md](docs/link.md)
the bridge protocol. Each folder has its own README.

## Rules

- Published under the RadioFoundry GitHub org, MIT. The user makes the first commit.
- One implementation of the chip's logic: the C in `lib/`. The module is C++ because SoapySDR's API is; Rust binds
  the C (`rust/lr20xx-sys`) and doesn't repeat it. `lr20xx-link` (the protocol in Rust) is the one exception, kept
  byte-for-byte equal to `proto/` by a cross-test.
- `lib/` began as RadioFoundry Gateway's `components/lr2021`. Files marked `sed` in `tools/upstream/files.txt` come
  from it through `transform.sed`; keep them close to upstream and log changes in `tools/upstream/README.md`, so
  fixes can go both ways (`tools/upstream/drift.sh`).
- Hardware facts go in `docs/iq.md` as numbered findings (R2.n, R3.n) with the board and date; code cites them.
  Never copy the LR2021's ROM: facts only. Facts from SAMS0N1TE/lr2021-sdr (GPL-3.0) are fine; its code is not.
- Licensing: track anything copied or ported in `PROVENANCE.md`, with its license in `LICENSES/`. Every source file
  starts with `SPDX-License-Identifier` and `SPDX-FileCopyrightText: 2026 Spencer Fowler`.
- Names: `lr20xx_` / `LR20XX_` in C, `lr20xx` crates, `driver=lr20xx` in SoapySDR. "LR2021" only for that part.
- US English. Prose in docs and comments: plain, direct, short sentences.
- Builds go under `build/` only.

## Tests

```sh
cmake -S . -B build/host && cmake --build build/host && ctest --test-dir build/host
cd rust && cargo fmt --check && CARGO_TARGET_DIR=../build/rust cargo clippy --all-features --all-targets -- -D warnings \
    && CARGO_TARGET_DIR=../build/rust cargo test --all-features
```

CI (`.github/workflows/ci.yml`) runs these on Linux and macOS with `-Werror`, the library under ASan and UBSan,
and every firmware variant.

New behavior gets a host test. Driver code is tested against the mock HAL (`lib/test/mock_hal.c`); the I/Q
session against the chip model in `lib/test/test_lr20xx_iq.c`.

## Firmware

ESP-IDF v5.5 (`. ~/esp/v5.5/esp-idf/export.sh`), from `firmware/`:

```sh
idf.py -B ../build/fw-w12 -D SDKCONFIG=../build/fw-w12/sdkconfig -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.w12" build
```

A USB link is one more overlay, last: `.ncm` (the network over USB), `.jtag` (serial on the USB Serial/JTAG port) or
`.usb` (serial on TinyUSB CDC), each in its own build dir (`build/fw-w12-ncm`, ...). Target settings are in
`sdkconfig.defaults.<target>`, which ESP-IDF adds itself; the C3 (`set-target esp32c3`, no board yet) is kept
building in CI. The protocol on the wire is `docs/link.md`; change it in `proto/`, `rust/lr20xx-link` (the
cross-test checks they agree), the module and `tools/fake_bridge/` together.

Lab notes (boards, addresses, who flashes what) go in `AGENTS.local.md`, which is git-ignored.
