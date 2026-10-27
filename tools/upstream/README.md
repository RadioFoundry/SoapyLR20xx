# Keeping in step with RadioFoundry Gateway

`lib/` began as RadioFoundry Gateway's `components/lr2021` (PROVENANCE.md). The two drift on purpose: each project
changes its copy as it needs. What's here keeps the drift visible, so a fix on either side can be carried across.

| File | What |
|---|---|
| `files.txt` | every imported file: its upstream path, its path here, and how it came (`sed` or `hand`) |
| `transform.sed` | the rename (`lr2021` to `lr20xx`, ESP-IDF's error codes and delays to `lr20xx_port.h`'s, the copyright line) |
| `import.sh` | runs the `sed` files of a gateway tree through the transform |
| `drift.sh` | the two diffs below, from `git archive` (read-only: no checkout, no stash) |
| `BASE` | the gateway commit last brought in |

## Checking

```sh
tools/upstream/drift.sh [path/to/RadioFoundryGateway [rev]]
```

It writes `build/drift/upstream.diff` (the gateway's changes since BASE, renamed) and `build/drift/ours.diff` (ours
to the `sed` files since BASE). A `sed` file that nobody changed diffs empty both ways.

- **Bringing a gateway change here**: a `sed` file's hunk applies nearly as it is (`patch -p1 -d build/drift/ours`
  shows it; copy the result back). A `hand` file's (`lr2021.c`, the tests' shim, `fe_lr2021.c`, the spec) shows as
  upstream's own diff under `build/drift/head/`: port it by hand.
- **Offering a change upstream**: `ours.diff` is in this project's names. Reverse the rename (`lr20xx` to `lr2021`,
  `LR20XX_` to `LR2021_`, the error codes back to ESP-IDF's) before applying it there.
- After either, move `BASE` to the gateway commit that now matches, and log it below.

## Log

What was carried across, newest first. Changes made here that the gateway may want go under "To offer".

### Kept out

Gateway code this library leaves out on purpose; take it out again when bringing changes over. The list is in
`AGENTS.local.md`.

### To offer

- `lr20xx.c`: the driver behind a HAL (`lr20xx_hal.h`) and a port (`lr20xx_port.h`), so it runs on Linux too.
  A larger change than the gateway needs; offer if it wants a host build of the driver.
- `lr20xx_iq.c`: the session, reader and transmit run as a portable library, out of `fe_lr2021.c`.
- `lr20xx_tx_max_run` and R3.43: an I/Q run's bytes are a 16-bit count, so I/Q runs are at most 32 767 samples
  (the gateway's `LR2021_TX_MAX_RUN` is right for its frequency samples only), and the spec's R3.12 should no
  longer call R3.5's cap wrong.
- R3.44: the W12's GC1109 held in TX between runs (`rf_hold_tx`), fallback FS, SetTx before clearing TX_DONE; and
  the W12's antenna power (the chip's + 12 dB in the PA band).
- `lr20xx_tx_stream.c`, the transmit loop: the next run's SetTx on the IRQ pin's edge, the FIFO level reckoned from
  the rate between reads, refills held near a run's end (R3.45); streamed transmissions from a pull callback, the
  last run cut once the FIFO is empty (R3.46); the next SetTx sent ahead of the predicted TX_DONE (R3.47).
- `lr20xx_tx_stream.c`, timed key-up: `at_us` (esp_timer's clock), the SetTx frame sent its learned duration early
  (key-up within +-5 us), `LR20XX_TX_LATE_US` (ready more than 100 us past it: nothing sent, `late`), `stop` seen
  during the wait, `ready_us` / `key_us` in the stats. For the repeater's or roamer's TDMA slots.
- `lr20xx_iq.c`: the CalibFE kept across transmissions until a reset (`cal_word`, R3.48), so a burst at the cell
  last calibrated sets up in 2.7 ms instead of 12.6 ms; `lr20xx_iq_tx_calibrates` says which a burst needs.

### Carried

- 2026-10-26: imported at e2a8565.
