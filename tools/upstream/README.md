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

### Carried

- 2026-10-26: imported at e2a8565.
