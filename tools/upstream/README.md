# Keeping in step with RadioFoundry Gateway

`lib/` and RadioFoundry Gateway's `components/lr2021` are the same code but for the name: every file in `files.txt`
marked `sed` differs only by the rename (`lr2021` to `lr20xx`, the copyright line, the spec's path). A change on
either side carries to the other mechanically. The gateway keeps a few files of its own (its patch RAM loader and
image, `lr2021_lora_extra.*`), and each side its own `CMakeLists.txt` and README.

| File | What |
|---|---|
| `files.txt` | every shared file: its gateway path, its path here, and how it's kept (`sed` or `hand`) |
| `transform.sed` | the rename, gateway to here |
| `export.sed` | its inverse, here to the gateway |
| `import.sh` | the `sed` files of a gateway tree through `transform.sed`, into a directory |
| `export.sh` | the `sed` files here through `export.sed`, into a gateway tree (it checks each comes back unchanged) |
| `drift.sh` | the two diffs below, from `git archive` (read-only: no checkout, no stash) |
| `BASE` | the gateway commit the two last matched at |

## Checking

```sh
tools/upstream/drift.sh [path/to/RadioFoundryGateway [rev]]
```

It writes `build/drift/upstream.diff` (the gateway's changes since BASE, renamed) and `build/drift/ours.diff` (ours
to the `sed` files since BASE). Both empty: the two match.

- **Bringing a gateway change here**: `tools/upstream/import.sh path/to/RadioFoundryGateway build/drift/in`, then
  copy the changed files from `build/drift/in/lib` over `lib/`. A `hand` file's change (the spec, CMake) shows as
  upstream's own diff under `build/drift/head/`: port it by hand.
- **Sending a change there**: `tools/upstream/export.sh path/to/RadioFoundryGateway` writes every `sed` file into the
  gateway's tree; commit it there.
- After either, move `BASE` to the gateway commit that now matches, and log it below.

## Log

What was carried across, newest first. Changes made here that the gateway may want go under "To offer".

### Kept out

Gateway code this library leaves out on purpose: its files aren't in `files.txt`, so neither script touches them.
The list is in `AGENTS.local.md`.

### To offer

Nothing: the two match at BASE.

### Carried

- 2026-10-27: everything here went to the gateway (its 43c5372): the HAL and port, `lr2021_iq`, the board and ROM
  tables, the transmit loop (R3.43-R3.48), the mock HAL tests. The gateway's front end runs on `lr2021_iq` now.
  New on the way: `slack_us` in the session config (the gateway reads with 600 us left before the FIFO fills), and
  the shared files made rename-clean (no paths or names only one side has). Both sides match since.
- 2026-10-26: imported at e2a8565.
