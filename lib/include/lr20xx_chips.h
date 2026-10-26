// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * Which firmware (ROM) versions the raw I/Q paths are known to work on. The paths are undocumented (docs/iq.md) and
 * found on the LR2021 with ROM 1.24; another ROM, or the LR2022 / LR2012, may lay them out differently. GetVersion
 * gives the ROM's version only, not the part, so the table is by version.
 *
 * Add a row once a ROM has been run on a board (and its findings are in docs/iq.md).
 */
#ifndef LR20XX_CHIPS_H
#define LR20XX_CHIPS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t major, minor; /* GetVersion */
    const char *part;     /* the part it was found on */
    bool iq_rx;           /* raw I/Q receive works (docs/iq.md section 2) */
    bool iq_tx;           /* raw transmit works (docs/iq.md section 3) */
    const char *tested;   /* where and when */
} lr20xx_rom_t;

/* The row for a version, or NULL for one not run yet. */
const lr20xx_rom_t *lr20xx_rom_find(uint8_t major, uint8_t minor);
/* Every row, *n of them. */
const lr20xx_rom_t *lr20xx_roms(int *n);

#ifdef __cplusplus
}
#endif

#endif
