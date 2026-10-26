// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/* The ROMs the raw paths are known on (lr20xx_chips.h). */
#include "lr20xx_chips.h"

#include <stddef.h>

static const lr20xx_rom_t roms[] = {
    /* docs/iq.md: R1.1, section 2 (receive), section 3 (transmit) */
    {0x01, 0x18, "LR2021", true, true, "ROM 1.24: a LilyGO T-Display-P4 (2026-10-18) and a Meshnology W12 (2026-10-24)"},
};

const lr20xx_rom_t *lr20xx_rom_find(uint8_t major, uint8_t minor)
{
    for (size_t i = 0; i < sizeof roms / sizeof roms[0]; i++)
        if (roms[i].major == major && roms[i].minor == minor)
            return &roms[i];
    return NULL;
}

const lr20xx_rom_t *lr20xx_roms(int *n)
{
    *n = (int)(sizeof roms / sizeof roms[0]);
    return roms;
}
