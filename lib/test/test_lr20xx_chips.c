// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
// Host tests of the ROM table and the board profiles (lr20xx_chips.h, lr20xx_board.h).
#include <stdio.h>
#include <string.h>

#include "lr20xx_board.h"
#include "lr20xx_chips.h"

static int checks, fails;

#define CHECK(cond, ...)                                \
    do {                                                \
        checks++;                                       \
        if (!(cond)) {                                  \
            fails++;                                    \
            printf("FAIL %s:%d: ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                        \
            printf("\n");                               \
        }                                               \
    } while (0)

static void roms(void)
{
    const lr20xx_rom_t *r = lr20xx_rom_find(0x01, 0x18);
    CHECK(r && r->iq_rx && r->iq_tx && !strcmp(r->part, "LR2021"), "1.24 is the LR2021's, both ways");
    CHECK(!lr20xx_rom_find(0x01, 0x19) && !lr20xx_rom_find(0x02, 0x18), "others unknown");
    int n = 0;
    CHECK(lr20xx_roms(&n) && n >= 1, "the table");
}

static void boards(void)
{
    const lr20xx_board_t *w = lr20xx_board_find("w12");
    CHECK(w && w->irq_dio == 7 && w->rfsw_n == 3 && w->tcxo_tune == -1, "the W12");
    CHECK(lr20xx_board_find("generic") == &lr20xx_board_generic && !lr20xx_board_find("x"), "by name");
    uint8_t dio[4], m[4];
    CHECK(lr20xx_board_rfsw(w, 915e6, dio, m) == 3 && dio[0] == 9 && m[0] == 4 && m[1] == 4 && m[2] == 6,
          "W12 in its PA band: CTX, CPS on in TX; CSD in RX and TX");
    CHECK(lr20xx_board_rfsw(w, 461e6, dio, m) == 3 && m[0] == 4 && m[1] == 0 && m[2] == 6, "outside: the bypass");
    CHECK(lr20xx_board_rfsw(&lr20xx_board_generic, 915e6, dio, m) == 0, "generic: no switch");

    lr20xx_board_buf_t b;
    char err[96] = "";
    CHECK(!lr20xx_board_parse("w12", &b, err, sizeof err) && b.b.irq_dio == 7 && !strcmp(b.b.name, "w12"), "a name");
    CHECK(!lr20xx_board_parse("w12 irq=-1 boost=2", &b, err, sizeof err) && b.b.irq_dio == -1 && b.b.rx_boost == 2 &&
              b.b.rfsw_n == 3,
          "a name and changes");
    CHECK(!lr20xx_board_parse("generic,tcxo=3:2000,rfsw=9:0x4:0x4,rfsw=10:4:0,pa=860e6:930e6,range=150e6:960e6",
                              &b, err, sizeof err),
          "everything (%s)", err);
    CHECK(b.b.tcxo_tune == 3 && b.b.tcxo_start_us == 2000 && b.b.rfsw_n == 2 && b.b.rfsw_dio[1] == 10 &&
              b.b.rfsw_bypass[1] == 0 && b.b.pa_lo_hz == 860e6 && b.b.freq_max == 960e6,
          "parsed");
    CHECK(!lr20xx_board_parse("irq=8", &b, err, sizeof err) && b.b.irq_dio == 8 && !strcmp(b.b.name, "custom"),
          "changes alone: on generic");
    CHECK(lr20xx_board_parse("nope", &b, err, sizeof err) && strstr(err, "nope"), "unknown board: %s", err);
    CHECK(lr20xx_board_parse("w12 irq=3", &b, err, sizeof err) && strstr(err, "irq"), "a DIO out of range: %s", err);
    CHECK(lr20xx_board_parse("w12 tcxo", &b, err, sizeof err), "no value");
    CHECK(lr20xx_board_parse("", &b, err, sizeof err), "empty");
    CHECK(lr20xx_board_parse("generic rfsw=9:1:1 rfsw=10:1:1 rfsw=11:1:1 rfsw=5:1:1 rfsw=6:1:1", &b, err, sizeof err),
          "too many switches");
}

int main(void)
{
    roms();
    boards();
    printf("lr20xx chips and boards: %d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
