// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/* The library's clock and waits on Linux and macOS (lr20xx_port.h). */
#define _POSIX_C_SOURCE 200809L
#include <time.h>

#include "lr20xx_port.h"

int64_t lr20xx_now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000 + t.tv_nsec / 1000;
}

void lr20xx_delay_us(uint32_t us)
{
    /* spin: a sleep this short overshoots by tens of microseconds */
    int64_t end = lr20xx_now_us() + us;
    while (lr20xx_now_us() < end) {
    }
}

void lr20xx_sleep_ms(int ms)
{
    struct timespec t = {ms / 1000, (long)(ms % 1000) * 1000000};
    while (nanosleep(&t, &t)) {
    }
}

void lr20xx_sleep_us(uint32_t us)
{
    struct timespec t = {us / 1000000, (long)(us % 1000000) * 1000};
    while (nanosleep(&t, &t)) {
    }
}

void lr20xx_sleep_tick(void) { lr20xx_sleep_ms(LR20XX_TICK_MS); }
