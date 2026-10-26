// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * What the library needs of the platform besides the chip's pins: error codes, a clock and waits. One port is
 * linked in: lib/port/posix.c (Linux, macOS), lib/port/esp_idf.c (ESP-IDF), or the tests' fake clock.
 *
 * The error codes keep ESP-IDF's values, so on ESP-IDF an lr20xx_err_t is an esp_err_t.
 */
#ifndef LR20XX_PORT_H
#define LR20XX_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int lr20xx_err_t;
#define LR20XX_OK 0
#define LR20XX_FAIL -1
#define LR20XX_ERR_NO_MEM 0x101
#define LR20XX_ERR_INVALID_ARG 0x102
#define LR20XX_ERR_INVALID_STATE 0x103
#define LR20XX_ERR_INVALID_SIZE 0x104
#define LR20XX_ERR_NOT_FOUND 0x105
#define LR20XX_ERR_NOT_SUPPORTED 0x106
#define LR20XX_ERR_TIMEOUT 0x107
#define LR20XX_ERR_INVALID_RESPONSE 0x108
#define LR20XX_ERR_INVALID_CRC 0x109

/* A name for an error code ("LR20XX_ERR_TIMEOUT"), for logs. */
const char *lr20xx_err_name(lr20xx_err_t e);

/* Microseconds from a fixed point (monotonic). */
int64_t lr20xx_now_us(void);
/* Wait us microseconds without giving up the CPU (short waits: BUSY polls, a pin's hold time). */
void lr20xx_delay_us(uint32_t us);
/* Give the CPU up for the scheduler's smallest sleep: one tick on FreeRTOS, LR20XX_TICK_MS elsewhere. */
void lr20xx_sleep_tick(void);
/* Sleep at least ms milliseconds. */
void lr20xx_sleep_ms(int ms);
/* Wait about us microseconds, giving the CPU up where the platform can for that long (POSIX: a sleep; FreeRTOS: a
 * tick if us is at least one, else a spin). */
void lr20xx_sleep_us(uint32_t us);

/* The length of lr20xx_sleep_tick, in ms. */
#ifndef LR20XX_TICK_MS
#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#define LR20XX_TICK_MS portTICK_PERIOD_MS
#else
#define LR20XX_TICK_MS 1
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif
