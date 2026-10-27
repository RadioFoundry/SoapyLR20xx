// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * The HAL on Linux (hal/linux.c): the chip on a spidev device (NSS its chip select), BUSY, NRESET and the IRQ
 * on GPIO lines of a GPIO character device (the kernel's v2 line API, no libgpiod). Built on Linux only
 * (LR20XX_HAL_LINUX).
 *
 * The FIFO lasts 2 ms at 125 kS/s int16: run the reader thread with a real-time priority (SCHED_FIFO) and, on a
 * Raspberry Pi, keep the SPI core clock fixed so the bus runs at the speed asked.
 */
#ifndef LR20XX_HAL_LINUX_H
#define LR20XX_HAL_LINUX_H

#include <stddef.h>

#include "lr20xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *spidev;   /* "/dev/spidev0.0" */
    int speed_hz;         /* 0: 16 MHz, the chip's most (DS 3.7.2) */
    const char *gpiochip; /* "/dev/gpiochip0" */
    int busy, nreset;     /* line offsets on gpiochip */
    int irq;              /* the line a DIO set to IRQ drives, -1 for none */
    int pa_en;            /* a line driven high while open (a front-end supply), -1 for none */
} lr20xx_linux_cfg_t;

/* Open the bus and lines into *hal (lr20xx_open takes it; lr20xx_close releases it). LR20XX_ERR_NOT_FOUND for a
 * missing device, LR20XX_FAIL for one that wouldn't take the setup (a line another program holds: EBUSY),
 * LR20XX_ERR_INVALID_ARG without busy and nreset; err (errlen bytes) says which. Needs Linux 5.10 or later (the
 * GPIO v2 API). */
lr20xx_err_t lr20xx_hal_linux_open(const lr20xx_linux_cfg_t *cfg, lr20xx_hal_t *hal, char *err, size_t errlen);

#ifdef __cplusplus
}
#endif

#endif
