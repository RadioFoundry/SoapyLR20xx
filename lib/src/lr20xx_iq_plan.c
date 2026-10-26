// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/* The receive modes' rates (lr20xx_iq.h): no I/O, so a host that drives the chip through a bridge plans as the
 * bridge does. */
#include <math.h>
#include <string.h>

#include "lr20xx_iq.h"

const double lr20xx_iq_gain_db[13] = {-8, -2, 4, 9, 14, 18.25, 22.5, 28, 32.5, 38, 43.25, 47.5, 53.5};

lr20xx_err_t lr20xx_iq_plan(double rate, lr20xx_iq_mode_t mode, lr20xx_iq_plan_t *p)
{
    memset(p, 0, sizeof *p);
    if (!(rate > 0))
        return LR20XX_ERR_INVALID_ARG;
    if (mode == LR20XX_IQ_AUTO)
        mode = rate <= LR20XX_IQ_FSK16_MAX ? LR20XX_IQ_FSK16 : LR20XX_IQ_CONV8;
    p->mode = mode;
    switch (mode) {
    case LR20XX_IQ_LORA:
        /* R2.1, R2.8: 4 x the bandwidth, measured (to about 0.5%), not crystal-exact; R2.10: the filter passes
         * about the bandwidth */
        p->lora_bw = rate < 187500 ? LR20XX_LORA_BW_31 : LR20XX_LORA_BW_62;
        p->rate_num = lr20xx_raw_rx_rate(p->lora_bw);
        p->rate_den = 1;
        p->format = LR20XX_IQ_CS16;
        p->sample_bytes = 4;
        p->passband_hz = p->rate_num / 4.0;
        break;
    case LR20XX_IQ_FSK16: {
        /* R2.19, R2.20: source 2 at the code's own rate, crystal-exact */
        int code = lr20xx_fsk_rx_bw_for(rate, LR20XX_IQ_FSK16_MAX);
        if (code < 0)
            return LR20XX_ERR_NOT_SUPPORTED;
        p->rx_bw = (uint8_t)code;
        lr20xx_fsk_rx_rate(p->rx_bw, &p->rate_num, &p->rate_den);
        p->format = LR20XX_IQ_CS16;
        p->sample_bytes = lr20xx_raw_sample_bytes(LR20XX_RAW_SRC_CS16);
        p->passband_hz = (double)p->rate_num / p->rate_den * lr20xx_fsk_filter_share(p->rx_bw);
        break;
    }
    case LR20XX_IQ_CONV8: {
        /* R2.21: the converter makes exactly the rate asked, from the fastest code whose filter stays within 0.8
         * of it */
        if (rate > LR20XX_IQ_CONV8_MAX || rate != floor(rate))
            return LR20XX_ERR_NOT_SUPPORTED;
        int code = lr20xx_conv_rx_bw_for(rate, 0.8);
        if (code < 0)
            return LR20XX_ERR_NOT_SUPPORTED;
        p->rx_bw = (uint8_t)code;
        uint32_t num, den;
        lr20xx_fsk_rx_rate(p->rx_bw, &num, &den);
        p->chip_rate = (double)num / den;
        p->rate_num = (uint32_t)rate;
        p->rate_den = 1;
        p->format = LR20XX_IQ_CS8;
        p->sample_bytes = lr20xx_raw_sample_bytes(LR20XX_RAW_SRC_CONV);
        p->passband_hz = p->chip_rate * lr20xx_fsk_filter_share(p->rx_bw);
        break;
    }
    default:
        return LR20XX_ERR_INVALID_ARG;
    }
    p->rate = (double)p->rate_num / p->rate_den;
    return LR20XX_OK;
}

int lr20xx_iq_rates(lr20xx_iq_mode_t mode, double *rates, int max, double *lo, double *hi)
{
    int n = 0;
    if (lo)
        *lo = 0;
    if (hi)
        *hi = 0;
    if (mode == LR20XX_IQ_LORA) {
        static const double r[2] = {125000, 250000};
        for (; n < 2 && n < max; n++)
            rates[n] = r[n];
        if (lo)
            *lo = 125000;
        if (hi)
            *hi = 250000;
    } else if (mode == LR20XX_IQ_FSK16 || mode == LR20XX_IQ_AUTO) {
        double r = 1;
        for (;;) {
            int code = lr20xx_fsk_rx_bw_for(r, LR20XX_IQ_FSK16_MAX);
            if (code < 0)
                break;
            uint32_t num, den;
            lr20xx_fsk_rx_rate((uint8_t)code, &num, &den);
            r = (double)num / den;
            if (n < max)
                rates[n] = r;
            n++;
            if (lo && n == 1)
                *lo = r;
            if (hi)
                *hi = r;
            r = nextafter(r, INFINITY);
        }
        if (n > max)
            n = max;
    }
    if (mode == LR20XX_IQ_CONV8 || mode == LR20XX_IQ_AUTO) {
        /* the lowest rate with a code (they're dense from there: the codes' rates are at most 2x apart) */
        double r = 1000;
        while (r < LR20XX_IQ_CONV8_MAX && lr20xx_conv_rx_bw_for(r, 0.8) < 0)
            r += 100;
        if (lo && (mode == LR20XX_IQ_CONV8 || r < *lo))
            *lo = r;
        if (hi)
            *hi = LR20XX_IQ_CONV8_MAX;
    }
    return n;
}

