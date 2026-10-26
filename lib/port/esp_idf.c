// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/* The library's clock and waits on ESP-IDF (lr20xx_port.h). */
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lr20xx_port.h"

int64_t lr20xx_now_us(void) { return esp_timer_get_time(); }
void lr20xx_delay_us(uint32_t us) { esp_rom_delay_us(us); }
void lr20xx_sleep_tick(void) { vTaskDelay(1); }

void lr20xx_sleep_ms(int ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    vTaskDelay(t ? t : 1);
}

void lr20xx_sleep_us(uint32_t us)
{
    if (us >= (uint32_t)portTICK_PERIOD_MS * 1000)
        vTaskDelay((TickType_t)(us / 1000 / portTICK_PERIOD_MS));
    else
        esp_rom_delay_us(us);
}
