# The rename from RadioFoundry Gateway's lr2021 component to this library (tools/upstream/README.md).
# import.sh runs every "sed" file of files.txt through it; keep it mechanical, so a fresh import diffs clean.

# Copyright: the same author, named.
s/SPDX-FileCopyrightText: 2026 The RadioFoundry Gateway authors/SPDX-FileCopyrightText: 2026 Spencer Fowler/

# Paths the comments cite.
s#docs/spec/lr2021-iq\.md#docs/iq.md#g
s#components/lr2021#lib#g

# ESP-IDF's error type and codes, and the few calls the driver-level files make (lib/include/lr20xx_port.h).
s/esp_err_to_name/lr20xx_err_name/g
s/esp_err_t/lr20xx_err_t/g
s/ESP_OK/LR20XX_OK/g
s/ESP_FAIL/LR20XX_FAIL/g
s/ESP_ERR_/LR20XX_ERR_/g
s/esp_timer_get_time()/lr20xx_now_us()/g
s/esp_rom_delay_us(/lr20xx_delay_us(/g
s/vTaskDelay(1)/lr20xx_sleep_tick()/g
s/portTICK_PERIOD_MS/LR20XX_TICK_MS/g
s/(esp_timer)/(lr20xx_now_us)/g
/#include "esp_rom_sys\.h"/d
/#include "freertos\/.*"/d
s/#include "esp_timer\.h"/#include "lr20xx_port.h"/

# The tests' scripted chip: the mock HAL in place of the ESP-IDF shim.
s/#include "esp_shim\.h"/#include "mock_hal.h"/
s/lr2021_config_t c = {\.spi_host = 1, \.sck = 1, \.mosi = 2, \.miso = 3, \.nss = 4, \.busy = 5, \.nreset = 6, \.irq = -1};/lr20xx_config_t c = mock_config();/

# Names: the family, not the part. "LR2021" alone (the chip) stays.
s/lr2021/lr20xx/g
s/LR2021_/LR20XX_/g
