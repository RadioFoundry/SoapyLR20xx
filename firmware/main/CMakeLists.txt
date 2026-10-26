# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Spencer Fowler
set(srcs main.c bridge.c link_tcp.c wifi.c http.c)
if(CONFIG_LR20XX_LINK_SERIAL)
    list(APPEND srcs link_serial.c)
endif()
if(CONFIG_LR20XX_SERIAL_TINYUSB)
    list(APPEND srcs link_usb.c)
endif()
if(CONFIG_LR20XX_USB_NCM)
    list(APPEND srcs link_ncm.c)
endif()
idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS .
    PRIV_REQUIRES lib proto nvs_flash esp_wifi esp_netif esp_event esp_timer console esp_http_server app_update
                  esp_app_format esp_driver_usb_serial_jtag esp_driver_uart lwip)
