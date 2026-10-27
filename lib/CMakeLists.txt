# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Spencer Fowler
# liblr20xx: a Semtech LR20xx over SPI, its command encodings and its raw I/Q session. An ESP-IDF component when
# ESP_PLATFORM is set (with the ESP-IDF HAL and port); otherwise static libraries for Linux / macOS and the tests.
set(LR20XX_PORTABLE_SRCS src/lr20xx_cmd.c src/lr20xx_tx.c src/lr20xx_radio.c src/lr20xx_lora.c src/lr20xx_fsk.c
    src/lr20xx_proto.c src/lr20xx_chips.c src/lr20xx_board.c
    src/lr20xx_iq_plan.c)
set(LR20XX_DRIVER_SRCS src/lr20xx.c src/lr20xx_tx_stream.c src/lr20xx_packet.c src/lr20xx_iq.c)

if(ESP_PLATFORM)
    idf_component_register(
        SRCS ${LR20XX_PORTABLE_SRCS} ${LR20XX_DRIVER_SRCS} port/esp_idf.c hal/esp_idf.c
        INCLUDE_DIRS include
        LDFRAGMENTS linker.lf
        PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_timer)
    return()
endif()

cmake_minimum_required(VERSION 3.16)
project(lr20xx C)

function(lr20xx_options t)
    set_target_properties(${t} PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON POSITION_INDEPENDENT_CODE ON)
    target_compile_options(${t} PRIVATE -Wall -Wextra -Wno-unused-parameter)
endfunction()

# The encodings alone: no I/O, no port.
add_library(lr20xx_portable STATIC ${LR20XX_PORTABLE_SRCS})
target_include_directories(lr20xx_portable PUBLIC include)
target_link_libraries(lr20xx_portable PUBLIC m)
lr20xx_options(lr20xx_portable)

# The driver without a port or HAL: the tests link the mock, programs a port and a HAL.
add_library(lr20xx_driver STATIC ${LR20XX_DRIVER_SRCS})
target_link_libraries(lr20xx_driver PUBLIC lr20xx_portable)
lr20xx_options(lr20xx_driver)

# The library for Linux / macOS: the driver, the POSIX port, and on Linux the spidev / GPIO HAL. The driver's sources
# go in it, not lr20xx_driver: the driver calls the port, and a single-pass linker (GNU ld) won't go back for it.
add_library(lr20xx STATIC port/posix.c ${LR20XX_DRIVER_SRCS})
target_link_libraries(lr20xx PUBLIC lr20xx_portable)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_sources(lr20xx PRIVATE hal/linux.c)
    target_compile_definitions(lr20xx PUBLIC LR20XX_HAL_LINUX=1)
endif()
lr20xx_options(lr20xx)

if(LR20XX_TESTS)
    add_library(lr20xx_mock STATIC test/mock_hal.c)
    target_include_directories(lr20xx_mock PUBLIC test)
    target_link_libraries(lr20xx_mock PUBLIC lr20xx_driver)
    lr20xx_options(lr20xx_mock)
    foreach(t lr20xx lr20xx_tx lr20xx_radio lr20xx_lora lr20xx_fsk lr20xx_proto lr20xx_chips)
        add_executable(test_${t} test/test_${t}.c)
        target_link_libraries(test_${t} PRIVATE lr20xx_portable)
        add_test(NAME ${t} COMMAND test_${t})
    endforeach()
    foreach(t lr20xx_driver lr20xx_tx_stream lr20xx_packet lr20xx_iq)
        add_executable(test_${t} test/test_${t}.c)
        target_link_libraries(test_${t} PRIVATE lr20xx_mock)
        add_test(NAME ${t} COMMAND test_${t})
    endforeach()
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        add_executable(test_lr20xx_hal_linux test/test_lr20xx_hal_linux.c)
        target_link_libraries(test_lr20xx_hal_linux PRIVATE lr20xx)
        lr20xx_options(test_lr20xx_hal_linux)
        add_test(NAME lr20xx_hal_linux COMMAND test_lr20xx_hal_linux)
    endif()
endif()
