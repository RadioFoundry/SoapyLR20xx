# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Spencer Fowler
# The LR20xx bridge: an ESP32-S3 that reads an LR20xx's raw I/Q and streams it to a host (README.md).
#   idf.py -B ../build/fw-w12 -D SDKCONFIG=../build/fw-w12/sdkconfig \
#          -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.w12" build
cmake_minimum_required(VERSION 3.16)
# The library (lib/: component "lib") and the bridge link (proto/: component "proto").
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/../lib ${CMAKE_CURRENT_LIST_DIR}/../proto)
set(PROJECT_VER "0.1.0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lr20xx_bridge)
