# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Spencer Fowler
# SoapyLR20xx on Linux / macOS: the library (lib/), the bridge link (proto/), the SoapySDR module (soapy/, when
# SoapySDR is installed) and the host tests.
#   cmake -S . -B build/host && cmake --build build/host && ctest --test-dir build/host
cmake_minimum_required(VERSION 3.16)
project(SoapyLR20xx VERSION 0.1.0 LANGUAGES C CXX)

option(LR20XX_TESTS "Build the host tests" ON)
option(LR20XX_SOAPY "Build the SoapySDR module (needs SoapySDR)" ON)
if(LR20XX_TESTS)
    enable_testing()
endif()

add_subdirectory(lib)
add_subdirectory(proto)
if(LR20XX_SOAPY)
    find_package(SoapySDR CONFIG QUIET)
    if(SoapySDR_FOUND)
        add_subdirectory(soapy)
    else()
        message(STATUS "SoapySDR not found: no SoapySDR module (Debian / Ubuntu libsoapysdr-dev, Homebrew soapysdr)")
    endif()
endif()
