# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Spencer Fowler
# The SoapySDR module (driver=lr20xx). Part of the top-level build, or on its own:
#   cmake -S soapy -B build/soapy && cmake --build build/soapy
cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(SoapyLR20xxModule VERSION 0.1.0 LANGUAGES C CXX)
    option(LR20XX_TESTS "Build the module's test" ON)
    if(LR20XX_TESTS)
        enable_testing()
    endif()
endif()
find_package(SoapySDR CONFIG REQUIRED)

set(LR20XX_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
if(NOT TARGET lr20xx_link)
    add_subdirectory(${LR20XX_ROOT}/proto ${CMAKE_CURRENT_BINARY_DIR}/proto)
endif()
if(NOT TARGET lr20xx_portable)
    # On its own: the encodings the module uses (rates and plans, transmit samples, GetTemp), without the rest of lib/.
    add_library(lr20xx_portable STATIC ${LR20XX_ROOT}/lib/src/lr20xx_cmd.c ${LR20XX_ROOT}/lib/src/lr20xx_tx.c
        ${LR20XX_ROOT}/lib/src/lr20xx_radio.c ${LR20XX_ROOT}/lib/src/lr20xx_iq_plan.c)
    target_include_directories(lr20xx_portable PUBLIC ${LR20XX_ROOT}/lib/include)
    target_link_libraries(lr20xx_portable PUBLIC m)
    set_target_properties(lr20xx_portable PROPERTIES C_STANDARD 11 POSITION_INDEPENDENT_CODE ON)
endif()

set(libs lr20xx_link lr20xx_portable)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND TARGET lr20xx)
    list(APPEND libs lr20xx Threads::Threads) # the local backend: spidev and GPIO through lib/
    find_package(Threads REQUIRED)
endif()

SOAPY_SDR_MODULE_UTIL(
    TARGET lr20xxSupport
    SOURCES SoapyLR20xx.cpp Backends.cpp
    LIBRARIES ${libs})
set_target_properties(lr20xxSupport PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_options(lr20xxSupport PRIVATE -Wall -Wextra -Wno-unused-parameter)

if(LR20XX_TESTS)
    # End to end against tools/fake_bridge: needs a python3 with SoapySDR's bindings; skipped (77) without one.
    set(pythons ${Python3_EXECUTABLE} python3 python3.15 python3.14 python3.13 python3.12)
    set(LR20XX_TEST_PYTHON "")
    foreach(py ${pythons})
        if(py AND NOT LR20XX_TEST_PYTHON)
            execute_process(COMMAND ${py} -c "import SoapySDR" RESULT_VARIABLE r OUTPUT_QUIET ERROR_QUIET)
            if(r EQUAL 0)
                set(LR20XX_TEST_PYTHON ${py})
            endif()
        endif()
    endforeach()
    if(LR20XX_TEST_PYTHON)
        add_test(NAME soapy_fake_bridge
            COMMAND ${LR20XX_TEST_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_fake_bridge.py
                    --module-dir $<TARGET_FILE_DIR:lr20xxSupport>
                    --bridge ${LR20XX_ROOT}/tools/fake_bridge/fake_bridge.py)
        set_tests_properties(soapy_fake_bridge PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)
    else()
        add_test(NAME soapy_fake_bridge COMMAND ${CMAKE_COMMAND} -E echo "no python3 with SoapySDR: skipped")
        set_tests_properties(soapy_fake_bridge PROPERTIES SKIP_REGULAR_EXPRESSION "skipped")
    endif()
endif()
