# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Spencer Fowler
# lr20xx_link: the bridge link's encoding (include/lr20xx_link.h). An ESP-IDF component when ESP_PLATFORM is set.
if(ESP_PLATFORM)
    idf_component_register(SRCS src/lr20xx_link.c INCLUDE_DIRS include LDFRAGMENTS linker.lf)
    return()
endif()
add_library(lr20xx_link STATIC src/lr20xx_link.c)
target_include_directories(lr20xx_link PUBLIC include)
set_target_properties(lr20xx_link PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON POSITION_INDEPENDENT_CODE ON)
target_compile_options(lr20xx_link PRIVATE -Wall -Wextra)
if(LR20XX_TESTS)
    add_executable(test_lr20xx_link test/test_lr20xx_link.c)
    target_link_libraries(test_lr20xx_link PRIVATE lr20xx_link)
    add_test(NAME lr20xx_link COMMAND test_lr20xx_link)
endif()
