# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/rpi_pico.overlay)

include_directories(include)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Led_button)

target_sources(app PRIVATE src/main.cpp)
#include_directories(boards)