# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/rpi_pico.overlay)

include_directories(include)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(step_gyro)

target_sources(app PRIVATE src/main.cpp)
include_directories(src)
file(GLOB_RECURSE SRCS src/**.cpp)
target_sources(app PRIVATE ${SRCS})
#include_directories(boards)