cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_CXX_STANDARD 17)
set(BOARD sst_pico)
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/rpi_pico.overlay)

include_directories(include)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(payload)

target_sources(app PRIVATE src/main.cpp)
include_directories(src)
file(GLOB_RECURSE SRCS src/**.cpp)
target_sources(app PRIVATE ${SRCS})


