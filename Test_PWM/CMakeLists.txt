# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Test_PWM)

target_sources(app PRIVATE src/main.cpp)
include_directories(src)
include_directories(boards)
file(GLOB_RECURSE SRCS src/**.cpp)
target_sources(app PRIVATE ${SRCS})