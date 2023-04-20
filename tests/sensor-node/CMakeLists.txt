# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sensor-node-test)

zephyr_include_directories(../../include)

FILE(
    GLOB test_sources
    src/*.c
    ../../src/*.c
)

# Exclude the main project file from the list
list(FILTER test_sources EXCLUDE REGEX "main\\.c$")

# add our main in test
list(APPEND test_sources src/main.c)

# Exclude the main project file from the list
target_sources(app PRIVATE ${test_sources})