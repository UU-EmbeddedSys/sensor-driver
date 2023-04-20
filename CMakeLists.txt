# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

# For the sake of demonstration, we add the driver directory as a zephyr module
# by hand. If your driver is a project that's managed by west, you can remove this line.
# list(APPEND ZEPHYR_EXTRA_MODULES
#   ${CMAKE_CURRENT_SOURCE_DIR}/drivers/adxl345
#   ${CMAKE_CURRENT_SOURCE_DIR}/drivers/bme680
#   ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ultrasonic
#   )

# Select our board 
set(BOARD rpi_pico)

# For using west flash with picoprobe
set(OPENOCD "/usr/local/bin/openocd")
set(OPENOCD_DEFAULT_PATH "/usr/local/share/openocd/scripts")
set(RPI_PICO_DEBUG_ADAPTER "picoprobe")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})


# This is only used by IntelliSense inside VS Code 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Project name
project(sensor-driver LANGUAGES C VERSION 0.0.1)

# Add the include directory
zephyr_include_directories(include)

# Add source files
FILE(GLOB SRC_FILES "src/*.c")
target_sources(app PRIVATE ${SRC_FILES})
