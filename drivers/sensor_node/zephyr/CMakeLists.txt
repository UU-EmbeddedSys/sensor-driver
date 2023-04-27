# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SENSOR_NODE)
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    sensor_node.c
    )
endif()
