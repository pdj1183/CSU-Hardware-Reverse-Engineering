# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(COMMON_PATH ../hal_nordic-master/nrfx/samples/common)
include(${COMMON_PATH}/common.cmake)

GET_DEVICE_CONFIG_FILES(${BOARD} ../boards)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adc_terminal)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE ../hal_nordic-master/nrfx/samples/common ../hal_nordic-master/nrfx/samples/src/nrfx_saadc/common)
