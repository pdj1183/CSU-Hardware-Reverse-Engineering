# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(COMMON_PATH /opt/nordic/ncs/v2.4.0/modules/hal/nordic/nrfx/samples/common)
include(${COMMON_PATH}/common.cmake)

GET_DEVICE_CONFIG_FILES(${BOARD} ../boards)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adc_terminal)

target_sources(app PRIVATE src/main.c  /opt/nordic/ncs/v2.4.0/modules/hal/nordic/nrfx/samples/src/nrfx_saadc/common/saadc_examples_common.c)
target_include_directories(app PRIVATE /opt/nordic/ncs/v2.4.0/modules/hal/nordic/nrfx/samples/common /opt/nordic/ncs/v2.4.0/modules/hal/nordic/nrfx/samples/src/nrfx_saadc/common)
