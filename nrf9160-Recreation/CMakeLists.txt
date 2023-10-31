#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf9160-Recreation)

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/saadc.c)

# NORDIC SDK APP END

zephyr_include_directories(src)

# Make folder containing certificates global so that it can be located by
# the AWS IoT library.
zephyr_include_directories_ifdef(CONFIG_AWS_IOT_PROVISION_CERTIFICATES certs)

target_include_directories(app PRIVATE ../hal_nordic-master/nrfx/samples/common ../hal_nordic-master/nrfx/samples/src/nrfx_saadc/common)

