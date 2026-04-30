# Copyright (c) 2021-2025 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

set(RV_ARCH "rv32gc")
set(RV_ABI "ilp32d")
set(CONFIG_HPM_SPI 1)
set(CONFIG_FREERTOS 1)
set(CONFIG_FATFS 1)
set(CONFIG_SDMMC 1)
set(CONFIG_CHERRYUSB 1)
set(CONFIG_USB_DEVICE 1)
set(CONFIG_USB_DEVICE_CDC_ACM 1)
set(CUSTOM_GCC_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../linker/${HPM_SOC_SERIES}/gcc/${CUSTOM_GCC_LINKER_FILE}.ld)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})
project(freertos_hello)

sdk_inc(src)
sdk_inc(../../../modules/tinyalloc)
sdk_src(../../../modules/tinyalloc/tinyalloc.c)
## LIBNAME may be a string containing one or more items separated by
## spaces. Convert it to a CMake list and iterate each element.
if(NOT ${LIBNAME} STREQUAL "")
    message(LIBNAME: ${LIBNAME})
	# replace spaces with semicolons to form a CMake list
	string(REPLACE " " ";" LIBNAME_LIST "${LIBNAME}")
    message(LIBNAME_LIST: ${LIBNAME_LIST})
    # Wrap the libraries with --start-group and --end-group to resolve circular dependencies
    sdk_ld_options("-Wl,--start-group")
	foreach(_lib ${LIBNAME_LIST})
		message("Processing library: ${_lib}")
		# If the entry already looks like a path or a .a file, link it as-is
		if(_lib MATCHES "\\.a$" OR _lib MATCHES ".*/.*" OR _lib MATCHES "^/.*")
			message("Linking library as-is: ${_lib}")
			target_link_libraries(${HPM_SDK_LIB_ITF} INTERFACE ${_lib})
		else()
			# Otherwise assume it's a library name and build the examples path
			message("Linking library with examples path: ${CMAKE_BINARY_DIR}/../../lib/examples/${_lib}.a")
			target_link_libraries(${HPM_SDK_LIB_ITF} INTERFACE ${CMAKE_BINARY_DIR}/../../lib/examples/${_lib}.a)
		endif()
	endforeach()
    sdk_ld_options("-Wl,--end-group")
endif()
# Disable array-bounds warnings for FreeRTOS code
sdk_compile_options("-Wno-array-bounds")
sdk_compile_definitions(-DUSE_NONVECTOR_MODE=1)
sdk_compile_definitions(-DDISABLE_IRQ_PREEMPTIVE=1)
sdk_compile_definitions(-DNO_CLEANUP_AT_START=1)
sdk_compile_definitions(-DHPM_I2C_DRV_DEFAULT_RETRY_COUNT=50000)
sdk_compile_definitions(-DHPM_SPI_DRV_DEFAULT_RETRY_COUNT=50000)


sdk_app_src(src/ardupilot_startup.c)
sdk_ld_options("-lm")
sdk_ld_options("-lstdc++")
sdk_ld_options("-Wl,--wrap,_malloc_r")
sdk_ld_options("-Wl,--wrap,realloc")
sdk_ld_options("-Wl,--wrap,printf")
sdk_ld_options("-Wl,--wrap,_realloc_r")
sdk_ld_options("-Wl,--wrap,free")
sdk_ld_options("-Wl,--wrap,calloc")
sdk_ld_options("-Wl,--wrap,malloc")
