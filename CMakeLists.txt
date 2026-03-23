#
# Copyright 2024 SignalCraft Technologies Inc.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

### Project Setup #############################################################
cmake_minimum_required(VERSION 3.5.1)
project(SC2470 CXX)

### Configure Compiler ########################################################
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_SYSTEM_NAME STREQUAL "FreeBSD" AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_EXE_LINKER_FLAGS "-lthr ${CMAKE_EXE_LINKER_FLAGS}")
    set(CMAKE_CXX_FLAGS "-stdlib=libc++ ${CMAKE_CXX_FLAGS}")
endif()

### Set up build environment ##################################################
option(UHD_USE_STATIC_LIBS OFF)
find_package(UHD 4.4.0 REQUIRED)

set(UHD_BOOST_REQUIRED_COMPONENTS
    program_options
    system
    thread
)
set(BOOST_MIN_VERSION 1.71)
include(UHDBoost)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/lib
    ${Boost_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
)
link_directories(${Boost_LIBRARY_DIRS})

add_subdirectory(lib)

list(APPEND sc2470_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/version.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/sc2470_impl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/sc2470_expert.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/udc/udc_ctrl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/udc/udc_phy.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/utility/range_util.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/utility/float_util.cpp
)
add_subdirectory(include)

message(STATUS "UHD Module Path (to which the extension will be linked):")
message(STATUS ${UHD_MODULE_PATH})

### Make the executable #######################################################
add_library(sc2470 SHARED ${sc2470_sources})

### Install ###################################################################
set(extension_target ${CMAKE_INSTALL_PREFIX}/lib/libsc2470.so)
message(STATUS "extension target:")
message(STATUS ${extension_target})


install(TARGETS sc2470 DESTINATION lib)
install(CODE "execute_process(COMMAND bash -c \"mkdir -p ${UHD_MODULE_PATH} && ln -s -f ${extension_target} -t ${UHD_MODULE_PATH}\")")
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION include FILES_MATCHING PATTERN "*.h*")