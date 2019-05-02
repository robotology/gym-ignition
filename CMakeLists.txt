# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.11)
project(Gympp VERSION 0.1)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add custom functions / macros
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Include useful features
include(GNUInstallDirs)

# Build type
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING
            "Choose the type of build, recommended options are: Debug or Release" FORCE)
    endif()
    set(GYMPP_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${GYMPP_BUILD_TYPES})
endif()

# Build all libraries as shared. Plugins and singleton do not play well
# together if libraries are static.
set(BUILD_SHARED_LIBS ON)

# Tweak linker flags in Linux
if(UNIX AND NOT APPLE)
    if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        get_filename_component(LINKER_BIN ${CMAKE_LINKER} NAME)
            if("${LINKER_BIN}" STREQUAL "ld")
                set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--unresolved-symbols=report-all")
            endif()
    endif()
endif()

# Control where binaries and libraries are placed in the build folder.
# This simplifies tests running in Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# Add custom CMake functions / macros
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Get include-what-you-use information when compiling
option(USE_IWYU "Get the output of include-what-you-use" OFF)
if(USE_IWYU)
    find_program(IWYU_PATH NAMES include-what-you-use iwyu)
    if(IWYU_PATH)
        set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
    endif()
endif()

# TODO: install basic package file
# TODO: RPATH

# Add the C++ sources subdirectory
add_subdirectory(gympp)

find_package(ignition-gazebo2 COMPONENTS all)
if(ignition-gazebo2_FOUND)
    add_subdirectory(ignition)
    add_subdirectory(deps)
    add_subdirectory(plugins)
    add_subdirectory(models)
    add_subdirectory(examples/cpp)

    option(ENABLE_BINDINGS "Enable swig bindings" ON)
    if(ENABLE_BINDINGS)
        add_subdirectory(bindings)
    endif()
else()
    message(STATUS "Ignition Gazebo not found. Enabled only the gympp interface and classes.")
endif()

# Add unistall target
include(AddUninstallTarget)
