# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.12)
project(GymIgnition VERSION 0.1)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include useful features
include(GNUInstallDirs)

# Build type
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING
            "Choose the type of build, recommended options are: Debug or Release" FORCE)
    endif()
    set(GYMIGNITION_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${GYMIGNITION_BUILD_TYPES})
endif()

# This new build mode configures the CMake project to be compatible with the pipeline to
# create the PyPI linux wheel
include(cmake/AddNewBuildMode.cmake)
add_new_build_mode(NAME "PyPI" TEMPLATE "Release")

# Expose shared or static compilation
set(GYMIGNITION_BUILD_SHARED_LIBRARY TRUE
    CACHE BOOL "Compile libraries as shared libraries")

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL "PyPI")
    # Apply the user choice
    set(BUILD_SHARED_LIBS ${GYMIGNITION_BUILD_SHARED_LIBRARY})
else()
    # Check that is Linux
    if(NOT (UNIX AND NOT APPLE))
        message(FATAL_ERROR "PyPI packages can be only created for Linux at the moment")
    endif()

    if(${GYMIGNITION_BUILD_SHARED_LIBRARY})
        message(WARNING "Enabling static compilation, required by the PyPI build mode")
    endif()

    # Force static compilation
    set(BUILD_SHARED_LIBS FALSE)
endif()

# Use -fPIC even if statically compiled
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

# Get include-what-you-use information when compiling
option(GYMIGNITION_USE_IWYU "Get the output of include-what-you-use" OFF)
if(GYMIGNITION_USE_IWYU)
    find_program(IWYU_PATH NAMES include-what-you-use iwyu)
    if(IWYU_PATH)
        set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
    endif()
endif()

# Settings for RPATH
if(NOT MSVC)
    option(ENABLE_RPATH "Enable RPATH installation" TRUE)
    mark_as_advanced(ENABLE_RPATH)
endif()

# Add custom functions / macros
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Find YCM
find_package(YCM QUIET)

# Bootstrap YCM if not found
if(NOT ${YCM_FOUND})
    include(BootstrapYCM)
endif()

# Configure RPATH
include(AddInstallRPATHSupport)
if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                              LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                                       "${CMAKE_INSTALL_FULL_LIBDIR}/lib"
                              INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                              DEPENDS ENABLE_RPATH
                              USE_LINK_PATH)
endif()

# Dependencies
add_subdirectory(deps)

# Add the C++ sources subdirectory
add_subdirectory(gympp)

# ========
# IGNITION
# ========

find_package(ignition-gazebo3 COMPONENTS all)
option(GYMIGNITION_USE_IGNITION
       "Build C++ code depending on Ignition Robotics" ${ignition-gazebo3_FOUND})

# Add the targets depending on Ignition Robotics
if(${GYMIGNITION_USE_IGNITION})
    add_subdirectory(ignition)
    add_subdirectory(plugins)
    add_subdirectory(gym_ignition_data)
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
