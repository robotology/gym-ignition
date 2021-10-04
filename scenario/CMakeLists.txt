# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.16)
project(scenario VERSION 1.3.0)

# Add custom functions / macros
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include useful features
include(GNUInstallDirs)

# Build type
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING
            "Choose the type of build, recommended options are: Debug or Release" FORCE)
    endif()
    set(SCENARIO_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${SCENARIO_BUILD_TYPES})
endif()

# This new build mode configures the CMake project to be compatible with the pipeline to
# create the PyPI linux wheel
include(AddNewBuildMode)
add_new_build_mode(NAME "PyPI" TEMPLATE "Release")

# Expose shared or static compilation
set(SCENARIO_BUILD_SHARED_LIBRARY TRUE
    CACHE BOOL "Compile libraries as shared libraries")

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL "PyPI")
    # Apply the user choice
    set(BUILD_SHARED_LIBS ${SCENARIO_BUILD_SHARED_LIBRARY})
else()
    # Check that is Linux
    if(NOT (UNIX AND NOT APPLE))
        message(FATAL_ERROR "PyPI packages can be only created for Linux at the moment")
    endif()

    if(SCENARIO_BUILD_SHARED_LIBRARY)
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
                set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--unresolved-symbols=report-all")
            endif()
    endif()
endif()

# Control where binaries and libraries are placed in the build folder.
# This simplifies tests running in Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# Get include-what-you-use information when compiling
option(SCENARIO_USE_IWYU "Get the output of include-what-you-use" OFF)
mark_as_advanced(SCENARIO_USE_IWYU)
if(SCENARIO_USE_IWYU)
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

# Dependencies
add_subdirectory(deps)

if(${CMAKE_VERSION} VERSION_GREATER 3.15)
    cmake_policy(SET CMP0094 NEW)
endif()

# Find virtualenv's before system's interpreters
set(Python3_FIND_VIRTUALENV "FIRST" CACHE STRING
    "Configure the detection of virtual environments")
set(Python3_FIND_VIRTUALENV_TYPES "FIRST" "ONLY" "STANDARD")
mark_as_advanced(Python3_FIND_VIRTUALENV)
set_property(CACHE Python3_FIND_VIRTUALENV PROPERTY STRINGS ${Python3_FIND_VIRTUALENV_TYPES})

# Find Python3
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "Using Python: ${Python3_EXECUTABLE}")

# Select the appropriate install prefix used throughout the project.
set(SCENARIO_INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR})
set(SCENARIO_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR})
set(SCENARIO_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR})

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    # Add the libraries installed in the Python site-package folder
    set(EXTRA_RPATH_DIRS
        "${Python3_SITELIB}"
        "${Python3_SITELIB}/scenario/bindings")
else()
    # Add the libraries installed in the Python site-package folder
    # (that in this case is CMAKE_INSTALL_PREFIX)
    set(EXTRA_RPATH_DIRS
        "${CMAKE_INSTALL_PREFIX}"
        "${CMAKE_INSTALL_PREFIX}/scenario/bindings")
endif()

# Configure RPATH
include(AddInstallRPATHSupport)
add_install_rpath_support(
    BIN_DIRS
    "${CMAKE_INSTALL_PREFIX}/${SCENARIO_INSTALL_BINDIR}"
    LIB_DIRS
    "${CMAKE_INSTALL_PREFIX}/${SCENARIO_INSTALL_LIBDIR}"
    "${CMAKE_INSTALL_PREFIX}/${SCENARIO_INSTALL_LIBDIR}/scenario/plugins"
    "${EXTRA_RPATH_DIRS}"
    INSTALL_NAME_DIR
    "${CMAKE_INSTALL_PREFIX}/${SCENARIO_INSTALL_LIBDIR}"
    DEPENDS ENABLE_RPATH
    USE_LINK_PATH)

# Find a supported Ignition distribution
if(NOT IGNITION_DISTRIBUTION)

    include(FindIgnitionDistribution)
    set(SUPPORTED_IGNITION_DISTRIBUTIONS
        "Fortress"
        "Edifice"
        "Dome"
        "Citadel")

    foreach(distribution IN LISTS SUPPORTED_IGNITION_DISTRIBUTIONS)

        find_ignition_distribution(
            CODENAME ${distribution}
            PACKAGES
            ignition-gazebo
            REQUIRED FALSE)

        if(${${distribution}_FOUND})
            message(STATUS "Found Ignition ${distribution}")

            # Select Ignition distribution
            set(IGNITION_DISTRIBUTION "${distribution}" CACHE
                STRING "The Ignition distribution found in the system")
            set_property(CACHE IGNITION_DISTRIBUTION PROPERTY
                STRINGS ${SUPPORTED_IGNITION_DISTRIBUTIONS})

            break()
        endif()

    endforeach()

endif()

if(NOT IGNITION_DISTRIBUTION OR "${IGNITION_DISTRIBUTION}" STREQUAL "")
    set(USE_IGNITION FALSE)
else()
    set(USE_IGNITION TRUE)
endif()

option(SCENARIO_USE_IGNITION
       "Build C++ code depending on Ignition"
       ${USE_IGNITION})

# Fail if Ignition is enabled but no compatible distribution was found
if(SCENARIO_USE_IGNITION AND "${IGNITION_DISTRIBUTION}" STREQUAL "")
    message(FATAL_ERROR "Failed to find a compatible Ignition Gazebo distribution")
endif()

# Alias the targets
if(SCENARIO_USE_IGNITION)
    include(ImportTargets${IGNITION_DISTRIBUTION})
endif()

# Helper for exporting targets
include(InstallBasicPackageFiles)

# =========
# SCENARI/O
# =========

add_subdirectory(src)

# ========
# BINDINGS
# ========

# Require to find Ignition libraries when packaging for PyPI
if(CMAKE_BUILD_TYPE STREQUAL "PyPI" AND NOT USE_IGNITION)
    message(FATAL_ERROR "Found no Ignition distribution for PyPI package")
endif()

find_package(SWIG 4.0 QUIET)
option(SCENARIO_ENABLE_BINDINGS "Enable SWIG bindings" ${SWIG_FOUND})

if(SCENARIO_ENABLE_BINDINGS)
    add_subdirectory(bindings)
endif()

# Add unistall target
include(AddUninstallTarget)
