# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# ============
# ScenarioCore
# ============

# Group files with the correct path for the target_sources command
file(GLOB_RECURSE CORE_HEADERS_BUILD include/*.h)
file(GLOB_RECURSE CORE_HEADERS_INSTALL
     RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
     *.h)

set(CORE_HEADERS
    $<BUILD_INTERFACE:${CORE_HEADERS_BUILD}>
    $<INSTALL_INTERFACE:${CORE_HEADERS_INSTALL}>)

add_library(ScenarioCore INTERFACE)
target_sources(ScenarioCore INTERFACE ${CORE_HEADERS})

target_include_directories(ScenarioCore INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ===================
# Install the targets
# ===================

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS ScenarioCore
        EXPORT scenario)
endif()
