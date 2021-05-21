# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# This file is here only to allow build systems to find the
# real CMake project that is stored in the scenario/ folder.
cmake_minimum_required(VERSION 3.16)
project(scenario)
add_subdirectory(scenario)

# The uninstall target resource must be included in the top-level CMakeLists
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/scenario/cmake)
include(AddUninstallTarget)

# =============
# DOCUMENTATION
# =============

if(BUILD_DOCS)
    add_subdirectory(docs)
endif()
