# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
#  This project is dual licensed under LGPL v2.1+ or Apache License.
#
# -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
#
#  This software may be modified and distributed under the terms of the
#  GNU Lesser General Public License v2.1 or any later version.
#
# -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# ===========
# ECMProvider
# ===========

add_library(ECMProvider SHARED
    include/scenario/plugins/gazebo/ECMProvider.h
    ECMProvider.cpp)

target_link_libraries(ECMProvider
    PUBLIC
    ignition-gazebo3::core
    PRIVATE
    ECMSingleton
    ExtraComponents)

target_include_directories(ECMProvider PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# ============
# ECMSingleton
# ============

add_library(ECMSingleton SHARED
    include/scenario/plugins/gazebo/ECMSingleton.h
    ECMSingleton.cpp)

target_include_directories(ECMSingleton PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(ECMSingleton
    PUBLIC
    ignition-gazebo3::core
    PRIVATE
    ScenarioGazebo)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    set_target_properties(ECMSingleton PROPERTIES
        PUBLIC_HEADER include/scenario/plugins/gazebo/ECMSingleton.h)
endif()

# ===================
# Install the targets
# ===================

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS ECMProvider
        EXPORT scenario
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/scenario/plugins
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/scenario/plugins
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/scenario/plugins)
    install(
        TARGETS ECMSingleton
        EXPORT scenario
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/scenario/plugins/gazebo)
endif()
