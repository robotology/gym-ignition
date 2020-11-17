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

# ==================
# ControllersFactory
# ==================

add_library(ControllersFactory STATIC
    ControllersFactory.h
    ControllersFactory.cpp)

target_link_libraries(ControllersFactory
    PUBLIC
    ${sdformat.sdformat}
    ScenarioCore::ScenarioABC
    ScenarioControllers::ControllersABC
    PRIVATE
    ScenarioGazebo::ScenarioGazebo
    ScenarioControllers::ComputedTorqueFixedBase)

target_include_directories(ControllersFactory PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

# ================
# ControllerRunner
# ================

add_library(ControllerRunner SHARED
    ControllerRunner.h
    ControllerRunner.cpp)

target_link_libraries(ControllerRunner
    PUBLIC
    ${ignition-gazebo.core}
    PRIVATE
    ScenarioGazebo::ScenarioGazebo
    ScenarioGazebo::ExtraComponents
    ScenarioControllers::ControllersABC
    ControllersFactory)

target_include_directories(ControllerRunner PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

# ===================
# Install the targets
# ===================

install(
    TARGETS ControllerRunner
    LIBRARY DESTINATION ${SCENARIO_INSTALL_LIBDIR}/scenario/plugins
    ARCHIVE DESTINATION ${SCENARIO_INSTALL_LIBDIR}/scenario/plugins
    RUNTIME DESTINATION ${SCENARIO_INSTALL_BINDIR})
