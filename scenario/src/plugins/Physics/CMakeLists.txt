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

# =============
# PhysicsSystem
# =============

add_library(PhysicsSystem SHARED
    Physics.hh
    EntityFeatureMap.hh
    CanonicalLinkModelTracker.hh
    Physics.cc)

target_link_libraries(PhysicsSystem
    PUBLIC
    ${ignition-gazebo.core}
    ${ignition-physics.ignition-physics}
    PRIVATE
    ScenarioGazebo::ScenarioGazebo
    ScenarioGazebo::ExtraComponents)

target_include_directories(PhysicsSystem PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

if(ENABLE_PROFILER)
    target_compile_definitions(PhysicsSystem PRIVATE "IGN_PROFILER_ENABLE=1")
endif()

# ===================
# Install the targets
# ===================

install(
    TARGETS PhysicsSystem
    LIBRARY DESTINATION ${SCENARIO_INSTALL_LIBDIR}/scenario/plugins
    ARCHIVE DESTINATION ${SCENARIO_INSTALL_LIBDIR}/scenario/plugins
    RUNTIME DESTINATION ${SCENARIO_INSTALL_BINDIR}/scenario/plugins)
