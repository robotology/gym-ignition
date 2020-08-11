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

# ================
# Extra Components
# ================

set(EXTRA_COMPONENTS_HDRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/JointPID.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/SimulatedTime.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/BasePoseTarget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/BaseWorldVelocityTarget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/BaseWorldAccelerationTarget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/MaxJointForce.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/JointControlMode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/WorldVelocityCmd.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/JointPositionTarget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/JointVelocityTarget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/JointAccelerationTarget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/HistoryOfAppliedJointForces.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/ExternalWorldWrenchCmdWithDuration.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/Timestamp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/JointControllerPeriod.h)

add_library(ExtraComponents INTERFACE)
target_sources(ExtraComponents INTERFACE ${EXTRA_COMPONENTS_HDRS})

target_include_directories(ExtraComponents INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(ExtraComponents INTERFACE ignition-gazebo3::core)

set_target_properties(ExtraComponents PROPERTIES
    PUBLIC_HEADER "${EXTRA_COMPONENTS_HDRS}")

# ==============
# ScenarioGazebo
# ==============

set(SCENARIO_PUBLIC_HDRS
    include/scenario/gazebo/World.h
    include/scenario/gazebo/Model.h
    include/scenario/gazebo/Joint.h
    include/scenario/gazebo/Link.h
    include/scenario/gazebo/Log.h
    include/scenario/gazebo/utils.h
    include/scenario/gazebo/exceptions.h)

add_library(ScenarioGazebo
    ${SCENARIO_PUBLIC_HDRS}
    include/scenario/gazebo/helpers.h
    src/World.cpp
    src/Model.cpp
    src/Joint.cpp
    src/Link.cpp
    src/utils.cpp
    src/helpers.cpp)

target_include_directories(ScenarioGazebo PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(ScenarioGazebo
    PRIVATE
    ExtraComponents
    ignition-gazebo3::core)

set_target_properties(ScenarioGazebo PROPERTIES
    PUBLIC_HEADER "${SCENARIO_PUBLIC_HDRS}")

# ===============
# GazeboSimulator
# ===============

add_library(GazeboSimulator
    include/scenario/gazebo/GazeboSimulator.h
    src/GazeboSimulator.cpp)

target_link_libraries(GazeboSimulator
    PRIVATE
    ECMSingleton
    ScenarioGazebo
    ExtraComponents
    ignition-gazebo3::core
    tiny-process-library)

set_target_properties(GazeboSimulator PROPERTIES
    PUBLIC_HEADER include/scenario/gazebo/GazeboSimulator.h)

target_include_directories(GazeboSimulator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ===================
# Install the targets
# ===================

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS
        ExtraComponents
        EXPORT scenario
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/scenario/gazebo/components)
    install(
        TARGETS
        ScenarioGazebo GazeboSimulator
        EXPORT scenario
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/scenario/gazebo)
endif()
