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

set(EXTRA_COMPONENTS_PUBLIC_HEADERS
    include/scenario/gazebo/components/JointPID.h
    include/scenario/gazebo/components/SimulatedTime.h
    include/scenario/gazebo/components/BasePoseTarget.h
    include/scenario/gazebo/components/BaseWorldVelocityTarget.h
    include/scenario/gazebo/components/BaseWorldAccelerationTarget.h
    include/scenario/gazebo/components/JointControlMode.h
    include/scenario/gazebo/components/JointController.h
    include/scenario/gazebo/components/JointPositionTarget.h
    include/scenario/gazebo/components/JointVelocityTarget.h
    include/scenario/gazebo/components/JointAccelerationTarget.h
    include/scenario/gazebo/components/HistoryOfAppliedJointForces.h
    include/scenario/gazebo/components/ExternalWorldWrenchCmdWithDuration.h
    include/scenario/gazebo/components/Timestamp.h
    include/scenario/gazebo/components/JointControllerPeriod.h
    include/scenario/gazebo/components/JointAcceleration.h
    )

add_library(ExtraComponents INTERFACE)
add_library(ScenarioGazebo::ExtraComponents ALIAS ExtraComponents)

target_include_directories(ExtraComponents INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${SCENARIO_INSTALL_INCLUDEDIR}>)

target_link_libraries(ExtraComponents INTERFACE ${ignition-gazebo.core})

set_target_properties(ExtraComponents PROPERTIES
    PUBLIC_HEADER "${EXTRA_COMPONENTS_PUBLIC_HEADERS}")

# https://stackoverflow.com/a/29214327
# As a workaround target_sources can be used, however it requires more
# boilerplate code and it has probles when exporting the INTERFACE target:
# https://crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/
add_custom_target(GazeboExtraComponents SOURCES ${EXTRA_COMPONENTS_PUBLIC_HEADERS})

# ==============
# ScenarioGazebo
# ==============

set(SCENARIO_GAZEBO_PUBLIC_HDRS
    include/scenario/gazebo/GazeboEntity.h
    include/scenario/gazebo/World.h
    include/scenario/gazebo/Model.h
    include/scenario/gazebo/Joint.h
    include/scenario/gazebo/Link.h
    include/scenario/gazebo/Log.h
    include/scenario/gazebo/utils.h
    include/scenario/gazebo/helpers.h
    include/scenario/gazebo/exceptions.h)

add_library(ScenarioGazebo
    ${SCENARIO_GAZEBO_PUBLIC_HDRS}
    src/World.cpp
    src/Model.cpp
    src/Joint.cpp
    src/Link.cpp
    src/utils.cpp
    src/helpers.cpp)
add_library(ScenarioGazebo::ScenarioGazebo ALIAS ScenarioGazebo)

target_include_directories(ScenarioGazebo PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${SCENARIO_INSTALL_INCLUDEDIR}>)

target_link_libraries(ScenarioGazebo
    PUBLIC
    ScenarioCore::ScenarioABC
    ${ignition-gazebo.core}
    ${ignition-common.ignition-common}
    PRIVATE
    ScenarioCore::CoreUtils
    ScenarioGazebo::ExtraComponents
    ${ignition-physics.ignition-physics}
    ${ignition-fuel_tools.ignition-fuel_tools})

set_target_properties(ScenarioGazebo PROPERTIES
    PUBLIC_HEADER "${SCENARIO_GAZEBO_PUBLIC_HDRS}")

# ===============
# GazeboSimulator
# ===============

add_library(GazeboSimulator
    include/scenario/gazebo/GazeboSimulator.h
    src/GazeboSimulator.cpp)
add_library(ScenarioGazebo::GazeboSimulator ALIAS GazeboSimulator)

target_include_directories(GazeboSimulator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${SCENARIO_INSTALL_INCLUDEDIR}>)

target_link_libraries(GazeboSimulator
    PUBLIC
    ScenarioCore::ScenarioABC
    PRIVATE
    tiny-process-library::tiny-process-library
    ${ignition-gazebo.core}
    ScenarioCore::CoreUtils
    ScenarioGazebo::ScenarioGazebo
    ScenarioGazebo::ExtraComponents
    ${ignition-fuel_tools.ignition-fuel_tools})

set_target_properties(GazeboSimulator PROPERTIES
    PUBLIC_HEADER include/scenario/gazebo/GazeboSimulator.h)

# ===================
# Install the targets
# ===================

get_property(TPL_IS_IMPORTED
    TARGET tiny-process-library::tiny-process-library
    PROPERTY IMPORTED)

if(TPL_IS_IMPORTED)
    set(EXPORT_TPL_TARGET)
else()
    set(EXPORT_TPL_TARGET tiny-process-library)
endif()

install(
    TARGETS
    ExtraComponents ${EXPORT_TPL_TARGET}
    EXPORT ScenarioGazeboExport
    LIBRARY DESTINATION ${SCENARIO_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${SCENARIO_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${SCENARIO_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION
    ${SCENARIO_INSTALL_INCLUDEDIR}/scenario/gazebo/components)

install(
    TARGETS
    ScenarioGazebo GazeboSimulator
    EXPORT ScenarioGazeboExport
    LIBRARY DESTINATION ${SCENARIO_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${SCENARIO_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${SCENARIO_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION
    ${SCENARIO_INSTALL_INCLUDEDIR}/scenario/gazebo)

install_basic_package_files(ScenarioGazebo
    COMPONENT Gazebo
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT ScenarioGazeboExport
    DEPENDENCIES ScenarioCore ${ignition-gazebo} ${ignition-common}
    NAMESPACE ScenarioGazebo::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${SCENARIO_INSTALL_LIBDIR}/cmake/ScenarioGazebo)
