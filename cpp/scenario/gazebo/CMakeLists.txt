# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# ================
# Extra Components
# ================

set(EXTRA_COMPONENTS_HDRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/JointPID.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/Constraint.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/ConstraintsData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/JointEffortLimit.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/JointControlMode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/JointPositionReset.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/JointVelocityReset.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/gazebo/components/WorldVelocityCmd.h)

add_library(ExtraComponents INTERFACE)
target_sources(ExtraComponents INTERFACE ${EXTRA_COMPONENTS_HDRS})

target_include_directories(ExtraComponents INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(ExtraComponents INTERFACE ignition-gazebo3::core)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    set_target_properties(ExtraComponents PROPERTIES
        PUBLIC_HEADER "${EXTRA_COMPONENTS_HDRS}")
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS
        ExtraComponents
        EXPORT scenario
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/scenario/gazebo/components)
endif()
