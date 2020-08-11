# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(iDynTree REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# ==========
# Controller
# ==========

set(CONTROLLER_HDRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/controllers/Controller.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/controllers/References.h)

add_library(Controller INTERFACE)
target_sources(Controller INTERFACE ${CONTROLLER_HDRS})

target_include_directories(Controller INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set_target_properties(Controller PROPERTIES PUBLIC_HEADER "${CONTROLLER_HDRS}")

# =======================
# ComputedTorqueFixedBase
# =======================

add_library(ComputedTorqueFixedBase
    include/scenario/controllers/ComputedTorqueFixedBase.h
    src/ComputedTorqueFixedBase.cpp)

target_link_libraries(ComputedTorqueFixedBase
    PUBLIC
    Controller
    PRIVATE
    ScenarioGazebo
    Eigen3::Eigen
    ignition-gazebo3::core
    iDynTree::idyntree-core
    iDynTree::idyntree-model
    iDynTree::idyntree-modelio-urdf
    iDynTree::idyntree-high-level)

target_include_directories(ComputedTorqueFixedBase PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set_target_properties(ComputedTorqueFixedBase PROPERTIES
    PUBLIC_HEADER include/scenario/controllers/ComputedTorqueFixedBase.h)

# ===================
# Install the targets
# ===================

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS
        Controller
        ComputedTorqueFixedBase
        EXPORT scenario
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/scenario/controllers)
endif()
