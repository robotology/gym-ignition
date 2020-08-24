# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(iDynTree REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# ==============
# ControllersABC
# ==============

set(CONTROLLERS_ABC_PUBLIC_HDRS
    include/scenario/controllers/Controller.h
    include/scenario/controllers/References.h)

add_library(ControllersABC INTERFACE)
add_library(ScenarioControllers::ControllersABC ALIAS ControllersABC)

target_include_directories(ControllersABC INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${SCENARIO_INSTALL_INCLUDEDIR}>)

set_target_properties(ControllersABC PROPERTIES
    PUBLIC_HEADER "${CONTROLLERS_ABC_PUBLIC_HDRS}")

# https://stackoverflow.com/a/29214327
# As a workaround target_sources can be used, however it requires more
# boilerplate code and it has probles when exporting the INTERFACE target:
# https://crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/
add_custom_target(ScenarioControllersABC SOURCES ${CONTROLLERS_ABC_PUBLIC_HDRS})

# =======================
# ComputedTorqueFixedBase
# =======================

add_library(ComputedTorqueFixedBase SHARED
    include/scenario/controllers/ComputedTorqueFixedBase.h
    src/ComputedTorqueFixedBase.cpp)
add_library(ScenarioControllers::ComputedTorqueFixedBase ALIAS ComputedTorqueFixedBase)

target_link_libraries(ComputedTorqueFixedBase
    PUBLIC
    ScenarioControllers::ControllersABC
    PRIVATE
    Eigen3::Eigen
    ScenarioCore::ScenarioABC
    iDynTree::idyntree-core
    iDynTree::idyntree-model
    iDynTree::idyntree-modelio-urdf
    iDynTree::idyntree-high-level)

target_include_directories(ComputedTorqueFixedBase PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${SCENARIO_INSTALL_INCLUDEDIR}>)

set_target_properties(ComputedTorqueFixedBase PROPERTIES
    PUBLIC_HEADER include/scenario/controllers/ComputedTorqueFixedBase.h)

# ===================
# Install the targets
# ===================

install(
    TARGETS
    ControllersABC
    ComputedTorqueFixedBase
    EXPORT ScenarioControllersExport
    LIBRARY DESTINATION ${SCENARIO_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${SCENARIO_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${SCENARIO_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION
    ${SCENARIO_INSTALL_INCLUDEDIR}/scenario/controllers)

install_basic_package_files(ScenarioControllers
    COMPONENT Controllers
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT ScenarioControllersExport
    NAMESPACE ScenarioControllers::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${SCENARIO_INSTALL_LIBDIR}/cmake/ScenarioControllers)
