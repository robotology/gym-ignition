# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# ===========
# ScenarioABC
# ===========

set(SCENARIO_ABC_PUBLIC_HEADERS
    include/scenario/core/World.h
    include/scenario/core/Model.h
    include/scenario/core/Joint.h
    include/scenario/core/Link.h)

add_library(ScenarioABC INTERFACE)
add_library(ScenarioCore::ScenarioABC ALIAS ScenarioABC)

target_include_directories(ScenarioABC INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${SCENARIO_INSTALL_INCLUDEDIR}>)

set_target_properties(ScenarioABC PROPERTIES
    PUBLIC_HEADER "${SCENARIO_ABC_PUBLIC_HEADERS}")

# https://stackoverflow.com/a/29214327
# As a workaround target_sources can be used, however it requires more
# boilerplate code and it has probles when exporting the INTERFACE target:
# https://crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/
add_custom_target(ScenarioCoreABC SOURCES ${SCENARIO_ABC_PUBLIC_HEADERS})

# =================
# ScenarioCoreUtils
# =================

set(CORE_UTILS_HEADERS
    include/scenario/core/utils/Log.h
    include/scenario/core/utils/signals.h
    include/scenario/core/utils/utils.h)

add_library(CoreUtils
    ${CORE_UTILS_HEADERS}
    src/signals.cpp
    src/utils.cpp)
add_library(ScenarioCore::CoreUtils ALIAS CoreUtils)

target_include_directories(CoreUtils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${SCENARIO_INSTALL_INCLUDEDIR}>)

set_target_properties(CoreUtils PROPERTIES
    PUBLIC_HEADER "${CORE_UTILS_HEADERS}")

# This definition is used by the "scenario" Python package
# to detect User / Developer installation mode
if(NOT SCENARIO_CALL_FROM_SETUP_PY)
    target_compile_options(CoreUtils PRIVATE
        -DSCENARIO_CMAKE_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")
endif()

# ===================
# Install the targets
# ===================

install(
    TARGETS ScenarioABC
    EXPORT ScenarioCoreExport
    LIBRARY DESTINATION ${SCENARIO_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${SCENARIO_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${SCENARIO_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION
    ${SCENARIO_INSTALL_INCLUDEDIR}/scenario/core)

install(
    TARGETS CoreUtils
    EXPORT ScenarioCoreExport
    LIBRARY DESTINATION ${SCENARIO_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${SCENARIO_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${SCENARIO_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION
    ${SCENARIO_INSTALL_INCLUDEDIR}/scenario/core/utils)

install_basic_package_files(ScenarioCore
    COMPONENT Core
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT ScenarioCoreExport
    NAMESPACE ScenarioCore::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${SCENARIO_INSTALL_LIBDIR}/cmake/ScenarioCore)
