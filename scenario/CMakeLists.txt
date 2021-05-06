# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_subdirectory(core)

set(SCENARIO_COMPONENTS ScenarioCore)
set(SCENARIO_PRIVATE_DEPENDENCIES "")

if(SCENARIO_USE_IGNITION)

    option(ENABLE_PROFILER "Enable Ignition Profiler" OFF)
    mark_as_advanced(ENABLE_PROFILER)

    add_subdirectory(gazebo)
    add_subdirectory(plugins)
    add_subdirectory(controllers)

    list(APPEND SCENARIO_COMPONENTS ScenarioGazebo ScenarioControllers)

endif()

# Dummy meta target for the top-level EXPORT
add_library(Scenario INTERFACE)

install(
    TARGETS Scenario
    EXPORT ScenarioExport)

install_basic_package_files(Scenario
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT ScenarioExport
    DEPENDENCIES ${SCENARIO_COMPONENTS}
    PRIVATE_DEPENDENCIES ${SCENARIO_PRIVATE_DEPENDENCIES}
    NAMESPACE Scenario::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${SCENARIO_INSTALL_LIBDIR}/cmake/Scenario)
