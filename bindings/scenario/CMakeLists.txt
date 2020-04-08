# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(scenario_swig_name "scenario_bindings")
set_source_files_properties(${scenario_swig_name}.i PROPERTIES CPLUSPLUS ON)

swig_add_library(${scenario_swig_name}
    TYPE SHARED
    LANGUAGE python
    SOURCES ${scenario_swig_name}.i)

target_link_libraries(${scenario_swig_name} PUBLIC
    ECMSingleton
    ScenarioGazebo
    GazeboSimulator
    Python3::Python)

set_property(TARGET ${scenario_swig_name} PROPERTY
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE)

#set_property(TARGET ${scenario_swig_name} PROPERTY SWIG_COMPILE_OPTIONS -builtin)

set_property(TARGET ${scenario_swig_name} PROPERTY SWIG_COMPILE_OPTIONS -py3)
set_property(TARGET ${scenario_swig_name} PROPERTY SWIG_COMPILE_OPTIONS -doxygen)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")

    install(
        TARGETS ${scenario_swig_name}
        EXPORT scenario
        LIBRARY DESTINATION ${Python3_SITELIB}
        ARCHIVE DESTINATION ${Python3_SITELIB}
        RUNTIME DESTINATION ${Python3_SITELIB})

    install(
        FILES
        ${CMAKE_CURRENT_BINARY_DIR}/../${scenario_swig_name}.py
        DESTINATION ${Python3_SITELIB})

else()

    install(
        TARGETS ${scenario_swig_name}
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX})

    install(
        FILES
        ${CMAKE_CURRENT_BINARY_DIR}/../${scenario_swig_name}.py
        DESTINATION ${CMAKE_INSTALL_PREFIX})

    # Install the plugins
    install(
        TARGETS ECMProvider PhysicsSystem ControllerRunner JointController
        LIBRARY DESTINATION ${PYTHON_PACKAGE_IMPORT_NAME}/plugins)

    # Install other shared libraries.
    # Required for libs that contain singletons otherwise when the
    # project is statically compiled they do not work properly.
    install(
        TARGETS ECMSingleton
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX})

endif()
