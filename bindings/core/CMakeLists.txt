# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(scenario_swig_name "core")
set_source_files_properties(${scenario_swig_name}.i PROPERTIES CPLUSPLUS ON)

swig_add_library(${scenario_swig_name}
    TYPE SHARED
    LANGUAGE python
    SOURCES ${scenario_swig_name}.i)

target_link_libraries(${scenario_swig_name} PUBLIC
    ScenarioCore
    ScenarioCoreUtils
    Python3::Python)

set_property(TARGET ${scenario_swig_name} PROPERTY
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE)

set_property(TARGET ${scenario_swig_name}
    PROPERTY SWIG_COMPILE_OPTIONS -doxygen)

# Add the to_gazebo() helpers
if(GYMIGNITION_USE_IGNITION)
    set_property(TARGET ${scenario_swig_name} PROPERTY
        SWIG_COMPILE_DEFINITIONS SCENARIO_HAS_GAZEBO)
    set_property(TARGET ${scenario_swig_name} PROPERTY
        SWIG_DEPENDS "../gazebo/to_gazebo.i")
    target_link_libraries(${scenario_swig_name} PUBLIC ScenarioGazebo)
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")

    install(
        TARGETS ${scenario_swig_name}
        EXPORT bindings
        LIBRARY DESTINATION ${Python3_SITELIB}/scenario/bindings
        ARCHIVE DESTINATION ${Python3_SITELIB}/scenario/bindings
        RUNTIME DESTINATION ${Python3_SITELIB}/scenario/bindings)

    install(
        FILES
        ${CMAKE_CURRENT_BINARY_DIR}/../${scenario_swig_name}.py
        DESTINATION ${Python3_SITELIB}/scenario/bindings)

else()

    install(
        TARGETS ${scenario_swig_name}
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/scenario/bindings)

    install(
        FILES
        ${CMAKE_CURRENT_BINARY_DIR}/../${scenario_swig_name}.py
        DESTINATION ${CMAKE_INSTALL_PREFIX}/scenario/bindings)

endif()
