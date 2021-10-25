# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(scenario_swig_name "gazebo")
set_source_files_properties(${scenario_swig_name}.i PROPERTIES CPLUSPLUS ON)

# The bindings shared library is stored in the Python package of the build tree
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../scenario/bindings)

swig_add_library(${scenario_swig_name}
    TYPE MODULE
    LANGUAGE python
    OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/../scenario/bindings
    OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/..
    SOURCES ${scenario_swig_name}.i)

target_link_libraries(${scenario_swig_name}
    PUBLIC
    ScenarioGazebo::ScenarioGazebo
    ScenarioGazebo::GazeboSimulator
    Python3::Python)
add_library(ScenarioSwig::Gazebo ALIAS gazebo)

set_property(TARGET ${scenario_swig_name} PROPERTY
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE)

# https://github.com/swig/swig/issues/672#issuecomment-400577864
set_property(TARGET ${scenario_swig_name} PROPERTY
    SWIG_COMPILE_OPTIONS -doxygen -Dfinal)

# Disable SWIG debug code due to the following error:
#   int SWIG_Python_ConvertPtrAndOwn(PyObject *, void **, swig_type_info *, int, int *):
#   Assertion `own' failed.
# https://github.com/swig/swig/issues/731
# https://github.com/swig/swig/issues/773
target_compile_definitions(${scenario_swig_name} PRIVATE NDEBUG)

# Get the gazebo.py wrapper file
get_property(WRAPPER_PY_FILE TARGET ${scenario_swig_name} PROPERTY SWIG_SUPPORT_FILES)

install(
    TARGETS ${scenario_swig_name}
    COMPONENT python
    LIBRARY DESTINATION ${SCENARIO_PACKAGE_INSTALL_DIR}/bindings
    ARCHIVE DESTINATION ${SCENARIO_PACKAGE_INSTALL_DIR}/bindings
    RUNTIME DESTINATION ${SCENARIO_PACKAGE_INSTALL_DIR}/bindings)

install(
    FILES ${WRAPPER_PY_FILE}
    DESTINATION ${SCENARIO_PACKAGE_INSTALL_DIR}/bindings
    COMPONENT python)
