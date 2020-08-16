# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(scenario_swig_name "gazebo")
set_source_files_properties(${scenario_swig_name}.i PROPERTIES CPLUSPLUS ON)

# The bindings shared library is stored in the Python package of the build tree
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../scenario/bindings)

swig_add_library(${scenario_swig_name}
    TYPE SHARED
    LANGUAGE python
    OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/../scenario/bindings
    OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/..
    SOURCES ${scenario_swig_name}.i)

target_link_libraries(${scenario_swig_name} PUBLIC
    ECMSingleton
    ScenarioCore
    ScenarioGazebo
    GazeboSimulator
    Python3::Python)
add_library(Scenario::SwigScenarioGazebo ALIAS gazebo)

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
    EXPORT bindings
    LIBRARY DESTINATION ${BINDINGS_INSTALL_PREFIX}/scenario/bindings
    ARCHIVE DESTINATION ${BINDINGS_INSTALL_PREFIX}/scenario/bindings
    RUNTIME DESTINATION ${BINDINGS_INSTALL_PREFIX}/scenario/bindings)

install(
    FILES ${WRAPPER_PY_FILE}
    DESTINATION ${BINDINGS_INSTALL_PREFIX}/scenario/bindings)

# When packaging for PyPI, also install in the Python site-package the
# Ignition Gazebo plugins and their dependending libraries
if(CMAKE_BUILD_TYPE STREQUAL "PyPI")
    # Install the plugins
    install(
        TARGETS ECMProvider PhysicsSystem ControllerRunner JointController
        LIBRARY DESTINATION ${BINDINGS_INSTALL_PREFIX}/scenario/plugins)

    # Install other shared libraries.
    # Required for libs that contain singletons otherwise when the
    # project is statically compiled they do not work properly.
    install(
        TARGETS ECMSingleton
        LIBRARY DESTINATION ${BINDINGS_INSTALL_PREFIX}/scenario/lib)
endif()
