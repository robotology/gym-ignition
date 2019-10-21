# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(PythonLibs REQUIRED)

if(${PYTHONLIBS_VERSION_STRING} VERSION_LESS 3.6)
    message(FATAL_ERROR
        "Python libs must be >= v3.6 (found: ${PYTHONLIBS_VERSION_STRING})")
endif()

if(${CMAKE_VERSION} VERSION_GREATER 3.13)
    cmake_policy(SET CMP0078 NEW)
endif()

if(${CMAKE_VERSION} VERSION_GREATER 3.14)
    cmake_policy(SET CMP0086 NEW)
endif()

find_package(SWIG REQUIRED)
set(UseSWIG_MODULE_VERSION 2)
include(${SWIG_USE_FILE})

set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR})
#set(SWIG_OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

set(swig_name "gympp_bindings")
set_source_files_properties(${swig_name}.i PROPERTIES CPLUSPLUS ON)

swig_add_library(${swig_name}
    TYPE SHARED
    LANGUAGE python
    SOURCES ${swig_name}.i)

target_link_libraries(${swig_name} PUBLIC
    GymFactory
    IgnitionEnvironment
    RobotSingleton
    GazeboWrapper
    IgnitionRobot
    ${PYTHON_LIBRARIES})

set_property(TARGET ${swig_name} PROPERTY
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE)

target_include_directories(${swig_name} PRIVATE
    ${PYTHON_INCLUDE_PATH})

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    set(PYTHON_FULL_VERSION "${PYTHONLIBS_VERSION_STRING}")
    string(REPLACE "." ";" PYTHON_FULL_VERSION ${PYTHON_FULL_VERSION})
    list(GET PYTHON_FULL_VERSION 0 PYTHON_MAJOR)
    list(GET PYTHON_FULL_VERSION 1 PYTHON_MINOR)

    install(
        TARGETS ${swig_name}
        EXPORT gympp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/python${PYTHON_MAJOR}.${PYTHON_MINOR}/site-packages
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/python${PYTHON_MAJOR}.${PYTHON_MINOR}/site-packages
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/python${PYTHON_MAJOR}.${PYTHON_MINOR}/site-packages)

    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/${swig_name}.py
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/python${PYTHON_MAJOR}.${PYTHON_MINOR}/site-packages)
else()
    set(PYTHON_PACKAGE_IMPORT_NAME "gym_ignition" CACHE STRING
        "Name of the installed package matching the desired 'import <package_name>'")

    install(
        TARGETS ${swig_name}
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX})

    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/${swig_name}.py
        DESTINATION ${CMAKE_INSTALL_PREFIX})

    install(
        TARGETS ECMProvider RobotController CartPolePlugin PhysicsSystem
        LIBRARY DESTINATION ${PYTHON_PACKAGE_IMPORT_NAME}/plugins)
endif()
