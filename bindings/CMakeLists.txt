# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(Python3 COMPONENTS Development REQUIRED)

if(${Python3_VERSION} VERSION_LESS 3.6)
    message(FATAL_ERROR
        "Python libs must be >= v3.6 (found: ${Python3_VERSION})")
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
    gympp
    IgnitionRobot
    RobotSingleton
    GymFactory
    GazeboEnvironment
    ECMSingleton
    RobotSingleton
    GazeboSimulator
    ScenarioGazebo
    Python3::Python)

set_property(TARGET ${swig_name} PROPERTY
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")

    install(
        TARGETS ${swig_name}
        EXPORT gympp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages)

    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/${swig_name}.py
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages)
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
