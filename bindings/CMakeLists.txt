# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(Python_ADDITIONAL_VERSIONS "3.6" CACHE STRING "Python to use for binding")

find_package(SWIG REQUIRED)
include(UseSWIG)

find_package(PythonLibs REQUIRED)

set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR})
#set(SWIG_OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

set_property(SOURCE gympp.i PROPERTY CPLUSPLUS ON)
swig_add_library(gympp LANGUAGE python SOURCES gympp.i)
swig_link_libraries(gympp GymFactory IgnitionEnvironment RobotSingleton GazeboWrapper)

set(target ${SWIG_MODULE_gympp_REAL_NAME})
#add_dependencies(${target} ${CMAKE_CURRENT_SOURCE_DIR}/optional.i)

set_property(TARGET ${target} PROPERTY
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE)

target_include_directories(${target} PRIVATE
    ${PYTHON_INCLUDE_PATH})

install(
    TARGETS ${target}
    EXPORT gympp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/bindings
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/bindings
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/gympp/bindings)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/gympp.py
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/bindings)
