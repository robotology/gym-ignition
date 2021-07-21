# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# The SWIG bindings are installed as a standalone <scenario> Python package.
#
# We create with CMake the following structure in the build tree:
#
# <build>/bindings/scenario/
# ├── bindings
# │   ├── core.py
# │   ├── _core.so
# │   ├── gazebo.py
# │   ├── _gazebo.so
# │   └── __init__.py
# └── __init__.py
#
# That is later installed either in the active virtualenv or packaged
# as a PyPI package. The former is related to the Develop Installation
# and the latter to the User Installation.
#
# Having a working Python package tree in the build folder has multiple
# benefit, including the possibility to use it without installing.
# Beyond this reason, our documentation pipeline requires to import
# a working package from the build tree.

if(${CMAKE_VERSION} VERSION_GREATER 3.13)
    cmake_policy(SET CMP0078 NEW)
endif()

if(${CMAKE_VERSION} VERSION_GREATER 3.14)
    cmake_policy(SET CMP0086 NEW)
endif()

find_package(SWIG 4.0 REQUIRED)
set(UseSWIG_MODULE_VERSION 2)
include(${SWIG_USE_FILE})

# By default, install ScenarIO in the python site-package directory
if(NOT BINDINGS_INSTALL_PREFIX)
    set(BINDINGS_INSTALL_PREFIX ${Python3_SITELIB})
endif()

# Expose the install prefix as CMake option
set(BINDINGS_INSTALL_PREFIX "${BINDINGS_INSTALL_PREFIX}"
    CACHE STRING "Installation prefix of the bindings")

# Final directory of the "scenario" package
if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    set(SCENARIO_PACKAGE_INSTALL_DIR "${BINDINGS_INSTALL_PREFIX}/scenario")
else()
    # If packaging for PyPI, install ScenarIO in the temp site-package directory
    # created by either setup.py or pip.
    # The "scenario/" folder is added by cmake_build_extension
    set(SCENARIO_PACKAGE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}")
endif()

# Add the SWIG folders
add_subdirectory(core)
if(SCENARIO_USE_IGNITION)
    add_subdirectory(gazebo)
endif()

# Move main init.py file to package root of the build tree
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
    ${CMAKE_CURRENT_BINARY_DIR}/scenario/__init__.py)

# Make scenario.bindings a package
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/scenario/bindings)
file(TOUCH ${CMAKE_CURRENT_BINARY_DIR}/scenario/bindings/__init__.py)

# Move main init.py file to package root of the install tree
install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
    DESTINATION ${SCENARIO_PACKAGE_INSTALL_DIR})
