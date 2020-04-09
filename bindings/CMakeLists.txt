# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(${CMAKE_VERSION} VERSION_GREATER 3.13)
    cmake_policy(SET CMP0078 NEW)
endif()

if(${CMAKE_VERSION} VERSION_GREATER 3.14)
    cmake_policy(SET CMP0086 NEW)
endif()

find_package(SWIG 4.0 REQUIRED)
set(UseSWIG_MODULE_VERSION 2)
include(${SWIG_USE_FILE})

set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR})
#set(SWIG_OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# This is needed to generate the install path of the plugins
# during the packaging of the PyPI archives
set(PYTHON_PACKAGE_IMPORT_NAME "gym_ignition" CACHE STRING
    "Name of the installed package matching the desired 'import <package_name>'")

if(GYMIGNITION_ENABLE_SCENARIO)
    add_subdirectory(scenario)

    if(GYMIGNITION_ENABLE_GYMPP)
        add_subdirectory(gympp)
    endif()
endif()


