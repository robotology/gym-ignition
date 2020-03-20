# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
#  This project is dual licensed under LGPL v2.1+ or Apache License.
#
# -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
#
#  This software may be modified and distributed under the terms of the
#  GNU Lesser General Public License v2.1 or any later version.
#
# -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

find_package(ignition-physics2 COMPONENTS dartsim REQUIRED)

add_library(PhysicsSystem SHARED
    Physics.h
    Physics.cpp)

target_link_libraries(PhysicsSystem
    PUBLIC
    ignition-gazebo3::core
    PRIVATE
    ignition-physics2
    ExtraComponents)

target_include_directories(PhysicsSystem PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_compile_definitions(PhysicsSystem PRIVATE
    ""dartsim_plugin_LIB=\"$<TARGET_SONAME_FILE:ignition-physics2::ignition-physics2-dartsim-plugin>\""")

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS PhysicsSystem
        EXPORT scenario
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/scenario/plugins
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/scenario/plugins
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/scenario/plugins)
endif()
