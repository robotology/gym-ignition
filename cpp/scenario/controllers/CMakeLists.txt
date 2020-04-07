# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
#  This project is dual licensed under LGPL v2.1+ or Apache License.
#
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#
#  This software may be modified and distributed under the terms of the
#  GNU Lesser General Public License v2.1 or any later version.
#
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
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

find_package(iDynTree REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# ==========
# Controller
# ==========

set(CONTROLLER_HDRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/controllers/Controller.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenario/controllers/References.h)

add_library(Controller INTERFACE)
target_sources(Controller INTERFACE ${CONTROLLER_HDRS})

target_include_directories(Controller INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set_target_properties(Controller PROPERTIES PUBLIC_HEADER "${CONTROLLER_HDRS}")



if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS
        Controller
        EXPORT scenario
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/scenario/controllers)
endif()
