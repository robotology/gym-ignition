# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# ======================
# RobotController PLUGIN
# ======================

add_library(RobotController SHARED
    RobotController.h
    RobotController.cpp)

target_link_libraries(RobotController
    PUBLIC
    ignition-gazebo3::core
    PRIVATE
    gympp
    RobotSingleton)

target_include_directories(RobotController PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS RobotController
        EXPORT gympp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/plugins
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/plugins
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/gympp/plugins)
endif()
