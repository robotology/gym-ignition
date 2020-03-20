# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# ==============
# GAZEBO WRAPPER
# ==============

add_library(GazeboWrapper
    include/gympp/gazebo/GazeboWrapper.h
    src/GazeboWrapper.cpp)

target_link_libraries(GazeboWrapper
    PRIVATE
    Task
    TaskSingleton
    RobotSingleton
    ECMSingleton
    IgnitionRobot
    ignition-gazebo3::core
    tiny-process-library)

set_target_properties(GazeboWrapper PROPERTIES
    PUBLIC_HEADER include/gympp/gazebo/GazeboWrapper.h)

target_include_directories(GazeboWrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ===================
# INSTALL THE TARGETS
# ===================

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS
        GazeboWrapper
        EXPORT gympp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp/gazebo)
endif()

