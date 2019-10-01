# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# =====================
# ENVIRONMENT CALLBACKS
# =====================

add_library(EnvironmentCallbacks INTERFACE)
target_sources(EnvironmentCallbacks INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gympp/gazebo/EnvironmentCallbacks.h)

target_include_directories(EnvironmentCallbacks INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(EnvironmentCallbacks INTERFACE gympp)

install(
    FILES include/gympp/gazebo/EnvironmentCallbacks.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp/gazebo)

# ===============================
# ENVIRONMENT CALLBACKS SINGLETON
# ===============================

add_library(EnvironmentCallbacksSingleton SHARED
    include/gympp/gazebo/EnvironmentCallbacksSingleton.h
    src/EnvironmentCallbacksSingleton.cpp)

target_include_directories(EnvironmentCallbacksSingleton PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(EnvironmentCallbacksSingleton
    PUBLIC
    gympp
    ignition-gazebo2::core
    PRIVATE
    EnvironmentCallbacks)

set_target_properties(EnvironmentCallbacksSingleton PROPERTIES
    PUBLIC_HEADER include/gympp/gazebo/EnvironmentCallbacksSingleton.h)

# =============
# ECM SINGLETON
# =============

add_library(ECMSingleton SHARED
    include/gympp/gazebo/ECMSingleton.h
    src/ECMSingleton.cpp)

target_include_directories(ECMSingleton PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(ECMSingleton
    PUBLIC
    ignition-gazebo2::core
    PRIVATE
    gympp)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPy")
    set_target_properties(ECMSingleton PROPERTIES
        PUBLIC_HEADER include/gympp/gazebo/ECMSingleton.h)
endif()

# ==============
# GAZEBO WRAPPER
# ==============

add_library(GazeboWrapper
    include/gympp/gazebo/GazeboWrapper.h
    src/GazeboWrapper.cpp)

target_link_libraries(GazeboWrapper
    PRIVATE
    EnvironmentCallbacks
    EnvironmentCallbacksSingleton
    ignition-gazebo2::core
    tiny-process-library)

set_target_properties(GazeboWrapper PROPERTIES
    PUBLIC_HEADER include/gympp/gazebo/GazeboWrapper.h)

target_include_directories(GazeboWrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ====================
# IGNITION ENVIRONMENT
# ====================

add_library(IgnitionEnvironment
    include/gympp/gazebo/IgnitionEnvironment.h
    src/IgnitionEnvironment.cpp)

target_link_libraries(IgnitionEnvironment
    PUBLIC
    gympp
    GazeboWrapper
    PRIVATE
    EnvironmentCallbacks
    EnvironmentCallbacksSingleton)

set_target_properties(IgnitionEnvironment PROPERTIES
    PUBLIC_HEADER include/gympp/gazebo/IgnitionEnvironment.h)

target_include_directories(IgnitionEnvironment PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ========================
# IGNITION ROBOT SINGLETON
# ========================

# This class must be shared, otherwise plugins do not share static objects like singletons
# with the memory of the program that loaded them

add_library(RobotSingleton SHARED
    include/gympp/gazebo/RobotSingleton.h
    src/RobotSingleton.cpp)

target_link_libraries(RobotSingleton
    PUBLIC
    gympp
    ignition-gazebo2::core)

set_target_properties(RobotSingleton PROPERTIES
    PUBLIC_HEADER include/gympp/gazebo/RobotSingleton.h)

target_include_directories(RobotSingleton PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ==============
# IGNITION ROBOT
# ==============

add_library(IgnitionRobot SHARED
    include/gympp/gazebo/IgnitionRobot.h
    src/IgnitionRobot.cpp)

target_link_libraries(IgnitionRobot
    PUBLIC
    gympp
    PRIVATE
    RobotSingleton)

set_target_properties(IgnitionRobot PROPERTIES
    PUBLIC_HEADER include/gympp/gazebo/IgnitionRobot.h)

target_include_directories(IgnitionRobot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ===================
# INSTALL THE TARGETS
# ===================

install(
    TARGETS
    IgnitionEnvironment
    GazeboWrapper
    RobotSingleton
    ECMSingleton
    EnvironmentCallbacks
    EnvironmentCallbacksSingleton
    IgnitionRobot
    EXPORT gympp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp/gazebo)
