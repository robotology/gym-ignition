# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# =============================
# EnvironmentCallbacksSingleton
# =============================

add_library(EnvironmentCallbacksSingleton SHARED
    include/gympp/gazebo/EnvironmentCallbacksSingleton.h
    src/EnvironmentCallbacksSingleton.cpp)

target_include_directories(EnvironmentCallbacksSingleton PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(EnvironmentCallbacksSingleton PUBLIC
    gympp
    ignition-gazebo1::core)

set_target_properties(EnvironmentCallbacksSingleton PROPERTIES
    PUBLIC_HEADER include/gympp/gazebo/EnvironmentCallbacksSingleton.h)

# ====================
# IGNITION ENVIRONMENT
# ====================

add_library(IgnitionEnvironment
    include/gympp/gazebo/IgnitionEnvironment.h
    src/IgnitionEnvironment.cpp)

target_link_libraries(IgnitionEnvironment
    PUBLIC
    gympp
    PRIVATE
    ignition-gazebo1::core
    tiny-process-library)

set_target_properties(IgnitionEnvironment PROPERTIES
    PUBLIC_HEADER include/gympp/gazebo/IgnitionEnvironment.h)

target_include_directories(IgnitionEnvironment PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ==============================
# IGNITION ENVIRONMENT CALLBACKS
# ==============================

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
    ignition-gazebo1::core)

set_target_properties(RobotSingleton PROPERTIES
    PUBLIC_HEADER include/gympp/gazebo/RobotSingleton.h)

target_include_directories(RobotSingleton PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ===================
# INSTALL THE TARGETS
# ===================

install(
    TARGETS
    IgnitionEnvironment
    RobotSingleton
    EnvironmentCallbacks
    EnvironmentCallbacksSingleton
    EXPORT gympp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp/gazebo)

# ====================
# IgnitionRobot PLUGIN
# ====================

add_library(IgnitionRobot SHARED
    include/gympp/gazebo/IgnitionRobot.h
    src/IgnitionRobot.cpp)

target_link_libraries(IgnitionRobot
    PUBLIC
    gympp
    IgnitionEnvironment
    RobotSingleton
    ignition-gazebo1::core)

target_include_directories(IgnitionRobot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(
    TARGETS IgnitionRobot
    EXPORT gympp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/plugins
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/plugins
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/gympp/plugins)
