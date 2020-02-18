# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# ==========
# COMPONENTS
# ==========

add_library(ExtraComponents INTERFACE)
target_sources(ExtraComponents INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gympp/gazebo/components/JointPositionReset.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gympp/gazebo/components/JointVelocityReset.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gympp/gazebo/components/WorldVelocityCmd.h)

target_include_directories(ExtraComponents INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(ExtraComponents INTERFACE ignition-gazebo3::core)

# =====================
# ENVIRONMENT CALLBACKS
# =====================

add_library(Task INTERFACE)
target_sources(Task INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gympp/gazebo/Task.h)

target_include_directories(Task INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(Task INTERFACE gympp)

# ==============
# TASK SINGLETON
# ==============

add_library(TaskSingleton
    include/gympp/gazebo/TaskSingleton.h
    src/TaskSingleton.cpp)

target_include_directories(TaskSingleton PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(TaskSingleton
    PUBLIC
    gympp
    PRIVATE
    Task)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    set_target_properties(TaskSingleton PROPERTIES
        PUBLIC_HEADER include/gympp/gazebo/TaskSingleton.h)
endif()

# =============
# ECM SINGLETON
# =============

add_library(ECMSingleton
    include/gympp/gazebo/ECMSingleton.h
    src/ECMSingleton.cpp)

target_include_directories(ECMSingleton PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(ECMSingleton
    PUBLIC
    ignition-gazebo3::core
    PRIVATE
    gympp)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
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
    Task
    TaskSingleton
    ignition-gazebo3::core)

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

add_library(RobotSingleton
    include/gympp/gazebo/RobotSingleton.h
    src/RobotSingleton.cpp)

target_link_libraries(RobotSingleton
    PUBLIC
    gympp
    ignition-gazebo3::core)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    set_target_properties(RobotSingleton PROPERTIES
        PUBLIC_HEADER include/gympp/gazebo/RobotSingleton.h)
endif()

target_include_directories(RobotSingleton PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ==============
# IGNITION ROBOT
# ==============

add_library(IgnitionRobot
    include/gympp/gazebo/IgnitionRobot.h
    src/IgnitionRobot.cpp)

target_link_libraries(IgnitionRobot
    PUBLIC
    gympp
    PRIVATE
    RobotSingleton
    ExtraComponents)

set_target_properties(IgnitionRobot PROPERTIES
    PUBLIC_HEADER include/gympp/gazebo/IgnitionRobot.h)

target_include_directories(IgnitionRobot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ===================
# INSTALL THE TARGETS
# ===================

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        FILES include/gympp/gazebo/Task.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp/gazebo)

    install(
        TARGETS
        IgnitionEnvironment
        GazeboWrapper
        RobotSingleton
        ECMSingleton
        Task
        TaskSingleton
        IgnitionRobot
        EXPORT gympp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp/gazebo)
endif()

