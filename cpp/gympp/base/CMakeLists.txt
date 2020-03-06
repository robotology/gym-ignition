# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# =====
# GYMPP
# =====

add_library(gympp
    include/gympp/base/Environment.h
    include/gympp/base/Common.h
    include/gympp/base/Log.h
    include/gympp/base/Random.h
    include/gympp/base/Space.h
    include/gympp/base/Robot.h
    Space.cpp
    Random.cpp)

find_package(ignition-common3 QUIET)

if(ignition-common3_FOUND)
    target_compile_definitions(gympp PUBLIC USE_IGNITION_LOGS)
    target_link_libraries(gympp PUBLIC
        ignition-common3::ignition-common3)
endif()

file(GLOB_RECURSE GYMPP_HEADERS_INSTALL include/*.h)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    set_target_properties(gympp PROPERTIES
        PUBLIC_HEADER "${GYMPP_HEADERS_INSTALL}")
endif()

target_include_directories(gympp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ====
# TASK
# ====

add_library(Task INTERFACE)
target_sources(Task INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gympp/base/Task.h)

target_include_directories(Task INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(Task INTERFACE gympp)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    set_target_properties(Task PROPERTIES
        PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/gympp/base/Task.h)
endif()

# ==============
# TASK SINGLETON
# ==============

add_library(TaskSingleton
    include/gympp/base/TaskSingleton.h
    TaskSingleton.cpp)

target_include_directories(TaskSingleton PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(TaskSingleton
    PRIVATE
    gympp
    Task)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    set_target_properties(TaskSingleton PROPERTIES
        PUBLIC_HEADER include/gympp/base/TaskSingleton.h)
endif()

# ===================
# INSTALL THE TARGETS
# ===================

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS gympp Task TaskSingleton
        EXPORT gympp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp/base)
endif()
