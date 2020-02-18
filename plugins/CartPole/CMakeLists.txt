# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# ===============
# CartPole PLUGIN
# ===============

add_library(CartPolePlugin SHARED
    CartPolePlugin.h
    CartPolePlugin.cpp)

target_link_libraries(CartPolePlugin
    PUBLIC
    Task
    ignition-gazebo3::core
    PRIVATE
    TaskSingleton
    RobotSingleton)

target_include_directories(CartPolePlugin PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

if(MSVC)
    # Import math symbols from standard cmath
    target_compile_definitions(CartPolePlugin PRIVATE "_USE_MATH_DEFINES")
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS CartPolePlugin
        EXPORT gympp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/plugins
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/plugins
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/gympp/plugins)
endif()
