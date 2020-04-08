# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# ===============
# PLUGIN DATABASE
# ===============

add_library(PluginDatabase INTERFACE)
target_sources(PluginDatabase INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gympp/plugins/PluginDatabase.h)

target_include_directories(PluginDatabase INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(PluginDatabase INTERFACE gympp)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    set_target_properties(PluginDatabase PROPERTIES
        PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/gympp/plugins/PluginDatabase.h)
endif()

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
    ECMSingleton
    TaskSingleton
    ScenarioGazebo)

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
