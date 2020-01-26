# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# ==================
# ECMProvider PLUGIN
# ==================

add_library(ECMProvider SHARED
    ECMProvider.h
    ECMProvider.cpp)

target_link_libraries(ECMProvider
    PUBLIC
    ignition-gazebo3::core
    PRIVATE
    gympp
    ECMSingleton)

target_include_directories(ECMProvider PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
install(
    TARGETS ECMProvider
    EXPORT gympp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/plugins
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/plugins
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/gympp/plugins)
endif()
