# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# ==============
# Physics PLUGIN
# ==============

find_package(ignition-physics2 COMPONENTS dartsim REQUIRED)

add_library(PhysicsSystem SHARED
    Physics.h
    Physics.cpp)

target_link_libraries(PhysicsSystem
    PUBLIC
    ignition-gazebo3::core
    ignition-physics2
    PRIVATE
    ExtraComponents)

target_include_directories(PhysicsSystem PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_compile_definitions(PhysicsSystem PRIVATE
    ""dartsim_plugin_LIB=\"$<TARGET_SONAME_FILE:ignition-physics2::ignition-physics2-dartsim-plugin>\""")

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS PhysicsSystem
        EXPORT gympp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/plugins
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp/plugins
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/gympp/plugins)
endif()
