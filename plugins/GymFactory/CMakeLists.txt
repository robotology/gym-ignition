# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(GYMFACTORY_PUBLIC_HEADERS
    include/gympp/GymFactory.h
    include/gympp/Metadata.h
    include/gympp/PluginDatabase.h)

add_library(GymFactory
    ${GYMFACTORY_PUBLIC_HEADERS}
    src/GymFactory.cpp)

target_link_libraries(GymFactory
    PUBLIC gympp IgnitionEnvironment
    PRIVATE ignition-gazebo3::core)

set_target_properties(GymFactory PROPERTIES
    PUBLIC_HEADER
    "${GYMFACTORY_PUBLIC_HEADERS}")

target_include_directories(GymFactory PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_definitions(GymFactory PRIVATE
    CARTPOLE_PLUGIN_PATH="$<TARGET_FILE_DIR:CartPolePlugin>")

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS GymFactory
        EXPORT gympp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp)
endif()
