# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_library(gympp
    include/gympp/Environment.h
    include/gympp/Common.h
    include/gympp/Log.h
    include/gympp/Random.h
    include/gympp/Space.h
    include/gympp/Robot.h
    src/Space.cpp
    src/Random.cpp)

find_package(ignition-common3 QUIET)

if(ignition-common3_FOUND)
    target_compile_definitions(gympp PUBLIC USE_IGNITION_LOGS)
    target_link_libraries(gympp PUBLIC
        ignition-common3::ignition-common3)
endif()

file(GLOB_RECURSE GYMPP_HEADERS_INSTALL include/*.h)

set_target_properties(gympp PROPERTIES
    PUBLIC_HEADER "${GYMPP_HEADERS_INSTALL}"
    POSITION_INDEPENDENT_CODE ON)

target_include_directories(gympp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS gympp
        EXPORT gympp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp)
endif()
