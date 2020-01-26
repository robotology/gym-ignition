# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(ControllersPublicHeaders
    include/gympp/controllers/Controller.h
    include/gympp/controllers/PositionController.h
    include/gympp/controllers/PositionControllerReferences.h
    include/gympp/controllers/ComputedTorqueFixedBase.h)

add_library(Controllers
    ${ControllersPublicHeaders}
    src/ComputedTorqueFixedBase.cpp)

set_target_properties(Controllers PROPERTIES PUBLIC_HEADER
     "${ControllersPublicHeaders}")

target_include_directories(Controllers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_include_directories(Controllers PRIVATE ${EIGEN3_INCLUDE_DIR})

target_link_libraries(Controllers
    PUBLIC
    gympp
    PRIVATE
    iDynTree::idyntree-core
    iDynTree::idyntree-model
    iDynTree::idyntree-modelio-urdf
    iDynTree::idyntree-high-level)

find_package(ignition-common3 QUIET)
if(ignition-common3_FOUND)
    target_compile_definitions(Controllers PRIVATE USE_IGNITION_LOGS)
    target_link_libraries(Controllers PRIVATE
        ignition-common3::ignition-common3)
endif()

# ===================
# INSTALL THE TARGETS
# ===================

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS
        Controllers
        EXPORT gympp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp/controllers)
endif()
