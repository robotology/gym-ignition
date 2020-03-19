# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(gympp_swig_name "gympp_bindings")
set_source_files_properties(${gympp_swig_name}.i PROPERTIES CPLUSPLUS ON)

swig_add_library(${gympp_swig_name}
    TYPE SHARED
    LANGUAGE python
    SOURCES ${gympp_swig_name}.i)

target_link_libraries(${gympp_swig_name} PUBLIC
    gympp
    Task
    TaskSingleton
    IgnitionRobot
    RobotSingleton
    GazeboEnvironment
    GymFactory
    GazeboSimulator
    Python3::Python)

set_property(TARGET ${gympp_swig_name} PROPERTY
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE)

#set_property(TARGET ${gympp_swig_name} PROPERTY SWIG_COMPILE_OPTIONS -builtin)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")

    set(PYTHON_FOLDER "python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

    install(
        TARGETS ${gympp_swig_name}
        EXPORT gympp
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PYTHON_FOLDER}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PYTHON_FOLDER}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/${PYTHON_FOLDER})

    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/${gympp_swig_name}.py
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PYTHON_FOLDER})

else()

    install(
        TARGETS ${gympp_swig_name}
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX})

    install(
        FILES
        ${CMAKE_CURRENT_BINARY_DIR}/../${gympp_swig_name}.py
        DESTINATION ${CMAKE_INSTALL_PREFIX})

    # Install the plugins
    install(
        TARGETS CartPolePlugin
        LIBRARY DESTINATION ${PYTHON_PACKAGE_IMPORT_NAME}/plugins)

endif()
