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
    GazeboEnvironment
    GymFactory
    GazeboSimulator
    Python3::Python)

set_property(TARGET ${gympp_swig_name} PROPERTY
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE)

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")

    install(
        TARGETS ${gympp_swig_name}
        EXPORT gympp
        LIBRARY DESTINATION ${Python3_SITELIB}
        ARCHIVE DESTINATION ${Python3_SITELIB}
        RUNTIME DESTINATION ${Python3_SITELIB})

    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/../${gympp_swig_name}.py
        DESTINATION ${Python3_SITELIB})

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
