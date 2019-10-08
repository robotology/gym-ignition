# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(CMAKE_BUILD_TYPE STREQUAL "PyPI")
    return()
endif()

# ===========
# WORLD FILES
# ===========

install(
    DIRECTORY worlds
    DESTINATION ${CMAKE_INSTALL_DATADIR}/gympp/gazebo)

# ==========
# SDF MODELS
# ==========

install(
    DIRECTORY CartPole
    DESTINATION ${CMAKE_INSTALL_DATADIR}/gympp/gazebo/models
    FILES_MATCHING PATTERN "*.sdf" PATTERN "*.urdf" PATTERN "*.config")

install(
    DIRECTORY Pendulum
    DESTINATION ${CMAKE_INSTALL_DATADIR}/gympp/gazebo/models
    FILES_MATCHING PATTERN "*.sdf" PATTERN "*.urdf" PATTERN "*.config")

install(
    DIRECTORY iCubGazebo
    DESTINATION ${CMAKE_INSTALL_DATADIR}/gympp/gazebo/models
    FILES_MATCHING PATTERN "*.sdf" PATTERN "*.config")

install(
    DIRECTORY iCubGazeboV2_5
    DESTINATION ${CMAKE_INSTALL_DATADIR}/gympp/gazebo/models
    FILES_MATCHING PATTERN "*.sdf" PATTERN "*.config")
