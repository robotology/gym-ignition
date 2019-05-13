# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include(FetchContent)

message(STATUS "Bootstrapping YCM")

FetchContent_Declare(YCM GIT_REPOSITORY https://github.com/robotology/ycm.git)
FetchContent_GetProperties(YCM)

if(NOT YCM_POPULATED)
    FetchContent_Populate(YCM)
    list(APPEND CMAKE_MODULE_PATH ${ycm_SOURCE_DIR}/modules)
endif()

