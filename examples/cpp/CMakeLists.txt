# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

#--------------------------------------
# Find ignition-fuel_tools
ign_find_package(ignition-fuel_tools4 REQUIRED VERSION 4.1)
set(IGN_FUEL_TOOLS_VER ${ignition-fuel_tools4_VERSION_MAJOR})

add_executable(LaunchCartPole LaunchCartPole.cpp)
target_link_libraries(LaunchCartPole PRIVATE GymFactory Clara
    ignition-fuel_tools${IGN_FUEL_TOOLS_VER}::ignition-fuel_tools${IGN_FUEL_TOOLS_VER})

add_executable(LaunchParallelCartPole LaunchParallelCartPole.cpp)
target_link_libraries(LaunchParallelCartPole PRIVATE GymFactory Clara
    ignition-fuel_tools${IGN_FUEL_TOOLS_VER}::ignition-fuel_tools${IGN_FUEL_TOOLS_VER})
