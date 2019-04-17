# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_executable(LaunchCartPole LaunchCartPole.cpp)
target_link_libraries(LaunchCartPole PRIVATE GymFactory Clara)

add_executable(LaunchParallelCartPole LaunchParallelCartPole.cpp)
target_link_libraries(LaunchParallelCartPole PRIVATE GymFactory Clara)
