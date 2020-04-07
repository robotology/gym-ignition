# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
import tempfile
from typing import Tuple
from gym_ignition import scenario_bindings as bindings


def id_gazebo_fn(val: Tuple[float, float, float]):
    if isinstance(val, tuple) and len(val) == 3:
        return f"step={val[0]}_rtf={val[1]}_iters={val[2]}"


@pytest.fixture(scope="function")
def gazebo_fixture(request):

    step_size, rtf, iterations = request.param
    gazebo = bindings.GazeboSimulator(step_size, rtf, iterations)

    yield gazebo

    gazebo.close()


def get_multi_world_sdf_file() -> str:

    multi_world_sdf_string = f"""
    <?xml version="1.0" ?>
    <sdf version="1.6">
        <world name="world1">
        </world>
        <world name="world2">
        </world>
    </sdf>"""

    handle, multi_world_sdf = tempfile.mkstemp()

    with open(handle, 'w') as f:
        f.write(multi_world_sdf_string)

    return multi_world_sdf


def get_cube_urdf_string() -> str:

    mass = 5.0
    edge = 0.2
    i = 1 / 12 * mass * (edge ** 2 + edge ** 2)
    cube_urdf = f"""
    <robot name="cube_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
        <link name="cube">
            <inertial>
              <origin rpy="0 0 0" xyz="0 0 0"/>
              <mass value="{mass}"/>
              <inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/>
            </inertial>
            <visual>
              <geometry>
                <box size="{edge} {edge} {edge}"/>
              </geometry>
              <origin rpy="0 0 0" xyz="0 0 0"/>
            </visual>
            <collision>
              <geometry>
                <box size="{edge} {edge} {edge}"/>
              </geometry>
              <origin rpy="0 0 0" xyz="0 0 0"/>
            </collision>
        </link>
    </robot>"""

    return cube_urdf


def get_cube_urdf() -> str:

    handle, model_file = tempfile.mkstemp()
    with open(handle, 'w') as f:
        f.write(get_cube_urdf_string())

    return model_file


def get_empty_world_sdf() -> str:

    world_sdf_string = bindings.getEmptyWorld()

    handle, world_file = tempfile.mkstemp()

    with open(handle, 'w') as f:
        f.write(world_sdf_string)

    return world_file
