# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import dataclasses
from typing import Tuple

import gym_ignition_models
import pytest
from gym_ignition.utils import misc

from scenario import gazebo as scenario


def id_gazebo_fn(val: Tuple[float, float, float]):
    if isinstance(val, tuple) and len(val) == 3:
        return f"step={val[0]}_rtf={val[1]}_iters={val[2]}"


@pytest.fixture(scope="function")
def gazebo_fixture(request):
    """
    Return an instance of the GazeboSimulator ensuring that it
    is closed even in case of failure.

    Example:

        from ..common.utils import gazebo_fixture as gazebo

        @pytest.mark.parametrize("gazebo", [(0.001, 1.0, 1)], indirect=True)
        def test_foo(gazebo: scenario.gazebo.GazeboSimulator):

            assert gazebo.initialize()
            # ...

        @pytest.mark.parametrize("gazebo, name",
                                 [((0.001, 1.0, 1), "name_1"),
                                  ((0.001, 1.0, 1), "name_2"),
                                  ((0.001, 1.0, 1), "name_3")],
                                  indirect="gazebo")
        def test_foo(gazebo: scenario.gazebo.GazeboSimulator, name: str):

            model_name = name
            assert gazebo.initialize()
            # ...
    """

    step_size, rtf, iterations = request.param
    gazebo = scenario.GazeboSimulator(step_size, rtf, iterations)

    yield gazebo

    gazebo.close()


@pytest.fixture(scope="function")
def default_world_fixture(request):
    """
    Initialize a default world with ground and physics.

    Example:

        from ..common.utils import default_world_fixture as default_world

        @pytest.mark.parametrize("default_world", [(0.001, 1.0, 1)], indirect=True)
        def test_foo(default_world: Tuple[scenario.GazeboSimulator, scenario.World]):

            # Get the simulator and the world
            gazebo, world = default_world

            # ...
    """

    step_size, rtf, iterations = request.param
    gazebo = scenario.GazeboSimulator(step_size, rtf, iterations)

    assert gazebo.initialize()

    world = gazebo.get_world().to_gazebo()
    assert world.insert_model(gym_ignition_models.get_model_file("ground_plane"))
    assert world.set_physics_engine(scenario.PhysicsEngine_dart)

    yield gazebo, world

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

    multi_world_sdf = misc.string_to_file(multi_world_sdf_string)
    return multi_world_sdf


def get_cube_sdf_string() -> str:
    return """
    <?xml version="1.0" ?>
    <sdf version="1.6">
        <model name='box'>
        <pose>0 0 0.5 0 -0 0</pose>
            <link name='box_link'>
                <inertial>
                <inertia>
                    <ixx>1</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>1</iyy>
                    <iyz>0</iyz>
                    <izz>1</izz>
                </inertia>
                <mass>1</mass>
                </inertial>
                <collision name='box_collision'>
                <geometry>
                    <box>
                        <size>1 1 1</size>
                    </box>
                </geometry>
                <surface>
                    <friction>
                        <ode/>
                    </friction>
                    <contact/>
                </surface>
                </collision>
                <visual name='box_visual'>
                <geometry>
                    <box>
                    <size>1 1 1</size>
                    </box>
                </geometry>
                <material>
                    <ambient>1 0 0 1</ambient>
                    <diffuse>1 0 0 1</diffuse>
                    <specular>1 0 0 1</specular>
                </material>
                </visual>
            </link>
        </model>
    </sdf>
    """


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

    model_file = misc.string_to_file(get_cube_urdf_string())
    return model_file


def get_empty_world_sdf() -> str:

    world_sdf_string = scenario.get_empty_world()

    world_file = misc.string_to_file(world_sdf_string)
    return world_file


@dataclasses.dataclass
class SphereURDF:

    mass: float = 5.0
    radius: float = 0.1
    restitution: float = 0

    def urdf(self) -> str:

        i = 2.0 / 5 * self.mass * self.radius * self.radius
        urdf = f"""
            <robot name="sphere_model" xmlns:xacro="http://www.ros.org/wiki/xacro">
                <link name="sphere">
                    <inertial>
                      <origin rpy="0 0 0" xyz="0 0 0"/>
                      <mass value="{self.mass}"/>
                      <inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/>
                    </inertial>
                    <visual>
                      <geometry>
                        <sphere radius="{self.radius}"/>
                      </geometry>
                      <origin rpy="0 0 0" xyz="0 0 0"/>
                    </visual>
                    <collision>
                      <geometry>
                        <sphere radius="{self.radius}"/>
                      </geometry>
                      <origin rpy="0 0 0" xyz="0 0 0"/>
                    </collision>
                </link>
                <gazebo reference="sphere">
                  <collision><surface><bounce>
                    <restitution_coefficient>{self.restitution}</restitution_coefficient>
                  </bounce></surface></collision>
                </gazebo>
            </robot>
            """

        return urdf
