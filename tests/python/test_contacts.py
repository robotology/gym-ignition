# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import tempfile
import numpy as np
from . import utils
from gym_ignition.robots import gazebo_robot


def get_cube_urdf() -> str:
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


class CubeGazeboRobot(gazebo_robot.GazeboRobot):
    def __init__(self, model_file: str, gazebo, initial_position: np.ndarray):
        # Initialize base class
        super().__init__(model_file=model_file,
                         gazebo=gazebo)

        ok_floating = self.set_as_floating_base(True)
        assert ok_floating, "Failed to set the robot as floating base"

        # Initial base position and orientation
        base_position = np.array(initial_position)
        base_orientation = np.array([1., 0., 0., 0.])
        ok_base_pose = self.set_initial_base_pose(base_position, base_orientation)
        assert ok_base_pose, "Failed to set base pose"

        # Insert the model in the simulation
        _ = self.gympp_robot


def test_contacts():
    # Get the simulator
    gazebo = utils.Gazebo(physics_rate=1000, iterations=1, rtf=100)

    # Serialize the cube urdf
    handle, model_file = tempfile.mkstemp()
    with open(handle, 'w') as f:
        f.write(get_cube_urdf())

    # Create the first cube and insert it in the simulation
    cube1 = CubeGazeboRobot(model_file=model_file,
                            gazebo=gazebo.simulator,
                            initial_position=np.array([0, 0, 1.0]))

    # Create the second cube and insert it in the simulation
    cube2 = CubeGazeboRobot(model_file=model_file,
                            gazebo=gazebo.simulator,
                            initial_position=np.array([0, 0, 2.5]))

    # Execute the first simulation step
    gazebo.step()

    # The cubes should be falling without contacts
    assert len(cube1.links_in_contact()) == 0
    assert len(cube2.links_in_contact()) == 0

    # Perform 500 steps.
    for _ in range(500):
        gazebo.step()

    # Cube1 should be touching ground
    assert len(cube1.links_in_contact()) == 1
    assert cube1.links_in_contact()[0] == 'cube'
    contact_data1 = cube1.contact_data('cube')
    assert len(contact_data1) > 0
    assert contact_data1[0].bodyA == cube1.name() + "::cube_collision"
    assert contact_data1[0].bodyB == "ground_plane::collision"

    # Cube 2 should be still floating
    assert len(cube2.links_in_contact()) == 0
    assert len(cube2.contact_data('cube')) == 0

    # Perform 500 steps.
    for _ in range(500):
        gazebo.step()

    # Now cube2 should be in contact with cube1
    assert len(cube2.links_in_contact()) == 1
    assert cube2.links_in_contact()[0] == 'cube'
    contact_data2 = cube2.contact_data('cube')
    assert len(contact_data2) > 0
    assert contact_data2[0].bodyA == cube2.name() + "::cube_collision"
    assert contact_data2[0].bodyB == cube1.name() + "::cube_collision"

    # And cube1 should be in contact with cube2 and ground_plane
    assert len(cube1.links_in_contact()) == 1
    assert cube1.links_in_contact()[0] == 'cube'
    contact_data1 = cube1.contact_data('cube')
    assert len(contact_data1) == 2

    for contact in contact_data1:
        assert contact.bodyA == cube1.name() + "::cube_collision"
        assert contact.bodyB == cube2.name() + "::cube_collision" or \
               contact.bodyB == "ground_plane::collision"
