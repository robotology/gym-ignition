# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import tempfile

import numpy as np

from . import utils


def test_external_force():
    # Get the simulator
    gazebo = utils.Gazebo(physics_rate=1000, iterations=1, rtf=100)

    # Create the first cube and insert it in the simulation
    cube = utils.CubeGazeboRobot(
        gazebo=gazebo.simulator, initial_position=np.array([0, 0, 1.0])
    )

    # Execute the first simulation step
    gazebo.step()

    # Get the position of the cube
    position, _ = cube.base_pose()
    assert position[0] == 0.0
    assert position[1] == 0.0

    # Apply a force for 1 second
    num_steps = 100
    f_x = 50.0
    f_y = -f_x / 10

    for _ in range(num_steps):
        ok_w = cube.apply_external_force(
            "cube", np.array([f_x, f_y, 0]), np.array([0.0, 0, 0])
        )
        assert ok_w

        # Step the simulation
        gazebo.step()

    # Get the position of the cube
    position, _ = cube.base_pose()
    print(position)
    assert position[0] > 0.0
    assert position[1] < 0.0
    assert np.allclose(-position[0], 10 * position[1])
