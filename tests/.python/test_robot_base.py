# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
import pytest

from . import utils


@pytest.mark.parametrize("simulator_name", ["gazebo", "pybullet"])
def test_robot_fixed_base(simulator_name: str):

    if simulator_name == "gazebo":
        simulator = utils.Gazebo(physics_rate=1000)
    elif simulator_name == "pybullet":
        simulator = utils.PyBullet(physics_rate=1000)
    else:
        raise ValueError(simulator_name)

    # Get the pendulum robot and specify its base position
    base_position = np.array([-3, 3, 2])
    robot = utils.get_pendulum(simulator=simulator, base_position=base_position)

    # Robot should not fall due to gravity
    for _ in range(100):
        simulator.step()

    assert np.allclose(robot.base_pose()[0], base_position, atol=1e-3)
    assert np.allclose(robot.base_pose()[1], np.array([1.0, 0, 0, 0]), atol=1e-3)


# @pytest.mark.parametrize("simulator_name", ["gazebo", "pybullet"])
# def test_robot_floating_base(simulator_name: str):
#
#     if simulator_name == "gazebo":
#         simulator = utils.Gazebo(physics_rate=1000)
#     elif simulator_name == "pybullet":
#         simulator = utils.PyBullet(physics_rate=1000)
#     else:
#         raise ValueError(simulator_name)
#
#     # Get the pendulum robot and specify its base position
#     base_position = np.array([-3, 3, 2])
#     robot = utils.get_pendulum(simulator=simulator, base_position=base_position)
#
#     assert np.allclose(robot.base_pose()[0], base_position, atol=1e-3)
#     assert np.allclose(robot.base_pose()[1], np.array([1.0, 0, 0, 0]), atol=1e-3)
#
#     old_pos_z = base_position[2]
#
#     # Robot will fall due to gravity
#     for _ in range(100):
#         simulator.step()
#         current_pos_z = robot.base_pose()[0][2]
#         assert current_pos_z < old_pos_z
#         old_pos_z = current_pos_z


@pytest.mark.parametrize("simulator_name", ["pybullet"])
def test_robot_floating_to_fixed(simulator_name: str):

    if simulator_name == "gazebo":
        simulator = utils.Gazebo(physics_rate=1000)
    elif simulator_name == "pybullet":
        simulator = utils.PyBullet(physics_rate=1000)
    else:
        raise ValueError(simulator_name)

    # Get the pendulum robot and specify its base position
    base_position = np.array([-3, 3, 2])
    robot = utils.get_pendulum(simulator=simulator, base_position=base_position)

    # Let's make the robot fall a little
    for _ in range(10):
        simulator.step()

    current_base_position, current_base_orientation = robot.base_pose()
    assert current_base_position[2] < base_position[2]

    ok_fixed_base = robot.set_as_floating_base(False)
    assert ok_fixed_base, "Failed to set the robot as fixed base"

    ok_reset_base_pose = robot.reset_base_pose(
        current_base_position, current_base_orientation
    )
    assert ok_reset_base_pose, "Failed to reset the base pose"

    # Robot should now not fall due to gravity
    for _ in range(100):
        simulator.step()

    assert np.allclose(robot.base_pose()[0], current_base_position, atol=1e-3)
    assert np.allclose(robot.base_pose()[1], current_base_orientation, atol=1e-3)
