# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.scenario

from typing import Tuple

import gym_ignition_models
import numpy as np

from scenario import core
from scenario import gazebo as scenario

from ..common.utils import default_world_fixture as default_world

# Set the verbosity
scenario.set_verbosity(scenario.Verbosity_debug)

# Panda PID gains
# https://github.com/mkrizmancic/franka_gazebo/blob/master/config/default.yaml
panda_pid_gains_1000Hz = {
    "panda_joint1": core.PID(50, 0, 20),
    "panda_joint2": core.PID(10000, 0, 500),
    "panda_joint3": core.PID(100, 0, 10),
    "panda_joint4": core.PID(1000, 0, 50),
    "panda_joint5": core.PID(100, 0, 10),
    "panda_joint6": core.PID(100, 0, 10),
    "panda_joint7": core.PID(10, 0.5, 0.1),
    "panda_finger_joint1": core.PID(100, 0, 50),
    "panda_finger_joint2": core.PID(100, 0, 50),
}


@pytest.mark.parametrize("default_world", [(1.0 / 1_000, 1.0, 1)], indirect=True)
def test_position_pid(default_world: Tuple[scenario.GazeboSimulator, scenario.World]):

    # Get the simulator and the world
    gazebo, world = default_world

    # Insert a panda model
    panda_urdf = gym_ignition_models.get_model_file("panda")
    assert world.insert_model(panda_urdf)
    assert "panda" in world.model_names()

    # Show the GUI
    # import time
    # gazebo.gui()
    # gazebo.run(paused=True)
    # time.sleep(3)

    # Get the model and cast it to Gazebo
    panda = world.get_model("panda").to_gazebo()

    # Reset joint1 to its middle position
    joint1 = panda.get_joint("panda_joint1").to_gazebo()
    joint1_range = np.abs(joint1.position_limit().max - joint1.position_limit().min)
    joint1_middle = joint1.position_limit().min + joint1_range / 2
    assert joint1.reset_position(joint1_middle)

    # Reset joint6 to its middle position
    joint6 = panda.get_joint("panda_joint6").to_gazebo()
    joint6_range = np.abs(joint6.position_limit().max - joint6.position_limit().min)
    joint6_middle = joint6.position_limit().min + joint6_range / 2
    assert joint6.reset_position(joint6_middle)

    # Update the model state without stepping the physics
    assert gazebo.run(paused=True)

    # Set the controller period equal to the physics step (1000Hz)
    panda.set_controller_period(gazebo.step_size())

    # Check that we have gains for all joints
    assert set(panda.joint_names()) == set(panda_pid_gains_1000Hz.keys())

    # Set the PID gains
    for joint_name, pid in panda_pid_gains_1000Hz.items():
        assert panda.get_joint(joint_name).set_pid(pid=pid)

    # Switch to position control mode (it sets the current position as active target)
    assert panda.set_joint_control_mode(core.JointControlMode_position)
    assert panda.joint_position_targets() == pytest.approx(panda.joint_positions())

    # Just fight gravity for a while
    for _ in range(1_000):
        assert gazebo.run()

    # Check that it didn't move
    assert panda.joint_positions() == pytest.approx(
        panda.joint_position_targets(), abs=np.deg2rad(1)
    )

    # joint1 trajectory
    q0_joint1 = joint1.position()
    q_joint1 = (
        0.9 * joint1_range / 2 * np.sin(2 * np.pi * 0.33 * t)
        for t in np.arange(start=0, stop=10.0, step=gazebo.step_size())
    )

    # joint6 trajectory
    q0_joint6 = joint6.position()
    q_joint6 = (
        0.9 * joint6_range / 2 * np.sin(2 * np.pi * 0.33 * t)
        for t in np.arange(start=0, stop=10.0, step=gazebo.step_size())
    )

    for _ in range(5_000):

        # Generate the new references
        joint1_reference = q0_joint1 + next(q_joint1)
        joint6_reference = q0_joint6 + next(q_joint6)

        # Set the new references
        assert joint1.set_position_target(position=joint1_reference)
        assert joint6.set_position_target(position=joint6_reference)

        # Run the simulation
        assert gazebo.run()

        # Check that trajectory is being followed
        assert joint1.position() == pytest.approx(joint1_reference, abs=np.deg2rad(3))
        assert joint6.position() == pytest.approx(joint6_reference, abs=np.deg2rad(3))
