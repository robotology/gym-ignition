# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.scenario

from typing import Tuple

import gym_ignition_models
import numpy as np

from scenario import core as scenario_core
from scenario import gazebo as scenario_gazebo

from ..common.utils import default_world_fixture as default_world

# Set the verbosity
scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_debug)


@pytest.mark.parametrize("default_world", [(1.0 / 1_000, 1.0, 1)], indirect=True)
def test_velocity_direct(
    default_world: Tuple[scenario_gazebo.GazeboSimulator, scenario_gazebo.World]
):

    # Get the simulator and the world
    gazebo, world = default_world

    # Get the URDF model
    pendulum_urdf = gym_ignition_models.get_model_file("pendulum")

    # Insert a pendulum model
    assert world.insert_model(pendulum_urdf)
    assert "pendulum" in world.model_names()

    # Get the model and cast it to Gazebo
    pendulum = world.get_model("pendulum").to_gazebo()

    # Add some friction
    assert pendulum.get_joint("pivot").to_gazebo().set_coulomb_friction(value=0.01)
    assert pendulum.get_joint("pivot").to_gazebo().set_viscous_friction(value=0.2)

    # Show the GUI
    # import time
    # gazebo.gui()
    # time.sleep(2)

    # Get the pivot joint
    assert "pivot" in pendulum.joint_names()
    pivot = pendulum.get_joint(joint_name="pivot")

    # Reset the joint to 90 degs and make it swing
    assert pivot.to_gazebo().reset_position(position=np.deg2rad(90))
    [gazebo.run() for _ in range(5_000)]

    # It should rest in its stable equilibrium point
    assert np.deg2rad(179.7) <= pivot.position() <= np.deg2rad(180.3)

    # Control the joint in velocity direct
    assert pivot.set_control_mode(
        mode=scenario_core.JointControlMode_velocity_follower_dart
    )

    # Set the velocity reference to 3.14 rad / s
    assert pivot.set_velocity_target(velocity=np.pi)

    # The target should be reached after just one run
    assert gazebo.run()
    assert pivot.velocity() == pytest.approx(np.pi)

    # Simulate for a while
    [gazebo.run() for _ in range(1_500)]

    # Check again
    assert pivot.velocity() == pytest.approx(np.pi)

    # Change direction abruptly
    assert pivot.set_velocity_target(velocity=-np.pi)
    assert gazebo.run()
    assert pivot.velocity() == pytest.approx(-np.pi)

    # Go in idle and make it swing
    assert pivot.set_control_mode(mode=scenario_core.JointControlMode_idle)
    [gazebo.run() for _ in range(5_000)]

    # It should rest in its stable equilibrium point
    assert (
        2 * np.pi + np.deg2rad(179.7)
        <= pivot.position()
        <= 2 * np.pi + np.deg2rad(180.3)
    )
