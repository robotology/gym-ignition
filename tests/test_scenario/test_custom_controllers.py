# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.scenario

import gym_ignition_models
import numpy as np
from gym_ignition.context.gazebo import controllers

from scenario import core
from scenario import gazebo as scenario

from ..common import utils
from ..common.utils import gazebo_fixture as gazebo

# Set the verbosity
scenario.set_verbosity(scenario.Verbosity_debug)


@pytest.mark.parametrize(
    "gazebo", [(0.001, 5.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_computed_torque_fixed_base(gazebo: scenario.GazeboSimulator):

    assert gazebo.initialize()
    step_size = gazebo.step_size()

    # Get the default world
    world = gazebo.get_world()

    # Insert the physics
    assert world.set_physics_engine(scenario.PhysicsEngine_dart)

    # Get the panda urdf
    panda_urdf = gym_ignition_models.get_model_file("panda")

    # Insert the panda arm
    model_name = "panda"
    assert world.insert_model(panda_urdf, core.Pose_identity(), model_name)

    # import time
    # gazebo.gui()
    # time.sleep(3)
    # gazebo.run(paused=True)

    # Get the model
    panda = world.get_model(model_name).to_gazebo()

    # Set the controller period
    panda.set_controller_period(step_size)

    # Insert the controller
    assert panda.insert_model_plugin(
        *controllers.ComputedTorqueFixedBase(
            kp=[10.0] * panda.dofs(),
            ki=[0.0] * panda.dofs(),
            kd=[3.0] * panda.dofs(),
            urdf=panda_urdf,
            joints=panda.joint_names(),
        ).args()
    )

    # Set the references
    assert panda.set_joint_position_targets([0.0] * panda.dofs())
    assert panda.set_joint_velocity_targets([0.0] * panda.dofs())
    assert panda.set_joint_acceleration_targets([0.0] * panda.dofs())

    joints_no_fingers = [j for j in panda.joint_names() if j.startswith("panda_joint")]
    nr_of_joints = len(joints_no_fingers)
    assert nr_of_joints > 0

    # Reset the joints state
    q0 = [np.deg2rad(45)] * nr_of_joints
    dq0 = [0.1] * nr_of_joints
    assert panda.reset_joint_positions(q0, joints_no_fingers)
    assert panda.reset_joint_velocities(dq0, joints_no_fingers)

    assert gazebo.run(True)
    assert panda.joint_positions(joints_no_fingers) == pytest.approx(q0)
    assert panda.joint_velocities(joints_no_fingers) == pytest.approx(dq0)

    # Step the simulator for a couple of seconds
    for _ in range(3000):
        gazebo.run()

    # Check that the the references have been reached
    assert panda.joint_positions() == pytest.approx(
        panda.joint_position_targets(), abs=np.deg2rad(1)
    )
    assert panda.joint_velocities() == pytest.approx(
        panda.joint_velocity_targets(), abs=0.05
    )

    # Apply an external force
    assert (
        panda.get_link("panda_link4").to_gazebo().apply_world_force([100.0, 0, 0], 0.5)
    )

    for _ in range(4000):
        assert gazebo.run()

    # Check that the the references have been reached
    assert panda.joint_positions() == pytest.approx(
        panda.joint_position_targets(), abs=np.deg2rad(1)
    )
    assert panda.joint_velocities() == pytest.approx(
        panda.joint_velocity_targets(), abs=0.05
    )
