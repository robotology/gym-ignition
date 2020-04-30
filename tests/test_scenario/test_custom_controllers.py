# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
pytestmark = pytest.mark.scenario

import numpy as np
import gym_ignition_models
from ..common import utils
from ..common.utils import gazebo_fixture as gazebo
from gym_ignition import scenario_bindings as bindings
from gym_ignition.controllers.gazebo import computed_torque_fixed_base as context

# Set the verbosity
bindings.setVerbosity(4)


@pytest.mark.parametrize("gazebo",
                         [(0.001, 5.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_computed_torque_fixed_base(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()
    step_size = gazebo.stepSize()

    # Get the default world
    world = gazebo.getWorld()

    # Insert the physics
    assert world.setPhysicsEngine(bindings.PhysicsEngine_Dart)

    # Get the panda urdf
    panda_urdf = gym_ignition_models.get_model_file("panda")

    # Insert the panda arm
    model_name = "panda"
    assert world.insertModel(panda_urdf, bindings.Pose_Identity(), model_name)

    # import time
    # gazebo.gui()
    # time.sleep(3)

    # Get the model
    panda: bindings.Model = world.getModel(model_name)

    # Set the controller period
    panda.setControllerPeriod(step_size)

    # Create the controller context
    controller_context = context.ComputedTorqueFixedBaseContext(
        name="ComputedTorqueFixedBase",
        kp=[10.0] * panda.dofs(),
        ki=[0.0] * panda.dofs(),
        kd=[3.0] * panda.dofs(),
        urdf=panda_urdf,
        joints=panda.jointNames(),
        gravity=[0, 0, -9.81])

    # Insert the controller
    assert panda.insertModelPlugin("libControllerRunner.so",
                                   "scenario::plugins::gazebo::ControllerRunner",
                                   controller_context.to_xml())

    # Set the references
    assert panda.setJointPositionTargets([0.0] * panda.dofs())
    assert panda.setJointVelocityTargets([0.0] * panda.dofs())
    assert panda.setJointAccelerationTargets([0.0] * panda.dofs())

    joints_no_fingers = [j for j in panda.jointNames() if j.startswith("panda_joint")]
    nr_of_joints = len(joints_no_fingers)
    assert nr_of_joints > 0

    # Reset the joints state
    q0 = [np.deg2rad(45)] * nr_of_joints
    dq0 = [0.1] * nr_of_joints
    assert panda.resetJointPositions(q0, joints_no_fingers)
    assert panda.resetJointVelocities(dq0, joints_no_fingers)

    assert gazebo.run(True)
    assert panda.jointPositions(joints_no_fingers) == pytest.approx(q0)
    assert panda.jointVelocities(joints_no_fingers) == pytest.approx(dq0)

    # Step the simulator for a couple of seconds
    for _ in range(3000):
        gazebo.run()

    # Check that the the references have been reached
    assert panda.jointPositions() == pytest.approx(panda.jointPositionTargets(),
                                                   abs=np.deg2rad(1))
    assert panda.jointVelocities() == pytest.approx(panda.jointVelocityTargets(),
                                                    abs=0.05)

    # Apply an external force
    assert panda.getLink("panda_link4").applyWorldForce([100.0, 0, 0], 0.5)

    for _ in range(4000):
        assert gazebo.run()

    # Check that the the references have been reached
    assert panda.jointPositions() == pytest.approx(panda.jointPositionTargets(),
                                                   abs=np.deg2rad(1))
    assert panda.jointVelocities() == pytest.approx(panda.jointVelocityTargets(),
                                                    abs=0.05)
