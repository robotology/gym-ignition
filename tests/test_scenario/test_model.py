# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
pytestmark = pytest.mark.scenario

from .. import utils
import gym_ignition_models
from ..utils import gazebo_fixture as gazebo
from gym_ignition import scenario_bindings as bindings

# Set the verbosity
bindings.setVerbosity(4)


def get_model(gazebo: bindings.GazeboSimulator,
              gym_ignition_model_name: str) -> bindings.Model:

    world = gazebo.getWorld()
    # TODO: assert world

    assert world.insertWorldPlugin("libPhysicsSystem.so",
                                   "scenario::plugins::gazebo::Physics")

    model_urdf = gym_ignition_models.get_model_file(gym_ignition_model_name)
    assert world.insertModel(model_urdf,
                             bindings.Pose_Identity(),
                             gym_ignition_model_name)

    model = world.getModel(gym_ignition_model_name)
    assert model.id() != 0

    return model


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_model_core_api(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()

    gym_ignition_model_name = "cartpole"
    model = get_model(gazebo, gym_ignition_model_name)

    assert model.id() != 0
    assert model.valid()
    assert model.name() == gym_ignition_model_name

    assert len(model.linkNames()) == model.nrOfLinks()
    assert len(model.jointNames()) == model.nrOfJoints()

    assert model.setControllerPeriod(0.42)
    assert model.controllerPeriod() == 0.42


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_model_joints(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()

    gym_ignition_model_name = "panda"
    model = get_model(gazebo, gym_ignition_model_name)

    q = model.jointPositions()
    assert pytest.approx(q) == [0.0] * model.dofs()

    dq = model.jointVelocities()
    assert pytest.approx(dq) == [0.0] * model.dofs()

    assert model.resetJointPositions([0.05] * model.dofs())
    assert model.jointPositions() == pytest.approx([0.0] * model.dofs())
    gazebo.run(paused=True)
    assert model.jointPositions() == pytest.approx([0.05] * model.dofs())
    assert model.jointVelocities() == pytest.approx([0.0] * model.dofs())

    gazebo.run(paused=False)
    assert model.jointVelocities() != pytest.approx([0.0] * model.dofs())

    assert model.resetJointVelocities([-0.1] * model.dofs())
    assert model.jointVelocities() != pytest.approx([-0.1] * model.dofs())
    gazebo.run(paused=True)
    assert model.jointVelocities() == pytest.approx([-0.1] * model.dofs())

    assert model.resetJointPositions([0.0] * model.dofs())
    assert model.resetJointVelocities([0.0] * model.dofs())

    joint_subset = model.jointNames()[0:4]
    assert model.resetJointPositions([-0.4] * len(joint_subset), joint_subset)
    assert model.resetJointVelocities([3.0] * len(joint_subset), joint_subset)
    gazebo.run(paused=True)
    assert model.jointPositions(joint_subset) == pytest.approx([-0.4] * len(joint_subset))
    assert model.jointVelocities(joint_subset) == pytest.approx([3.0] * len(joint_subset))
    assert model.jointPositions() == pytest.approx(
        [-0.4] * len(joint_subset) + [0.0] * (model.dofs() - len(joint_subset)))
    assert model.jointVelocities() == pytest.approx(
        [3.0] * len(joint_subset) + [0.0] * (model.dofs() - len(joint_subset)))


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_model_base(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()

    gym_ignition_model_name = "pendulum"
    model = get_model(gazebo, gym_ignition_model_name)
    assert gym_ignition_model_name in gazebo.getWorld().modelNames()

    assert model.baseFrame() == "support"
    assert model.setBaseFrame("support")
    # assert model.setBaseFrame("pendulum")  # TODO: Not yet supported

    # Check that the pose is the identical
    gazebo.run(paused=True)
    assert model.basePosition() == pytest.approx([0, 0, 0])
    assert model.baseOrientation() == pytest.approx([1, 0, 0, 0])

    # Reset the base pose
    new_base_pose = dict(position=[5, 5, 0], orientation=[0, 1, 0, 0])
    assert model.resetBasePose(new_base_pose['position'], new_base_pose['orientation'])

    # Before stepping the simulation the pose should be the initial one
    assert model.basePosition() == pytest.approx([0, 0, 0])
    assert model.baseOrientation() == pytest.approx([1, 0, 0, 0])

    # Step the simulator and check that the pose changes
    gazebo.run(paused=True)
    assert model.basePosition() == pytest.approx(new_base_pose['position'])
    assert model.baseOrientation() == pytest.approx(new_base_pose['orientation'])

    # Reset the linear velocity
    lin_velocity = [0, 0, 5.0]
    assert model.resetBaseLinearVelocity(lin_velocity)
    assert model.baseLinearVelocity() == pytest.approx([0, 0, 0])
    gazebo.run()
    assert model.baseLinearVelocity() == pytest.approx(lin_velocity, abs=0.01)

    # The linear velocity of the support must be the same
    assert "support" in model.linkNames()
    assert model.getLink("support").worldLinearVelocity() == pytest.approx(lin_velocity,
                                                                           abs=0.01)

    # Reset the angular velocity
    ang_velocity = [0.1, 0.5, -3.0]
    assert model.resetBaseAngularVelocity(ang_velocity)
    assert model.baseAngularVelocity() == pytest.approx([0, 0, 0])
    gazebo.run()
    assert model.baseAngularVelocity() == pytest.approx(ang_velocity, abs=0.01)

    # The angular velocity of the support must be the same
    assert "support" in model.linkNames()
    assert model.getLink("support").worldAngularVelocity() == pytest.approx(ang_velocity,
                                                                            abs=0.01)


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_model_references(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()

    gym_ignition_model_name = "cartpole"
    model = get_model(gazebo, gym_ignition_model_name)
    assert gym_ignition_model_name in gazebo.getWorld().modelNames()

    assert model.setJointPositionTargets([0.5, -3])
    assert model.jointPositionTargets() == pytest.approx([0.5, -3])
    assert model.jointPositionTargets(["pivot"]) == pytest.approx([-3])

    assert model.setJointVelocityTargets([-0.1, 6])
    assert model.jointVelocityTargets() == pytest.approx([-0.1, 6])
    assert model.jointVelocityTargets(["pivot"]) == pytest.approx([6])

    assert model.setJointAccelerationTargets([-0, 3.14])
    assert model.jointAccelerationTargets() == pytest.approx([-0, 3.14])
    assert model.jointAccelerationTargets(["pivot"]) == pytest.approx([3.14])

    # assert not model.setJointGeneralizedForceTargets([20.1, -13])
    # assert model.setJointControlMode(bindings.JointControlMode_Force)
    # assert model.setJointGeneralizedForceTargets([20.1, -13])
    # assert model.jointGeneralizedForceTargets() == pytest.approx([20.1, -13])
    # assert model.jointGeneralizedForceTargets(["pivot"]) == pytest.approx([-13])

    assert model.setBasePoseTarget([0, 0, 5], [0, 0, 0, 1.0])
    assert model.setBaseOrientationTarget([0, 0, 1.0, 0])
    assert model.basePositionTarget() == pytest.approx([0, 0, 5])
    assert model.baseOrientationTarget() == pytest.approx([0, 0, 1.0, 0])

    assert model.setBaseLinearVelocityTarget([1, 2, 3])
    assert model.setBaseAngularVelocityTarget([4, 5, 6])
    assert model.setBaseAngularAccelerationTarget([-1, -2, -3])
    assert model.baseLinearVelocityTarget() == pytest.approx([1, 2, 3])
    assert model.baseAngularVelocityTarget() == pytest.approx([4, 5, 6])
    assert model.baseAngularAccelerationTarget() == pytest.approx([-1, -2, -3])


# def test_model_contacts():
#     pass
