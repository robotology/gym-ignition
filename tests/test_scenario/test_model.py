# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
pytestmark = pytest.mark.scenario

import numpy as np
from typing import Tuple
from ..common import utils
import gym_ignition_models
from ..common.utils import gazebo_fixture as gazebo
from gym_ignition import scenario_bindings as bindings
from ..common.utils import default_world_fixture as default_world

# Set the verbosity
bindings.set_verbosity(4)


def get_model(gazebo: bindings.GazeboSimulator,
              gym_ignition_model_name: str) -> bindings.Model:

    world = gazebo.get_world()
    # TODO: assert world

    assert world.set_physics_engine(bindings.PhysicsEngine_dart)

    model_urdf = gym_ignition_models.get_model_file(gym_ignition_model_name)
    assert world.insert_model(model_urdf,
                              bindings.Pose_identity(),
                              gym_ignition_model_name)

    model = world.get_model(gym_ignition_model_name)
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

    assert len(model.link_names()) == model.nr_of_links()
    assert len(model.joint_names()) == model.nr_of_joints()

    assert model.set_controller_period(0.42)
    assert model.controller_period() == 0.42


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_model_joints(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()

    gym_ignition_model_name = "panda"
    model = get_model(gazebo, gym_ignition_model_name)

    q = model.joint_positions()
    assert pytest.approx(q) == [0.0] * model.dofs()

    dq = model.joint_velocities()
    assert pytest.approx(dq) == [0.0] * model.dofs()

    assert model.reset_joint_positions([0.05] * model.dofs())
    assert model.joint_positions() == pytest.approx([0.0] * model.dofs())
    gazebo.run(paused=True)
    assert model.joint_positions() == pytest.approx([0.05] * model.dofs())
    assert model.joint_velocities() == pytest.approx([0.0] * model.dofs())

    gazebo.run(paused=False)
    assert model.joint_velocities() != pytest.approx([0.0] * model.dofs())

    assert model.reset_joint_velocities([-0.1] * model.dofs())
    assert model.joint_velocities() != pytest.approx([-0.1] * model.dofs())
    gazebo.run(paused=True)
    assert model.joint_velocities() == pytest.approx([-0.1] * model.dofs())

    assert model.reset_joint_positions([0.0] * model.dofs())
    assert model.reset_joint_velocities([0.0] * model.dofs())

    joint_subset = model.joint_names()[0:4]
    assert model.reset_joint_positions([-0.4] * len(joint_subset), joint_subset)
    assert model.reset_joint_velocities([3.0] * len(joint_subset), joint_subset)
    gazebo.run(paused=True)
    assert model.joint_positions(joint_subset) == \
        pytest.approx([-0.4] * len(joint_subset))
    assert model.joint_velocities(joint_subset) == \
        pytest.approx([3.0] * len(joint_subset))
    assert model.joint_positions() == pytest.approx(
        [-0.4] * len(joint_subset) + [0.0] * (model.dofs() - len(joint_subset)))
    assert model.joint_velocities() == pytest.approx(
        [3.0] * len(joint_subset) + [0.0] * (model.dofs() - len(joint_subset)))


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_model_base(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()

    gym_ignition_model_name = "pendulum"
    model = get_model(gazebo, gym_ignition_model_name)
    assert gym_ignition_model_name in gazebo.get_world().model_names()

    assert model.base_frame() == "support"
    assert model.set_base_frame("support")
    # assert model.set_base_frame("pendulum")  # TODO: Not yet supported

    # Check that the pose is the identical
    gazebo.run(paused=True)
    assert model.base_position() == pytest.approx([0, 0, 0])
    assert model.base_orientation() == pytest.approx([1, 0, 0, 0])

    # Reset the base pose
    new_base_pose = dict(position=[5, 5, 0], orientation=[0, 1, 0, 0])
    assert model.reset_base_pose(new_base_pose['position'], new_base_pose['orientation'])

    # Before stepping the simulation the pose should be the initial one
    assert model.base_position() == pytest.approx([0, 0, 0])
    assert model.base_orientation() == pytest.approx([1, 0, 0, 0])

    # Step the simulator and check that the pose changes
    gazebo.run(paused=True)
    assert model.base_position() == pytest.approx(new_base_pose['position'])
    assert model.base_orientation() == pytest.approx(new_base_pose['orientation'])

    # Reset the linear velocity
    lin_velocity = [0, 0, 5.0]
    assert model.reset_base_world_linear_velocity(lin_velocity)
    assert model.base_world_linear_velocity() == pytest.approx([0, 0, 0])
    gazebo.run()
    assert model.base_world_linear_velocity() == pytest.approx(lin_velocity, abs=0.01)

    # The linear velocity of the support must be the same
    assert "support" in model.link_names()
    assert model.get_link("support").world_linear_velocity() == \
        pytest.approx(lin_velocity, abs=0.01)

    # Reset the angular velocity
    ang_velocity = [0.1, 0.5, -3.0]
    assert model.reset_base_world_angular_velocity(ang_velocity)
    assert model.base_world_angular_velocity() == pytest.approx([0, 0, 0])
    gazebo.run()
    assert model.base_world_angular_velocity() == pytest.approx(ang_velocity, abs=0.01)

    # The angular velocity of the support must be the same
    assert "support" in model.link_names()
    assert model.get_link("support").world_angular_velocity() == \
        pytest.approx(ang_velocity, abs=0.01)


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_model_references(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()

    gym_ignition_model_name = "cartpole"
    model = get_model(gazebo, gym_ignition_model_name)
    assert gym_ignition_model_name in gazebo.get_world().model_names()

    assert model.set_joint_position_targets([0.5, -3])
    assert model.joint_position_targets() == pytest.approx([0.5, -3])
    assert model.joint_position_targets(["pivot"]) == pytest.approx([-3])

    assert model.set_joint_velocity_targets([-0.1, 6])
    assert model.joint_velocity_targets() == pytest.approx([-0.1, 6])
    assert model.joint_velocity_targets(["pivot"]) == pytest.approx([6])

    assert model.set_joint_acceleration_targets([-0, 3.14])
    assert model.joint_acceleration_targets() == pytest.approx([-0, 3.14])
    assert model.joint_acceleration_targets(["pivot"]) == pytest.approx([3.14])

    # TODO
    # assert not model.set_joint_generalized_force_targets([20.1, -13])
    # assert model.set_joint_control_mode()(bindings.JointControlMode_force)
    # assert model.set_joint_generalized_force_targets([20.1, -13])
    # assert model.joint_generalized_force_targets() == pytest.approx([20.1, -13])
    # assert model.joint_generalized_force_targets(["pivot"]) == pytest.approx([-13])

    assert model.set_base_pose_target([0, 0, 5], [0, 0, 0, 1.0])
    assert model.set_base_orientation_target([0, 0, 1.0, 0])
    assert model.base_position_target() == pytest.approx([0, 0, 5])
    assert model.base_orientation_target() == pytest.approx([0, 0, 1.0, 0])

    assert model.set_base_world_linear_velocity_target([1, 2, 3])
    assert model.set_base_world_angular_velocity_target([4, 5, 6])
    assert model.set_base_world_angular_acceleration_target([-1, -2, -3])
    assert model.base_world_linear_velocity_target() == pytest.approx([1, 2, 3])
    assert model.base_world_angular_velocity_target() == pytest.approx([4, 5, 6])
    assert model.base_world_angular_acceleration_target() == pytest.approx([-1, -2, -3])


# def test_model_contacts():
#     pass


@pytest.mark.parametrize("default_world", [(1.0 / 1_000, 1.0, 1)], indirect=True)
def test_history_of_joint_forces(
        default_world: Tuple[bindings.GazeboSimulator, bindings.World]):

    # Get the simulator and the world
    gazebo, world = default_world

    # Insert a panda model
    panda_urdf = gym_ignition_models.get_model_file("panda")
    assert world.insert_model(panda_urdf)
    assert "panda" in world.model_names()

    # Get the model
    panda = world.get_model("panda")

    # Control the robot in Force
    assert panda.set_joint_control_mode(bindings.JointControlMode_force)

    # Enable the history for 3 steps
    assert panda.enable_history_of_applied_joint_forces(True, 3)

    # Initialize the torques applied in the first run
    torques = np.zeros(panda.dofs())

    # History of joint forces
    history_last_three_runs = np.zeros(panda.dofs() * 3)

    for _ in range(10):

        torques += 0.1
        assert panda.set_joint_generalized_force_targets(torques)

        gazebo.run()

        history_last_three_runs = np.concatenate((history_last_three_runs, torques))
        history_last_three_runs = history_last_three_runs[-panda.dofs() * 3:]

        assert panda.history_of_applied_joint_forces() == \
            pytest.approx(history_last_three_runs)
