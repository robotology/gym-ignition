# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.scenario

import gym_ignition_models
import numpy as np
from gym_ignition.utils import misc

from scenario import core as scenario_core
from scenario import gazebo as scenario_gazebo

from ..common import utils
from ..common.utils import gazebo_fixture as gazebo

# Set the verbosity
scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_debug)


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_pendulum(gazebo: scenario_gazebo.GazeboSimulator):

    # Load two different worlds
    empty_world_sdf = utils.get_empty_world_sdf()
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "dart")
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "bullet")

    # Initialize the simulator
    assert gazebo.initialize()

    # Get the urdf files of the models
    pendulum_urdf = gym_ignition_models.get_model_file(robot_name="pendulum")
    ground_plane_urdf = gym_ignition_models.get_model_file(robot_name="ground_plane")

    # Populate the DART world
    world_dart = gazebo.get_world("dart")
    assert world_dart.set_physics_engine(scenario_gazebo.PhysicsEngine_dart)
    assert world_dart.insert_model(ground_plane_urdf)
    assert world_dart.insert_model(pendulum_urdf)
    pendulum_dart = world_dart.get_model(model_name="pendulum")

    # Populate the Bullet world
    world_bullet = gazebo.get_world("bullet")
    assert world_bullet.set_physics_engine(scenario_gazebo.PhysicsEngine_bullet)
    assert world_bullet.insert_model(ground_plane_urdf)
    assert world_bullet.insert_model(pendulum_urdf)
    pendulum_bullet = world_bullet.get_model(model_name="pendulum")

    # Disable force and velocity limits
    _ = [
        j.to_gazebo().set_generalized_force_target(np.finfo(float).max)
        for j in pendulum_dart.joints() + pendulum_bullet.joints()
    ]
    _ = [
        j.to_gazebo().set_velocity_limit(np.finfo(float).max)
        for j in pendulum_dart.joints()
    ]

    # Reset the pole position
    # Note: this is not yet implemented in the bullet plugin, we apply a small force
    #       to make the pendulum swing
    # assert pendulum_dart.get_joint(joint_name="pivot").to_gazebo().\
    #     reset_position(position=0.01)
    # assert pendulum_bullet.get_joint(joint_name="pivot").to_gazebo().\
    #     reset_position(position=0.01)

    # Control the joints in force
    assert pendulum_dart.set_joint_control_mode(scenario_core.JointControlMode_force)
    assert pendulum_bullet.set_joint_control_mode(scenario_core.JointControlMode_force)

    # Give a small push to the pole
    assert pendulum_dart.get_joint(joint_name="pivot").set_generalized_force_target(
        force=50.0
    )
    assert pendulum_bullet.get_joint(joint_name="pivot").set_generalized_force_target(
        force=50.0
    )
    gazebo.run()
    assert pendulum_dart.get_joint(joint_name="pivot").set_generalized_force_target(
        force=0.0
    )
    assert pendulum_bullet.get_joint(joint_name="pivot").set_generalized_force_target(
        force=0.0
    )

    # Integration time
    dt = gazebo.step_size() * gazebo.steps_per_run()

    for _ in np.arange(start=0.0, stop=1.0, step=dt):

        # Run the simulator
        assert gazebo.run()

        # Check that both worlds evolve similarly
        assert pendulum_dart.joint_positions() == pytest.approx(
            pendulum_bullet.joint_positions(), abs=1e-03
        )
        assert pendulum_dart.joint_velocities() == pytest.approx(
            pendulum_bullet.joint_velocities(), abs=1e-02
        )


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_bouncing_ball(gazebo: scenario_gazebo.GazeboSimulator):

    # Load two different worlds
    empty_world_sdf = utils.get_empty_world_sdf()
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "dart")
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "bullet")

    # Initialize the simulator
    assert gazebo.initialize()

    sphere_urdf_string = utils.SphereURDF(restitution=0.8).urdf()
    sphere_urdf = misc.string_to_file(sphere_urdf_string)
    ground_plane_urdf = gym_ignition_models.get_model_file(robot_name="ground_plane")

    # Pose of the sphere
    sphere_pose = scenario_core.Pose([0, 0, 0.5], [1, 0, 0, 0])

    # Populate the DART world
    world_dart = gazebo.get_world("dart")
    assert world_dart.set_physics_engine(scenario_gazebo.PhysicsEngine_dart)
    assert world_dart.insert_model(ground_plane_urdf)
    assert world_dart.insert_model(sphere_urdf, sphere_pose)
    sphere_dart = world_dart.get_model(model_name="sphere_model")

    # Populate the Bullet world
    world_bullet = gazebo.get_world("bullet")
    assert world_bullet.set_physics_engine(scenario_gazebo.PhysicsEngine_bullet)
    assert world_bullet.insert_model(ground_plane_urdf)
    assert world_bullet.insert_model(sphere_urdf, sphere_pose)
    sphere_bullet = world_bullet.get_model(model_name="sphere_model")

    # Integration time
    dt = gazebo.step_size() * gazebo.steps_per_run()

    # Enable contacts
    assert sphere_dart.enable_contacts(True)
    # assert sphere_bullet.enable_contacts(True)

    for _ in np.arange(start=0.0, stop=3.0, step=dt):

        # Run the simulator
        assert gazebo.run()

        # TODO: Bullet does not have bouncing yet, we stop the simulation when the in dart
        #       a contact is detected
        if len(sphere_dart.contacts()) > 0:
            break

        # Check that both worlds evolve similarly
        assert sphere_dart.base_position() == pytest.approx(
            sphere_bullet.base_position(), abs=0.005
        )
        # assert sphere_dart.base_world_linear_velocity() == \
        #        pytest.approx(sphere_bullet.base_world_linear_velocity(), abs=0.001)
        assert sphere_dart.base_world_angular_velocity() == pytest.approx(
            sphere_bullet.base_world_angular_velocity()
        )


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_position_controller(gazebo: scenario_gazebo.GazeboSimulator):

    # Load two different worlds
    empty_world_sdf = utils.get_empty_world_sdf()
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "dart")
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "bullet")

    # Initialize the simulator
    assert gazebo.initialize()

    # Get the urdf files of the models
    pendulum_urdf = gym_ignition_models.get_model_file(robot_name="pendulum")
    ground_plane_urdf = gym_ignition_models.get_model_file(robot_name="ground_plane")

    # Populate the DART world
    world_dart = gazebo.get_world("dart")
    assert world_dart.set_physics_engine(scenario_gazebo.PhysicsEngine_dart)
    assert world_dart.insert_model(ground_plane_urdf)
    assert world_dart.insert_model(pendulum_urdf)
    pendulum_dart = world_dart.get_model(model_name="pendulum")

    # Populate the Bullet world
    world_bullet = gazebo.get_world("bullet")
    assert world_bullet.set_physics_engine(scenario_gazebo.PhysicsEngine_bullet)
    assert world_bullet.insert_model(ground_plane_urdf)
    assert world_bullet.insert_model(pendulum_urdf)
    pendulum_bullet = world_bullet.get_model(model_name="pendulum")

    # Get the pivot joints
    pivot_dart = pendulum_dart.get_joint("pivot").to_gazebo()
    pivot_bullet = pendulum_dart.get_joint("pivot").to_gazebo()

    # Disable force and velocity limits
    _ = [
        j.set_max_generalized_force(np.finfo(float).max)
        for j in pendulum_dart.joints() + pendulum_bullet.joints()
    ]
    _ = [j.set_velocity_limit(np.finfo(float).max) for j in pendulum_dart.joints()]

    # TODO: friction

    # Update the model state without stepping the physics
    assert gazebo.run(paused=True)

    # Control the joints in force
    assert pivot_dart.set_control_mode(scenario_core.JointControlMode_force)
    assert pivot_bullet.set_control_mode(scenario_core.JointControlMode_force)

    # Give a small push to the pole
    for j in (pivot_dart, pivot_bullet):
        assert j.set_generalized_force_target(force=50.0)
    gazebo.run()
    for j in (pivot_dart, pivot_bullet):
        assert j.set_generalized_force_target(force=0.0)

    # Make it swing for a while
    dt = gazebo.step_size() * gazebo.steps_per_run()
    for _ in np.arange(start=0.0, step=dt, stop=0.5):
        assert gazebo.run()
        assert pivot_dart.position() == pytest.approx(pivot_bullet.position())

    # Set the controller period equal to the physics step (1000Hz)
    assert pendulum_dart.set_controller_period(gazebo.step_size())
    assert pendulum_bullet.set_controller_period(gazebo.step_size())

    for j in (pivot_dart, pivot_bullet):
        # Set the PID gain
        pid = scenario_core.PID(100, 0, 20)
        assert j.set_pid(pid=pid)

    for m in (pendulum_dart, pendulum_bullet):
        # Switch to position control mode (it sets the current position as active target)
        assert m.set_joint_control_mode(scenario_core.JointControlMode_position)
        assert m.joint_position_targets() == pytest.approx(m.joint_positions())

    assert pivot_dart.position() == pytest.approx(pivot_bullet.position())
    q0 = pivot_dart.position()
    f = 1.0
    T = 5.0
    trajectory = (
        np.deg2rad(90.0) * np.sin(2 * np.pi * f * t)
        for t in np.arange(start=0, step=dt, stop=T)
    )

    for _ in np.arange(start=0, step=dt, stop=T):

        # Generate the new references
        pivot_reference = q0 + next(trajectory)

        # Set the new references
        for j in (pivot_dart, pivot_bullet):
            assert j.set_position_target(position=pivot_reference)

        # Run the simulation
        assert gazebo.run()

        # Check that trajectory is being followed
        for j in (pivot_dart, pivot_bullet):
            assert j.position() == pytest.approx(pivot_reference, abs=np.deg2rad(3))
