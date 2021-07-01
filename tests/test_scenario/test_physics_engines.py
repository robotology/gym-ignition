# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
pytestmark = pytest.mark.scenario

import numpy as np
import gym_ignition_models
from ..common import utils
from gym_ignition.utils import misc
from scenario import core as scenario_core
from scenario import gazebo as scenario_gazebo
from ..common.utils import gazebo_fixture as gazebo

# Set the verbosity
scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_debug)


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_pendulum(gazebo: scenario_gazebo.GazeboSimulator):

    # Load two different worlds
    empty_world_sdf = utils.get_empty_world_sdf()
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "dart")
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "bullet")

    # Initialize the simulator
    assert gazebo.initialize()

    pendulum_urdf = gym_ignition_models.get_model_file(robot_name="pendulum")
    ground_plane_urdf = gym_ignition_models.get_model_file(robot_name="ground_plane")

    # Populate the DART world
    world_dart = gazebo.get_world("dart")
    world_dart.set_physics_engine(scenario_gazebo.PhysicsEngine_dart)
    world_dart.insert_model(ground_plane_urdf)
    world_dart.insert_model(pendulum_urdf)
    pendulum_dart = world_dart.get_model(model_name="pendulum")

    # Populate the Bullet world
    world_bullet = gazebo.get_world("bullet")
    world_bullet.set_physics_engine(scenario_gazebo.PhysicsEngine_bullet)
    world_bullet.insert_model(ground_plane_urdf)
    world_bullet.insert_model(pendulum_urdf)
    pendulum_bullet = world_bullet.get_model(model_name="pendulum")

    # Reset the pole position
    # Note: this is not yet implemented in the bullet plugin
    # assert pendulum_dart.get_joint(joint_name="pivot").to_gazebo().\
    #     reset_position(position=0.01)
    # assert pendulum_bullet.get_joint(joint_name="pivot").to_gazebo().\
    #     reset_position(position=0.01)

    # Control the joints in force
    assert pendulum_dart.set_joint_control_mode(scenario_core.JointControlMode_force)
    assert pendulum_bullet.set_joint_control_mode(scenario_core.JointControlMode_force)

    # Give a small push to the pole
    assert pendulum_dart.get_joint(joint_name="pivot").\
        set_generalized_force_target(force=50.0)
    assert pendulum_bullet.get_joint(joint_name="pivot").\
        set_generalized_force_target(force=50.0)
    gazebo.run()
    assert pendulum_dart.get_joint(joint_name="pivot").\
        set_generalized_force_target(force=0.0)
    assert pendulum_bullet.get_joint(joint_name="pivot").\
        set_generalized_force_target(force=0.0)

    # Integration time
    dt = gazebo.step_size() * gazebo.steps_per_run()

    for _ in np.arange(start=0.0, stop=1.0, step=dt):

        # Run the simulator
        assert gazebo.run()

        # Check that both worlds evolve similarly
        assert pendulum_dart.joint_positions() == \
               pytest.approx(pendulum_bullet.joint_positions(), abs=1e-03)
        assert pendulum_dart.joint_velocities() == \
               pytest.approx(pendulum_bullet.joint_velocities(), abs=1e-02)


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_bouncing_ball(gazebo: scenario_gazebo.GazeboSimulator):

    # Load two different worlds
    empty_world_sdf = utils.get_empty_world_sdf()
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "dart")
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "bullet")

    # Initialize the simulator
    assert gazebo.initialize()

    # gazebo.gui()
    # import time
    # time.sleep(1)
    # gazebo.run(paused=True)

    sphere_urdf_string = utils.SphereURDF(restitution=0.8).urdf()
    sphere_urdf = misc.string_to_file(sphere_urdf_string)
    ground_plane_urdf = gym_ignition_models.get_model_file(robot_name="ground_plane")

    # Pose of the sphere
    sphere_pose = scenario_core.Pose([0, 0, 0.5], [1, 0, 0, 0])

    # Populate the DART world
    world_dart = gazebo.get_world("dart")
    world_dart.set_physics_engine(scenario_gazebo.PhysicsEngine_dart)
    world_dart.insert_model(ground_plane_urdf)
    world_dart.insert_model(sphere_urdf, sphere_pose)
    sphere_dart = world_dart.get_model(model_name="sphere_model")

    # Populate the Bullet world
    world_bullet = gazebo.get_world("bullet")
    world_bullet.set_physics_engine(scenario_gazebo.PhysicsEngine_bullet)
    world_bullet.insert_model(ground_plane_urdf)
    world_bullet.insert_model(sphere_urdf, sphere_pose)
    sphere_bullet = world_bullet.get_model(model_name="sphere_model")

    # Integration time
    dt = gazebo.step_size() * gazebo.steps_per_run()

    # Enable contacts
    sphere_dart.enable_contacts(True)
    # sphere_bullet.enable_contacts(True)

    for _ in np.arange(start=0.0, stop=3.0, step=dt):

        # Run the simulator
        assert gazebo.run()

        # Bullet does not have bouncing yet
        if len(sphere_dart.contacts()) > 0:
            break

        # print(sphere_dart.base_position(),
        #       sphere_bullet.base_position())
        # print(sphere_dart.base_world_linear_velocity(),
        #       sphere_bullet.base_world_linear_velocity())

        # Check that both worlds evolve similarly
        assert sphere_dart.base_position() == \
               pytest.approx(sphere_bullet.base_position(), abs=0.005)
        assert sphere_dart.base_world_linear_velocity() == \
               pytest.approx(sphere_bullet.base_world_linear_velocity(), abs=0.001)
        assert sphere_dart.base_world_angular_velocity() == \
               pytest.approx(sphere_bullet.base_world_angular_velocity())

    # time.sleep(3)


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def _test_position_controller(gazebo: scenario_gazebo.GazeboSimulator):

    raise NotImplementedError
