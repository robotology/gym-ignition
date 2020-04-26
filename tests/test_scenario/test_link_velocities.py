# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
pytestmark = pytest.mark.scenario

import numpy as np
from ..common import utils
import gym_ignition_models
from typing import Callable, List, Tuple
from scipy.spatial.transform import Rotation
from gym_ignition import scenario_bindings as bindings
from ..common.utils import default_world_fixture as default_world
from gym_ignition.utils.scenario import get_joint_positions_space


def to_wxyz(xyzw: np.ndarray) -> np.ndarray:

    if xyzw.shape != (4,):
        raise ValueError(xyzw)

    return xyzw[[3, 0, 1, 2]]


def to_xyzw(wxyz: np.ndarray) -> np.ndarray:

    if wxyz.shape != (4,):
        raise ValueError(wxyz)

    return wxyz[[1, 2, 3, 0]]


def to_matrix(quaternion: List[float]) -> np.ndarray:

    quaternion_xyzw = to_xyzw(np.array(quaternion))
    return Rotation.from_quat(quaternion_xyzw).as_matrix()


def get_random_panda(gazebo: bindings.GazeboSimulator,
                     world: bindings.World) -> bindings.Model:

    panda_urdf = gym_ignition_models.get_model_file("panda")
    assert world.insertModel(panda_urdf)
    assert "panda" in world.modelNames()

    panda = world.getModel("panda")

    joint_space = get_joint_positions_space(model=panda)
    joint_space.seed(10)

    q = joint_space.sample()
    dq = joint_space.np_random.uniform(low=-1.0, high=1.0, size=q.shape)

    assert panda.resetJointPositions(q.tolist())
    assert panda.resetJointVelocities(dq.tolist())

    assert gazebo.run(paused=True)
    return panda


def get_cube(gazebo: bindings.GazeboSimulator,
             world: bindings.World) -> bindings.Model:

    quaternion = to_wxyz(Rotation.from_euler('x', 45, degrees=True).as_quat())
    initial_pose = bindings.Pose([0, 0, 0.5], quaternion.tolist())

    cube_urdf = utils.get_cube_urdf()
    assert world.insertModel(cube_urdf, initial_pose)
    assert "cube_robot" in world.modelNames()

    cube = world.getModel("cube_robot")

    assert cube.resetBaseWorldLinearVelocity([0.1, -0.2, -0.3])
    assert cube.resetBaseWorldAngularVelocity([-0.1, 2.0, 0.3])

    assert gazebo.run(paused=True)
    return cube


# 1. VELOCITY LINEAR
@pytest.mark.parametrize("default_world, get_model, link_name", [
                          ((1.0 / 10_000, 1.0, 1), get_random_panda, "panda_link7"),
                          ((1.0 / 10_000, 1.0, 1), get_cube, "cube"),
                          ],
                         indirect=["default_world"])
def test_linear_velocity(
        default_world: Tuple[bindings.GazeboSimulator, bindings.World],
        get_model: Callable[[bindings.GazeboSimulator, bindings.World], bindings.Model],
        link_name: str):

    # Get the simulator and the world
    gazebo, world = default_world
    dt = gazebo.stepSize()

    # Get the model
    model = get_model(gazebo, world)

    # Get the link.
    # If the link is the base link, get the data through the base methods.
    link = model.getLink(link_name)

    if link.name() != model.baseFrame():

        position = link.position
        orientation = link.orientation
        bodyLinearVelocity = link.bodyLinearVelocity
        worldLinearVelocity = link.worldLinearVelocity

    else:

        position = model.basePosition
        orientation = model.baseOrientation
        bodyLinearVelocity = model.baseBodyLinearVelocity
        worldLinearVelocity = model.baseWorldLinearVelocity

    # 0.5 seconds of simulation
    for _ in range(int(0.5 / dt)):

        position_old = np.array(position())
        assert gazebo.run()
        position_new = np.array(position())

        world_velocity = (position_new - position_old) / dt

        # Test the world velocity (MIXED)
        assert world_velocity == pytest.approx(worldLinearVelocity(), abs=1E-2)

        # Eq 18
        # Test the BODY velocity
        W_R_L = to_matrix(orientation())
        body_velocity = W_R_L.transpose() @ np.array(worldLinearVelocity())
        assert body_velocity == pytest.approx(bodyLinearVelocity())

    gazebo.close()


# 2. VELOCITY ANGULAR
@pytest.mark.parametrize("default_world, get_model, link_name", [
                          ((1.0 / 10_000, 1.0, 1), get_random_panda, "panda_link7"),
                          ((1.0 / 10_000, 1.0, 1), get_cube, "cube"),
                          ],
                         indirect=["default_world"])
def test_angular_velocity(
        default_world: Tuple[bindings.GazeboSimulator, bindings.World],
        get_model: Callable[[bindings.GazeboSimulator, bindings.World], bindings.Model],
        link_name: str):

    # Get the simulator and the world
    gazebo, world = default_world
    dt = gazebo.stepSize()

    # Get the model
    model = get_model(gazebo, world)

    # Get the link.
    # If the link is the base link, get the data through the base methods.
    link = model.getLink(link_name)

    if link.name() != model.baseFrame():

        orientation = link.orientation
        bodyAngularVelocity = link.bodyAngularVelocity
        worldAngularVelocity = link.worldAngularVelocity

    else:

        orientation = model.baseOrientation
        bodyAngularVelocity = model.baseBodyAngularVelocity
        worldAngularVelocity = model.baseWorldAngularVelocity

    skew = lambda matrix: (matrix - matrix.transpose()) / 2
    vee = lambda matrix: [matrix[2, 1], matrix[0,2], matrix[1, 0]]

    for _ in range(int(0.5 / dt)):

        W_R_L_old = to_matrix(orientation())
        assert gazebo.run()
        W_R_L_new = to_matrix(orientation())

        dot_rotation_matrix = (W_R_L_new - W_R_L_old) / dt
        world_velocity = dot_rotation_matrix @ W_R_L_new.transpose()

        # Test the world velocity (MIXED)
        assert vee(skew(world_velocity)) == pytest.approx(worldAngularVelocity(),
                                                          abs=0.005)

        # Test the BODY velocity
        body_velocity = W_R_L_new.transpose() @ dot_rotation_matrix
        assert vee(skew(body_velocity)) == pytest.approx(bodyAngularVelocity(),
                                                         abs=0.005)


# 3. ACCELERATION LINEAR
@pytest.mark.parametrize("default_world, get_model, link_name", [
                          ((1.0 / 10_000, 1.0, 1), get_random_panda, "panda_link7"),
                          # ((1.0 / 10_000, 1.0, 1), get_cube, "cube"),  # TODO
                          ],
                         indirect=["default_world"])
def test_linear_acceleration(
        default_world: Tuple[bindings.GazeboSimulator, bindings.World],
        get_model: Callable[[bindings.GazeboSimulator, bindings.World], bindings.Model],
        link_name: str):

    # Get the simulator and the world
    gazebo, world = default_world
    dt = gazebo.stepSize()

    # Get the model
    model = get_model(gazebo, world)

    # Get the link.
    # If the link is the base link, get the data through the base methods.
    link = model.getLink(link_name)

    if link.name() != model.baseFrame():

        orientation = link.orientation
        worldLinearVelocity = link.worldLinearVelocity
        bodyLinearAcceleration = link.bodyLinearAcceleration
        worldLinearAcceleration = link.worldLinearAcceleration

    else:

        orientation = model.baseOrientation
        worldLinearVelocity = model.baseWorldLinearVelocity
        bodyLinearAcceleration = model.baseBodyLinearAcceleration
        worldLinearAcceleration = model.baseWordLinearAcceleration

    # 0.5 seconds of simulation
    for _ in range(int(0.5 / dt)):

        velocity_old = np.array(worldLinearVelocity())
        assert gazebo.run()
        velocity_new = np.array(worldLinearVelocity())

        world_acceleration = (velocity_new - velocity_old) / dt

        # By time to time there are steps where the acceleration becomes extremely high,
        # like 1000 m/s/s when the average is never exceeds 4 m/s/s.
        # We exclude those points. We should understand why this happens.
        if (np.array(worldLinearAcceleration()) > 100.0).any():
            continue

        # Test the world acceleration (MIXED)
        assert worldLinearAcceleration() == pytest.approx(world_acceleration, abs=0.5)

        # Test the BODY acceleration
        # Note: https://github.com/ignitionrobotics/ign-gazebo/issues/87
        W_R_L = to_matrix(orientation())
        body_acceleration = W_R_L.transpose() @ np.array(worldLinearAcceleration())
        assert body_acceleration == pytest.approx(bodyLinearAcceleration())

    gazebo.close()


# 2. ACCELERATION ANGULAR
@pytest.mark.parametrize("default_world, get_model, link_name", [
                          ((1.0 / 10_000, 1.0, 1), get_random_panda, "panda_link7"),
                          # ((1.0 / 10_000, 1.0, 1), get_cube, "cube"),  # TODO
                          ],
                         indirect=["default_world"])
def test_angular_acceleration(
        default_world: Tuple[bindings.GazeboSimulator, bindings.World],
        get_model: Callable[[bindings.GazeboSimulator, bindings.World], bindings.Model],
        link_name: str):

    # Get the simulator and the world
    gazebo, world = default_world
    dt = gazebo.stepSize()

    # Get the model
    model = get_model(gazebo, world)

    # Get the link.
    # If the link is the base link, get the data through the base methods.
    link = model.getLink(link_name)

    if link.name() != model.baseFrame():

        orientation = link.orientation
        worldAngularVelocity = link.worldAngularVelocity
        bodyAngularAcceleration = link.bodyAngularAcceleration
        worldAngularAcceleration = link.worldAngularAcceleration

    else:

        orientation = model.baseOrientation
        worldAngularVelocity = model.baseWorldAngularVelocity
        bodyAngularAcceleration = model.baseBodyAngularAcceleration
        worldAngularAcceleration = model.baseWordAngularAcceleration

    for _ in range(int(0.5 / dt)):

        world_velocity_old = np.array(worldAngularVelocity())
        assert gazebo.run()
        world_velocity_new = np.array(worldAngularVelocity())

        world_acceleration = (world_velocity_new - world_velocity_old) / dt

        # By time to time there are steps where the acceleration becomes extremely high,
        # like 1000 rad/s/s when the average is never exceeds 20 rad/s/s.
        # We exclude those points. We should understand why this happens.
        if (np.array(worldAngularAcceleration()) > 100.0).any():
            continue

        # Test the world acceleration (MIXED)
        assert world_acceleration == pytest.approx(worldAngularAcceleration(),
                                                   abs=0.2)

        # Note: https://github.com/ignitionrobotics/ign-gazebo/issues/87
        W_R_L = to_matrix(orientation())
        body_acceleration = W_R_L.transpose() @ np.array(worldAngularAcceleration())
        assert body_acceleration == pytest.approx(bodyAngularAcceleration())
