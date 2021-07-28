# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.scenario

from typing import Callable, Tuple

import gym_ignition_models
import numpy as np
from gym_ignition.utils.scenario import get_joint_positions_space
from scipy.spatial.transform import Rotation

from scenario import core
from scenario import gazebo as scenario

from ..common import utils
from ..common.utils import default_world_fixture as default_world

# Set the verbosity
scenario.set_verbosity(scenario.Verbosity_debug)


def to_wxyz(xyzw: np.ndarray) -> np.ndarray:

    if xyzw.shape != (4,):
        raise ValueError(xyzw)

    return xyzw[[3, 0, 1, 2]]


def to_xyzw(wxyz: np.ndarray) -> np.ndarray:

    if wxyz.shape != (4,):
        raise ValueError(wxyz)

    return wxyz[[1, 2, 3, 0]]


def to_matrix(quaternion: Tuple[float, float, float, float]) -> np.ndarray:

    quaternion_xyzw = to_xyzw(np.array(quaternion))
    return Rotation.from_quat(quaternion_xyzw).as_matrix()


def get_random_panda(
    gazebo: scenario.GazeboSimulator, world: scenario.World
) -> core.Model:

    panda_urdf = gym_ignition_models.get_model_file("panda")
    assert world.insert_model(panda_urdf)
    assert "panda" in world.model_names()

    panda = world.get_model("panda")

    joint_space = get_joint_positions_space(model=panda)
    joint_space.seed(10)

    q = joint_space.sample()
    dq = joint_space.np_random.uniform(low=-1.0, high=1.0, size=q.shape)

    assert panda.to_gazebo().reset_joint_positions(q.tolist())
    assert panda.to_gazebo().reset_joint_velocities(dq.tolist())

    assert gazebo.run(paused=True)
    return panda


def get_cube(gazebo: scenario.GazeboSimulator, world: scenario.World) -> core.Model:

    quaternion = to_wxyz(Rotation.from_euler("x", 45, degrees=True).as_quat())
    initial_pose = core.Pose([0, 0, 0.5], quaternion.tolist())

    cube_urdf = utils.get_cube_urdf()
    assert world.insert_model(cube_urdf, initial_pose)
    assert "cube_robot" in world.model_names()

    cube = world.get_model("cube_robot")

    assert cube.to_gazebo().reset_base_world_linear_velocity([0.1, -0.2, -0.3])
    assert cube.to_gazebo().reset_base_world_angular_velocity([-0.1, 2.0, 0.3])

    assert gazebo.run(paused=True)
    return cube


# 1. VELOCITY LINEAR
@pytest.mark.parametrize(
    "default_world, get_model, link_name",
    [
        ((1.0 / 10_000, 1.0, 1), get_random_panda, "panda_link7"),
        ((1.0 / 10_000, 1.0, 1), get_cube, "cube"),
    ],
    indirect=["default_world"],
)
def test_linear_velocity(
    default_world: Tuple[scenario.GazeboSimulator, scenario.World],
    get_model: Callable[[scenario.GazeboSimulator, scenario.World], core.Model],
    link_name: str,
):

    # Get the simulator and the world
    gazebo, world = default_world
    dt = gazebo.step_size()

    # Get the model
    model = get_model(gazebo, world)

    # Get the link.
    # If the link is the base link, get the data through the base methods.
    link = model.get_link(link_name)

    if link.name() != model.base_frame():

        position = link.position
        orientation = link.orientation
        body_linear_velocity = link.body_linear_velocity
        world_linear_velocity = link.world_linear_velocity

    else:

        position = model.base_position
        orientation = model.base_orientation
        body_linear_velocity = model.base_body_linear_velocity
        world_linear_velocity = model.base_world_linear_velocity

    # 0.5 seconds of simulation
    for _ in range(int(0.5 / dt)):

        position_old = np.array(position())
        assert gazebo.run()
        position_new = np.array(position())

        world_velocity = (position_new - position_old) / dt

        # Test the world velocity (MIXED)
        assert world_velocity == pytest.approx(world_linear_velocity(), abs=1e-2)

        # Eq 18
        # Test the BODY velocity
        W_R_L = to_matrix(orientation())
        body_velocity = W_R_L.transpose() @ np.array(world_linear_velocity())
        assert body_velocity == pytest.approx(body_linear_velocity())

    gazebo.close()


# 2. VELOCITY ANGULAR
@pytest.mark.parametrize(
    "default_world, get_model, link_name",
    [
        ((1.0 / 10_000, 1.0, 1), get_random_panda, "panda_link7"),
        ((1.0 / 10_000, 1.0, 1), get_cube, "cube"),
    ],
    indirect=["default_world"],
)
def test_angular_velocity(
    default_world: Tuple[scenario.GazeboSimulator, scenario.World],
    get_model: Callable[[scenario.GazeboSimulator, scenario.World], core.Model],
    link_name: str,
):

    # Get the simulator and the world
    gazebo, world = default_world
    dt = gazebo.step_size()

    # Get the model
    model = get_model(gazebo, world)

    # Get the link.
    # If the link is the base link, get the data through the base methods.
    link = model.get_link(link_name)

    if link.name() != model.base_frame():

        orientation = link.orientation
        body_angular_velocity = link.body_angular_velocity
        world_angular_velocity = link.world_angular_velocity

    else:

        orientation = model.base_orientation
        body_angular_velocity = model.base_body_angular_velocity
        world_angular_velocity = model.base_world_angular_velocity

    skew = lambda matrix: (matrix - matrix.transpose()) / 2
    vee = lambda matrix: [matrix[2, 1], matrix[0, 2], matrix[1, 0]]

    for _ in range(int(0.5 / dt)):

        W_R_L_old = to_matrix(orientation())
        assert gazebo.run()
        W_R_L_new = to_matrix(orientation())

        dot_rotation_matrix = (W_R_L_new - W_R_L_old) / dt
        world_velocity = dot_rotation_matrix @ W_R_L_new.transpose()

        # Test the world velocity (MIXED)
        assert vee(skew(world_velocity)) == pytest.approx(
            world_angular_velocity(), abs=0.005
        )

        # Test the BODY velocity
        body_velocity = W_R_L_new.transpose() @ dot_rotation_matrix
        assert vee(skew(body_velocity)) == pytest.approx(
            body_angular_velocity(), abs=0.005
        )


# 3. ACCELERATION LINEAR
@pytest.mark.parametrize(
    "default_world, get_model, link_name",
    [
        ((1.0 / 10_000, 1.0, 1), get_random_panda, "panda_link7"),
        # ((1.0 / 10_000, 1.0, 1), get_cube, "cube"),  # TODO
    ],
    indirect=["default_world"],
)
def test_linear_acceleration(
    default_world: Tuple[scenario.GazeboSimulator, scenario.World],
    get_model: Callable[[scenario.GazeboSimulator, scenario.World], core.Model],
    link_name: str,
):

    # Get the simulator and the world
    gazebo, world = default_world
    dt = gazebo.step_size()

    # Get the model
    model = get_model(gazebo, world)

    # Get the link.
    # If the link is the base link, get the data through the base methods.
    link = model.get_link(link_name)

    if link.name() != model.base_frame():

        orientation = link.orientation
        world_linear_velocity = link.world_linear_velocity
        body_linear_acceleration = link.body_linear_acceleration
        world_linear_acceleration = link.world_linear_acceleration

    else:

        orientation = model.base_orientation
        world_linear_velocity = model.base_world_linear_velocity
        body_linear_acceleration = model.base_body_linear_acceleration
        world_linear_acceleration = model.base_word_linear_acceleration

    # 0.5 seconds of simulation
    for _ in range(int(0.5 / dt)):

        velocity_old = np.array(world_linear_velocity())
        assert gazebo.run()
        velocity_new = np.array(world_linear_velocity())

        world_acceleration = (velocity_new - velocity_old) / dt

        # By time to time there are steps where the acceleration becomes extremely high,
        # like 1000 m/s/s when the average is never exceeds 4 m/s/s.
        # We exclude those points. We should understand why this happens.
        if (np.array(world_linear_acceleration()) > 100.0).any():
            continue

        # Test the world acceleration (MIXED)
        assert world_linear_acceleration() == pytest.approx(world_acceleration, abs=0.5)

        # Test the BODY acceleration
        # Note: https://github.com/ignitionrobotics/ign-gazebo/issues/87
        W_R_L = to_matrix(orientation())
        body_acceleration = W_R_L.transpose() @ np.array(world_linear_acceleration())
        assert body_acceleration == pytest.approx(body_linear_acceleration())

    gazebo.close()


# 2. ACCELERATION ANGULAR
@pytest.mark.parametrize(
    "default_world, get_model, link_name",
    [
        ((1.0 / 10_000, 1.0, 1), get_random_panda, "panda_link7"),
        # ((1.0 / 10_000, 1.0, 1), get_cube, "cube"),  # TODO
    ],
    indirect=["default_world"],
)
def test_angular_acceleration(
    default_world: Tuple[scenario.GazeboSimulator, scenario.World],
    get_model: Callable[[scenario.GazeboSimulator, scenario.World], core.Model],
    link_name: str,
):

    # Get the simulator and the world
    gazebo, world = default_world
    dt = gazebo.step_size()

    # Get the model
    model = get_model(gazebo, world)

    # Get the link.
    # If the link is the base link, get the data through the base methods.
    link = model.get_link(link_name)

    if link.name() != model.base_frame():

        orientation = link.orientation
        world_angular_velocity = link.world_angular_velocity
        body_angular_acceleration = link.body_angular_acceleration
        world_angular_acceleration = link.world_angular_acceleration

    else:

        orientation = model.base_orientation
        world_angular_velocity = model.base_world_angular_velocity
        body_angular_acceleration = model.base_body_angular_acceleration
        world_angular_acceleration = model.base_word_angular_acceleration

    for _ in range(int(0.5 / dt)):

        world_velocity_old = np.array(world_angular_velocity())
        assert gazebo.run()
        world_velocity_new = np.array(world_angular_velocity())

        world_acceleration = (world_velocity_new - world_velocity_old) / dt

        # By time to time there are steps where the acceleration becomes extremely high,
        # like 1000 rad/s/s when the average is never exceeds 20 rad/s/s.
        # We exclude those points. We should understand why this happens.
        if (np.array(world_angular_acceleration()) > 100.0).any():
            continue

        # Test the world acceleration (MIXED)
        assert world_acceleration == pytest.approx(
            world_angular_acceleration(), abs=0.2
        )

        # Note: https://github.com/ignitionrobotics/ign-gazebo/issues/87
        W_R_L = to_matrix(orientation())
        body_acceleration = W_R_L.transpose() @ np.array(world_angular_acceleration())
        assert body_acceleration == pytest.approx(body_angular_acceleration())
