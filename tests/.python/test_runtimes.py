# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
import numpy as np
import pytest
from gym.envs import registry
from gym.envs.registration import register
from gym_ignition.robots.sim import gazebo, pybullet
from gym_ignition.tasks.cartpole_discrete import CartPoleDiscrete
from gym_ignition.tasks.pendulum_swingup import PendulumSwingUp
from gym_ignition.utils import logger

# Set verbosity
logger.set_level(gym.logger.DEBUG)

if "Pendulum-Ignition-PyTest-v0" not in [spec.id for spec in list(registry.all())]:
    register(
        id="Pendulum-Ignition-PyTest-v0",
        entry_point="gym_ignition.runtimes.gazebo_runtime:GazeboRuntime",
        max_episode_steps=1000,
        kwargs={
            "task_cls": PendulumSwingUp,
            "robot_cls": gazebo.pendulum.PendulumGazeboRobot,
            "model": "Pendulum/Pendulum.urdf",
            "world": "DefaultEmptyWorld.world",
            "rtf": 100,
            "agent_rate": 4000,
            "physics_rate": 4000,
            "hard_reset": False,
        },
    )

if "Pendulum-PyBullet-PyTest-v0" not in [spec.id for spec in list(registry.all())]:
    register(
        id="Pendulum-PyBullet-PyTest-v0",
        entry_point="gym_ignition.runtimes.pybullet_runtime:PyBulletRuntime",
        max_episode_steps=1000,
        kwargs={
            "task_cls": PendulumSwingUp,
            "robot_cls": pybullet.pendulum.PendulumPyBulletRobot,
            "model": "Pendulum/Pendulum.urdf",
            "world": "plane_implicit.urdf",
            "rtf": 100,
            "agent_rate": 4000,
            "physics_rate": 4000,  # To keep errors small, pybullet needs a higher rate
            "hard_reset": False,
        },
    )

if "CartPoleDiscrete-Ignition-PyTest-v0" not in [
    spec.id for spec in list(registry.all())
]:
    register(
        id="CartPoleDiscrete-Ignition-PyTest-v0",
        entry_point="gym_ignition.runtimes.gazebo_runtime:GazeboRuntime",
        max_episode_steps=500,
        kwargs={
            "task_cls": CartPoleDiscrete,
            "robot_cls": gazebo.cartpole.CartPoleGazeboRobot,
            "model": "CartPole/CartPole.urdf",
            "world": "DefaultEmptyWorld.world",
            "rtf": 100,
            "agent_rate": 4000,
            "physics_rate": 4000,
            "hard_reset": False,
        },
    )

if "CartPoleDiscrete-PyBullet-PyTest-v0" not in [
    spec.id for spec in list(registry.all())
]:
    register(
        id="CartPoleDiscrete-PyBullet-PyTest-v0",
        entry_point="gym_ignition.runtimes.pybullet_runtime:PyBulletRuntime",
        max_episode_steps=500,
        kwargs={
            "task_cls": CartPoleDiscrete,
            "robot_cls": pybullet.cartpole.CartPolePyBulletRobot,
            "model": "CartPole/CartPole.urdf",
            "world": "plane_implicit.urdf",
            "rtf": 100,
            "agent_rate": 4000,
            "physics_rate": 4000,
            "hard_reset": False,
        },
    )


def template_compare_environments(env_name_a: str, env_name_b: str, max_error: float):
    # Create the pendulum
    env_a = gym.make(env_name_a)
    env_b = gym.make(env_name_b)

    assert (
        env_a.unwrapped.spec._kwargs["agent_rate"]
        == env_b.unwrapped.spec._kwargs["agent_rate"]
    )

    logger.set_level(gym.logger.DEBUG)

    # Render the environment
    # env.render('human')
    # time.sleep(5)

    # Seed the environment
    env_a.seed(42)
    env_b.seed(42)

    range_obs = env_a.observation_space.high - env_a.observation_space.low
    range_obs = np.where(np.isinf(range_obs), 1000, range_obs)

    for epoch in range(10):
        # Reset the environments
        observation_a = env_a.reset()
        observation_b = env_b.reset()

        assert np.allclose(
            observation_a, observation_b
        ), "Observations after reset don't match"

        # Initialize intermediate variables
        iteration = 0
        done_a = False

        while not done_a:
            iteration += 1

            # Sample a random action from the environment a
            action = env_a.action_space.sample()

            # Step the environments
            observation_a, _, done_a, _ = env_b.step(action)
            observation_b, _, done_b, _ = env_a.step(action)

            error = np.sum(
                np.abs(observation_a / range_obs - observation_b / range_obs)
            )

            if error > max_error:
                print("===================")
                print(f"Environment A name: {env_name_a}")
                print(f"Environment B name: {env_name_b}")
                print(f"Rollout: #{epoch}@{iteration}")
                print(f"Error: {error}")
                print(f"Max Error: {max_error}")
                print(f"Observation A: {observation_a}")
                print(f"Observation B: {observation_b}")
                print("===================")
                assert False, "The error is bigger than the threshold"

    env_a.close()
    env_b.close()


@pytest.mark.parametrize(
    "env_name_a, env_name_b, max_error",
    [
        ("Pendulum-Ignition-PyTest-v0", "Pendulum-PyBullet-PyTest-v0", 0.05),
        (
            "CartPoleDiscrete-Ignition-PyTest-v0",
            "CartPoleDiscrete-PyBullet-PyTest-v0",
            0.03,
        ),
    ],
)
def test_compare_environments(env_name_a: str, env_name_b: str, max_error: float):
    template_compare_environments(env_name_a, env_name_b, max_error)
