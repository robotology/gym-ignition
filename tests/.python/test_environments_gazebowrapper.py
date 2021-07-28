# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
import pytest
from gym_ignition.utils import logger

# Set verbosity
logger.set_level(gym.logger.DEBUG)


def template_run_environment(env_name):
    logger.info(f"Testing environment '{env_name}'")
    env = gym.make(env_name)
    assert env, f"Failed to create '{env_name}' environment"

    observation = env.observation_space.sample()
    assert observation.size > 0, "The sampled observation is empty"

    observation = env.reset()
    assert observation.size > 0, "The observation is empty"

    for _ in range(10):
        action = env.action_space.sample()
        state, reward, done, _ = env.step(action)
        assert state.size > 0, "The environment didn't return a valid state"

    env.close()


@pytest.mark.parametrize(
    "env_name",
    [
        "CartPoleDiscrete-Gazebo-v0",
        "CartPoleContinuous-Gazebo-v0",
        "Pendulum-Gazebo-v0",
    ],
)
def test_environment(env_name: str):
    template_run_environment(env_name)
