#!/usr/bin/env python3

# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
import pytest
import gym_ignition
from gym import logger

# Set gym verbosity
logger.set_level(logger.WARN)


def test_create_cpp_environment():
    env = gym.make("CartPoleIgnition-v0")
    assert env, "Failed to create CartPoleIgnition-v0 environment"

    action = env.action_space.sample()
    assert isinstance(action, int), "The sampled action is empty"

    observation = env.observation_space.sample()
    assert observation.size > 0, "The sampled observation is empty"

    observation = env.reset()
    assert observation.size > 0, "The observation is empty"

    state, reward, done, _ = env.step(action)
    assert state.size > 0, "The environment didn't return a valid state"


def test_create_python_environment():
    env = gym.make("CartPoleIgnitionPython-v0")
    assert env, "Failed to create CartPoleIgnitionPython-v0 environment"

    action = env.action_space.sample()
    assert isinstance(action, int), "The sampled action is empty"

    observation = env.observation_space.sample()
    assert observation.size > 0, "The sampled observation is empty"

    observation = env.reset()
    assert observation.size > 0, "The observation is empty"

    state, reward, done, _ = env.step(action)
    assert state.size > 0, "The environment didn't return a valid state"
