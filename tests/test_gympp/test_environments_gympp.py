# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
pytestmark = pytest.mark.gympp

import gym
from scenario import gazebo as scenario

try:
    import gympp_bindings as bindings
except ImportError:
    pytest.skip("gympp bindings not found", allow_module_level=True)


# Set verbosity
scenario.set_verbosity(4)


def test_create_cpp_environment():

    env = gym.make("CartPoleDiscrete-Gympp-v0")
    assert env

    env.seed(42)

    action = env.action_space.sample()
    assert isinstance(action, int)

    observation = env.observation_space.sample()
    assert observation.size > 0

    observation = env.reset()
    assert observation.size > 0

    state, reward, done, _ = env.step(action)
    assert state.size > 0
