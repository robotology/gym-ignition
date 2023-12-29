# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.gym_gz

import gymnasium as gym
from gym_gz.utils.logger import set_level
from gym_gz_environments import randomizers

# Set the verbosity
set_level(gym.logger.DEBUG)


def make_env(**kwargs) -> gym.Env:

    import gymnasium as gym
    import gym_gz_environments

    return gym.make("CartPoleDiscreteBalancing-Gazebo-v0", **kwargs)


@pytest.mark.parametrize("num_physics_rollouts", [0, 2])
def test_reproducibility(num_physics_rollouts: int):

    env1 = randomizers.cartpole.CartpoleEnvRandomizer(
        env=make_env, num_physics_rollouts=num_physics_rollouts
    )

    env2 = randomizers.cartpole.CartpoleEnvRandomizer(
        env=make_env, num_physics_rollouts=num_physics_rollouts
    )

    assert env1 != env2

    env1.seed(42)
    env2.seed(42)

    for _ in range(5):

        # Reset the environments with the same seed
        observation1, _ = env1.reset(seed=42, options={})
        observation2, _ = env2.reset(seed=42, options={})
        assert observation1 == pytest.approx(observation2)

        # Initialize returned values
        done = False

        while not done:

            # Sample a random action
            action1 = env1.action_space.sample()
            action2 = env2.action_space.sample()
            assert action1 == pytest.approx(action2)

            # Step the environment
            observation1, reward1, terminated1, truncated1, info1 = env1.step(action1)
            observation2, reward2, terminated2, truncated2, info2 = env2.step(action2)

            assert truncated1 == pytest.approx(truncated2)
            assert terminated1 == pytest.approx(terminated2)
            assert info1 == pytest.approx(info2)
            assert reward1 == pytest.approx(reward2)
            assert observation1 == pytest.approx(observation2)

            # Check if the episode is done
            done1 = terminated1 or truncated1
            done2 = terminated2 or truncated2

            done = done1

    env1.close()
    env2.close()
