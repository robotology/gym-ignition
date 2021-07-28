# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import functools
import time

import gym
from gym_ignition.utils import logger
from gym_ignition_environments import randomizers

# Set verbosity
logger.set_level(gym.logger.ERROR)
# logger.set_level(gym.logger.DEBUG)

# Available tasks
env_id = "CartPoleDiscreteBalancing-Gazebo-v0"
# env_id = "CartPoleContinuousBalancing-Gazebo-v0"
# env_id = "CartPoleContinuousSwingup-Gazebo-v0"


def make_env_from_id(env_id: str, **kwargs) -> gym.Env:
    import gym
    import gym_ignition_environments

    return gym.make(env_id, **kwargs)


# Create a partial function passing the environment id
make_env = functools.partial(make_env_from_id, env_id=env_id)

# Wrap the environment with the randomizer.
# This is a simple example no randomization are applied.
env = randomizers.cartpole_no_rand.CartpoleEnvNoRandomizations(env=make_env)

# Wrap the environment with the randomizer.
# This is a complex example that randomizes both the physics and the model.
# env = randomizers.cartpole.CartpoleEnvRandomizer(
#     env=make_env, seed=42, num_physics_rollouts=5)

# Enable the rendering
# env.render('human')

# Initialize the seed
env.seed(42)

for epoch in range(10):

    # Reset the environment
    observation = env.reset()

    # Initialize returned values
    done = False
    totalReward = 0

    while not done:

        # Execute a random action
        action = env.action_space.sample()
        observation, reward, done, _ = env.step(action)

        # Render the environment.
        # It is not required to call this in the loop if physics is not randomized.
        # env.render('human')

        # Accumulate the reward
        totalReward += reward

        # Print the observation
        msg = ""
        for value in observation:
            msg += "\t%.6f" % value
        logger.debug(msg)

    print(f"Reward episode #{epoch}: {totalReward}")

env.close()
time.sleep(5)
