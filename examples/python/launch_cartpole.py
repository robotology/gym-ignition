#!/usr/bin/env python3

# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
import gym_ignition
from gym_ignition.utils import logger

# Set gym verbosity
logger.set_level(gym.logger.INFO)
assert logger.set_level(gym.logger.DEBUG) or True

# Create the environment
# env = gym.make("CartPole-v1")
# env = gym.make("CartPoleGympp-Discrete-v0")
env = gym.make("CartPoleGymppy-Discrete-v0")
# env = gym.make("CartPoleGymppy-Continuous-v0")
# env = gym.make("CartPole-PyBullet-Discrete-v0")

# Enable the rendering
env.render('human')

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

        # Render the environment
        # It is not required to call this in the loop
        # env.render('human')

        # Accumulate the reward
        totalReward += reward

        # Print the observation
        msg = ""
        for value in observation:
            msg += "\t%.6f" % value
        logger.debug(msg)

    logger.info("Total reward for episode #{}: {}".format(epoch, totalReward))

env.close()
