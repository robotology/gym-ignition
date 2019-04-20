#!/usr/bin/env python3

# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
from gym import logger
import gym_ignition

# Set gym verbosity
logger.set_level(logger.INFO)
assert(logger.set_level(logger.DEBUG) or True)

logger.debug("Initializing the environment")

# Create the environment
# env = gym.make("CartPole-v1")
env = gym.make("CartPoleIgnition-v0")
# env = gym.make("CartPoleIgnitionPython-v0")

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
        env.render('human')

        # Accumulate the reward
        totalReward += reward

        # Print the observation
        msg = ""
        for value in observation:
            msg += "\t%.6f" % value
        logger.debug(msg)

    logger.info("Total reward for episode #{}: {}".format(epoch, totalReward))

env.close()
