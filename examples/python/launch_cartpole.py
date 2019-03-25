#!/usr/bin/env python3

# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
import gym_ignition

# Create the environment
env = gym.make("CartPoleIgnition-v0")

# Initialize the seed
env.seed(42)

# Render the environment
env.render('human')

for _ in range(10):
    # Reset the environment
    observation = env.reset()

    # Initialize returned values
    done = False
    totalReward = 0

    while not done:
        # Execute a random action
        action = env.action_space.sample()
        observation, reward, done, _ = env.step(action)

        totalReward += reward
        print(observation)

    print(totalReward)
