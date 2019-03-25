#!/usr/bin/env python3
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
