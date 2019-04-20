# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym_ignition import IgnitionPythonEnv
from gym_ignition.utils.typing import *

import gym
from gym.utils import seeding
from gym import logger
import numpy as np

class CartPolePythonEnv(IgnitionPythonEnv):
    def __init__(self):
        # Initialize base class
        super().__init__()

        # Private attributes
        self._np_random = None
        self._force_mag = 10
        self._steps_beyond_done = None

        # Configure action space
        self.action_space = gym.spaces.Discrete(2)

        self._x_threshold = 2.4
        self._theta_threshold_radians = 12 * 2 * np.pi / 360

        # Configure observation limits
        high = np.array([
            self._x_threshold * 2,
            np.finfo(np.float32).max,
            self._theta_threshold_radians * 2,
            np.finfo(np.float32).max
        ])

        # Configure the observation space
        self.observation_space = gym.spaces.Box(-high, high, dtype=np.float32)

        # Seed the environment
        self.seed()

    def step(self, action: Action) -> State:
        assert self.action_space.contains(action), \
            "%r (%s) invalid" % (action, type(action))

        # Get the robot object
        robot = self.robot

        # Read the action and send the force to the cart
        force = self._force_mag if action == 1 else -self._force_mag
        ok = robot.setJointForce("linear", force)

        if not ok:
            raise Exception("Failed to set the force to the cart")

        # Step the simulator
        ok = self.gazebo.run()

        if not ok:
            raise Exception("Failed to step gazebo")

        # Get the new joint positions
        x = robot.jointPosition("linear")
        theta = robot.jointPosition("pivot")

        # Get the new joint velocities
        x_dot = robot.jointVelocity("linear")
        theta_dot = robot.jointVelocity("pivot")

        # Calculate if the environment reached its termination
        done = np.abs(x) > self._x_threshold or \
               np.abs(theta) > self._theta_threshold_radians

        # Initialize the reward
        reward = None

        # Calculate the reward
        if not done:
            reward = 1.0
        else:
            if self._steps_beyond_done is None:
                # Pole just fell
                self._steps_beyond_done = 0
                reward = 1.0
            else:
                self._steps_beyond_done += 1
                reward = 0.0

                # Warn the user to call reset
                if self._steps_beyond_done == 1:
                    logger.warn("You are calling 'step()' even though this environment "
                                "has already returned done = True. You should always "
                                "call 'reset()' once you receive 'done = True' -- any "
                                "further steps are undefined behavior.")

        done = bool(done)
        reward = Reward(reward)
        observation = Observation(np.array([x, x_dot, theta, theta_dot]))

        return State((observation, reward, done, {}))

    def reset(self) -> Observation:
        # Initialize the environment with a new random state
        new_state = self._np_random.uniform(low=-0.05, high=0.05, size=(4,))
        ok1 = self.robot.resetJoint("linear", new_state[0], new_state[1])
        ok2 = self.robot.resetJoint("pivot", new_state[2], new_state[3])

        if not (ok1 and ok2):
            raise Exception("Failed to reset robot state")

        # Reset the flag that assures reset is properly called
        self._steps_beyond_done = None

        return Observation(np.array(new_state))

    def seed(self, seed: int = None) -> SeedList:
        # Generate the rng
        self._np_random, seed = seeding.np_random(seed)

        # Spaces need to be seeded. Apply the same logic contained in gym.seeding.
        short_seed = seeding._int_list_from_bigint(seeding.hash_seed(seed))

        # Seed the spaces
        self.action_space.seed(short_seed)
        self.observation_space.seed(short_seed)

        return SeedList([seed])

    def _get_model_sdf(self) -> str:
        return "CartPole/CartPole.sdf"

    def _get_world_sdf(self) -> str:
        return "CartPole.world"
