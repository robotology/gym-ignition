# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym
import numpy as np
from typing import Tuple
from gym_ignition.base import task
from gym_ignition.utils.typing import Action, Reward, Observation
from gym_ignition.utils.typing import ActionSpace, ObservationSpace


class CartPoleContinuousSwingup(task.Task, abc.ABC):

    def __init__(self,
                 agent_rate: float,
                 reward_cart_at_center: bool = True,
                 **kwargs):

        # Initialize the Task base class
        task.Task.__init__(self, agent_rate=agent_rate)

        # Name of the cartpole model
        self.model_name = None

        # Space for resetting the task
        self._reset_space = None

        # Private attributes
        self._reward_cart_at_center = reward_cart_at_center

        # Variables limits
        self._x_threshold = 2.4

    def create_spaces(self) -> Tuple[ActionSpace, ObservationSpace]:

        # Create the action space
        max_force = 50.0
        action_space = gym.spaces.Box(low=np.array([-max_force]),
                                      high=np.array([max_force]),
                                      dtype=np.float32)

        # Configure reset limits
        high = np.array([
            self._x_threshold,         # x
            np.finfo(np.float32).max,  # dx
            np.finfo(np.float32).max,  # q
            np.finfo(np.float32).max   # dq
        ])

        # Configure the reset space
        self._reset_space = gym.spaces.Box(low=-high,
                                           high=high,
                                           dtype=np.float32)

        # Configure the observation space
        obs_high = high.copy() * 1.2
        observation_space = gym.spaces.Box(low=-obs_high,
                                           high=obs_high,
                                           dtype=np.float32)

        return action_space, observation_space

    def set_action(self, action: Action) -> None:

        # Get the force value
        force = action.tolist()[0]

        # Set the force value
        model = self.world.getModel(self.model_name)
        ok_force = model.getJoint("linear").setGeneralizedForceTarget(force)

        if not ok_force:
            raise RuntimeError("Failed to set the force to the cart")

    def get_observation(self) -> Observation:

        # Get the model
        model = self.world.getModel(self.model_name)

        # Get the new joint positions and velocities
        q, x = model.jointPositions(["pivot", "linear"])
        dq, dx = model.jointVelocities(["pivot", "linear"])

        # Create the observation
        observation = Observation(np.array([x, dx, q, dq]))

        # Return the observation
        return observation

    def get_reward(self) -> Reward:

        # Calculate the reward
        reward = 1.0 if not self.is_done() else 0.0

        if self._reward_cart_at_center:
            # Get the observation
            x, dx, _, _ = self.get_observation()

            reward = reward \
                - 0.10 * np.abs(x) \
                - 0.10 * np.abs(dx) \
                - 10.0 * (x >= self._x_threshold)

        return reward

    def is_done(self) -> bool:

        # Get the observation
        observation = self.get_observation()

        # The environment is done if the observation is outside its space
        done = not self.observation_space.contains(observation)

        return done

    def reset_task(self) -> None:

        if self.model_name not in self.world.modelNames():
            raise RuntimeError("Cartpole model not found in the world")
