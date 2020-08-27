# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym
import numpy as np
from typing import Tuple
from gym_ignition.base import task
from scenario import core as scenario
from gym_ignition.utils.typing import Action, Observation, Reward
from gym_ignition.utils.typing import ActionSpace, ObservationSpace


class PendulumSwingUp(task.Task, abc.ABC):

    def __init__(self, agent_rate: float, **kwargs):

        # Initialize the Task base class
        task.Task.__init__(self, agent_rate=agent_rate)

        # Name of the pendulum model
        self.model_name = None

        # Limits
        self._max_speed = 10.0
        self._max_torque = 50.0

    def create_spaces(self) -> Tuple[ActionSpace, ObservationSpace]:

        action_space = gym.spaces.Box(low=-self._max_torque,
                                      high=self._max_torque,
                                      shape=(1,),
                                      dtype=np.float32)

        high = np.array([
             1.0,  # cos(theta)
             1.0,  # sin(theta)
             self._max_speed
        ])

        observation_space = gym.spaces.Box(low=-high, high=high, dtype=np.float32)

        return action_space, observation_space

    def set_action(self, action: Action) -> None:

        # Get the force value
        force = action.tolist()[0]

        # Set the force value
        model = self.world.get_model(self.model_name)
        ok_force = model.get_joint("pivot").set_generalized_force_target(force)

        if not ok_force:
            raise RuntimeError("Failed to set the force to the pendulum")

    def get_observation(self) -> Observation:

        # Get the model
        model = self.world.get_model(self.model_name)

        # Get the new joint position and velocity
        q = model.get_joint("pivot").position()
        dq = model.get_joint("pivot").velocity()

        # Create the observation
        observation = Observation(np.array([np.cos(q), np.sin(q), dq]))

        # Return the observation
        return observation

    def get_reward(self) -> Reward:

        # This environment is done only if the observation goes outside its limits.
        # Since it can happen only when velocity is too high, penalize this happening.
        cost = 100.0 if self.is_done() else 0.0

        # Get the model
        model = self.world.get_model(self.model_name)

        # Get the pendulum state
        q = model.get_joint("pivot").position()
        dq = model.get_joint("pivot").velocity()
        tau = model.get_joint("pivot").generalized_force_target()

        # Calculate the cost
        cost += (q ** 2) + 0.1 * (dq ** 2) + 0.001 * (tau ** 2)

        return Reward(-cost)

    def is_done(self) -> bool:

        # Get the observation
        observation = self.get_observation()

        # The environment is done if the observation is outside its space
        done = not self.observation_space.contains(observation)

        return done

    def reset_task(self) -> None:

        if self.model_name not in self.world.model_names():
            raise RuntimeError("The cartpole model was not inserted in the world")

        # Get the model
        model = self.world.get_model(self.model_name)

        # Control the pendulum in force mode
        pivot = model.get_joint("pivot")
        ok_control_mode = pivot.set_control_mode(scenario.JointControlMode_force)

        if not ok_control_mode:
            raise RuntimeError("Failed to change the control mode of the pendulum")

        # Sample an observation
        cos_q, sin_q, dq = self.observation_space.sample()

        # Compute the angle
        q = np.arctan2(sin_q, cos_q)

        # Convert to float
        q, dq = float(q), float(dq)

        # Reset the pendulum state
        ok_reset = pivot.to_gazebo().reset(q, dq)

        if not ok_reset:
            raise RuntimeError("Failed to reset the pendulum state")
