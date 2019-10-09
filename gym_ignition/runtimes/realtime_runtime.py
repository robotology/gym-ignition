# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
from gym_ignition.base import runtime, task
from gym_ignition.utils.typing import State, Action, Observation


class RealTimeRuntime(runtime.Runtime):
    def __init__(self,
                 task_cls: type,
                 robot_cls: type,
                 agent_rate: float,
                 **kwargs):
        # Build the environment
        task_object = task_cls(**kwargs)

        assert isinstance(task_object, task.Task), \
            "'task_cls' object must inherit from Task"

        super().__init__(task=task_object, agent_rate=agent_rate)

        # TODO: This class is only a draft
        raise NotImplementedError

    # ==========
    # PROPERTIES
    # ==========

    @property
    def agent_rate(self) -> float:
        raise NotImplementedError

    @agent_rate.setter
    def agent_rate(self, agent_rate: float) -> None:
        raise NotImplementedError

    # ===============
    # gym.Env METHODS
    # ===============

    def step(self, action: Action) -> State:
        # Validate action and robot
        assert self.action_space.contains(action), \
            "%r (%s) invalid" % (action, type(action))

        # Set the action
        ok_action = self.task.set_action(action)
        assert ok_action, "Failed to set the action"

        # TODO: realtime step

        # Get the observation
        observation = self.task.get_observation()
        assert self.observation_space.contains(observation), \
            "%r (%s) invalid" % (observation, type(observation))

        # Get the reward
        # TODO: use the wrapper method?
        reward = self.task.get_reward()
        assert reward, "Failed to get the reward"

        # Check termination
        done = self.task.is_done()

        return State((observation, reward, done, {}))

    def reset(self) -> Observation:
        # TODO realtime reset

        # Get the observation
        observation = self.task.get_observation()

        return Observation(observation)

    def render(self, mode: str = 'human', **kwargs) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError
