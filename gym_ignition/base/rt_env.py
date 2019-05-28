# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
# from gym_ignition.utils import logger
from gym_ignition.utils.typing import *


class RTEnv(gym.Wrapper):
    def __init__(self,
                 task: type,
                 robot : type,
                 agent_rate: float,
                 **kwargs):
        super().__init__(task)
        raise NotImplementedError

    @property
    def unwrapped(self):
        return self

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
        ok_action = self.env._set_action(action)
        assert ok_action, "Failed to set the action"

        # TODO: Real Time step
        raise NotImplementedError

        # Get the observation
        observation = self.env._get_observation()
        assert self.observation_space.contains(observation), \
            "%r (%s) invalid" % (observation, type(observation))

        # Get the reward
        # TODO: use the wrapper method?
        reward = self.env._get_reward()
        assert reward, "Failed to get the reward"

        # Check termination
        done = self.env._is_done()

        return State((observation, reward, done, {}))

    def reset(self) -> Observation:
        # TODO Real Time reset
        raise NotImplementedError

        # Get the observation
        observation = self.env._get_observation()
        assert self.observation_space.contains(observation), \
            "%r (%s) invalid" % (observation, type(observation))

        return Observation(observation)

    def render(self, mode: str = 'human', **kwargs) -> None:
        pass

    def close(self) -> None:
        pass

    def seed(self, seed: int = None) -> SeedList:
        # Seed the wrapped environment (task)
        seed = self.env.seed(seed)

        # Update the spaces of the wrapper
        self.action_space = self.env.action_space
        self.observation_space = self.env.observation_space

        return seed
