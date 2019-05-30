# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
from abc import ABC, abstractmethod
from gym_ignition.utils.typing import *
from gym_ignition.base.robot import Robot


class Task(ABC, gym.Env):
    def __init__(self, robot: Robot = None) -> None:
        self._robot = robot
        self._np_random = None

    @property
    def robot(self) -> Robot:
        if self._robot:
            assert self._robot.valid(), "The robot interface is not valid"
            return self._robot

        assert False, "The robot interface object was never stored"

    @robot.setter
    def robot(self, robot: Robot) -> None:
        assert robot.valid(), "Robot object is not valid"
        self._robot = robot

    @abstractmethod
    def _set_action(self, action: Action) -> bool:
        pass

    @abstractmethod
    def _get_observation(self) -> Observation:
        pass

    @abstractmethod
    def _get_reward(self) -> float:
        pass

    @abstractmethod
    def _is_done(self) -> bool:
        pass

    @abstractmethod
    def _reset(self) -> bool:
        pass

    def seed(self, seed: int = None) -> SeedList:
        if not seed:
            seed = np.random.randint(2**32 - 1)

        # Seed numpy
        self._np_random = np.random
        self._np_random.seed(seed)

        # Seed the spaces
        self.action_space.seed(seed)
        self.observation_space.seed(seed)

        return SeedList([seed])
