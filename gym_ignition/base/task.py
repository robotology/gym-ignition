# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym
import numpy as np
from typing import Tuple
from gym_ignition.base.robot import robot_abc, RobotFeatures
from gym_ignition.utils.typing import Action, Observation, Reward, SeedList


class Task(gym.Env, abc.ABC):

    def __init__(self, robot: RobotFeatures = None) -> None:
        self._robot = robot
        self._np_random = None

        # The parent class gym.Env has the following defined as class variables.
        # Since they are hidden from the users, and child classes of Task need to fill
        # their values, here we define them again as object variables.
        #
        # Note that object variables shadow class variables when accessed from an object
        # instance. This is not a problem because accessing spaces from the class
        # object is a not a common practice.
        self.action_space = None
        self.observation_space = None

    # ==========
    # PROPERTIES
    # ==========

    @property
    def robot(self) -> RobotFeatures:
        if self._robot:
            assert self._robot.valid(), "The robot interface is not valid"
            return self._robot

        assert False, "The robot interface object was never stored"

    @robot.setter
    def robot(self, robot: robot_abc.RobotABC) -> None:
        if not robot.valid(): raise Exception("Robot object is not valid")
        self._robot = robot

    # ==============
    # TASK INTERFACE
    # ==============

    @abc.abstractmethod
    def _reset(self) -> bool:
        pass

    @abc.abstractmethod
    def _set_action(self, action: Action) -> bool:
        pass

    @abc.abstractmethod
    def _get_observation(self) -> Observation:
        pass

    @abc.abstractmethod
    def _get_reward(self) -> Reward:
        pass

    @abc.abstractmethod
    def _is_done(self) -> bool:
        pass

    # =================
    # gym.Env INTERFACE
    # =================

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
