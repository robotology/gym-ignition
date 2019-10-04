# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym
import numpy as np
from typing import Tuple
from gym.utils import seeding
from gym_ignition.base.robot import robot_abc, RobotFeatures
from gym_ignition.utils.typing import ActionSpace, ObservationSpace
from gym_ignition.utils.typing import Action, Observation, Reward, SeedList


class Task(gym.Env, abc.ABC):
    """
    Interface to define a decision-making task.

    The Task is the central interface of each environment.
    It defines the logic of the environment in a format that is agnostic of both the
    runtime (either simulated or real-time) and the robot.

    Beyond containing the logic of the task, objects that inherit from this interface
    expose an empty gym.Env interface. This is required because the real environment
    exposed to the agent consists of a Task wrapped by a runtime wrapper, that inherits
    from gym.Wrapper. Runtime wrappers implement the gym.Env interface, and particularly
    gym.Env.step. This method, depending on the runtime, can interface with a physics
    engine and step the simulator, or can handle real-time execution.

    In order to make Task objects generic from the runtime, also the interfacing with the
    robot needs to be abstracted. In fact, the access of robot data depends on the
    selected runtime. For example, in simulation data can be directly gathered from the
    physics engine, and in a real-time setting can be asked through a robotic middleware.
    Tasks abstract this interfacing by operating on a Robot interface, which is then
    specialized for the different runtimes.
    """

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
    def _create_spaces(self) -> Tuple[ActionSpace, ObservationSpace]:
        """
        Create the action and observations spaces.

        Returns:
            A tuple containing the action and observation spaces.
        """

    @abc.abstractmethod
    def _reset(self) -> bool:
        """
        Reset the task.

        This method contains the logic of resetting the environment.
        It is called in the gym.Env.reset method.

        Returns:
            True if successful, False otherwise.
        """

    @abc.abstractmethod
    def _set_action(self, action: Action) -> bool:
        """
        Set the task action.

        This method contains the logic of setting the environment action.
        It is called in the beginning of the gym.Env.step method.

        Args:
            action: The action to set.

        Returns:
            True if successful, False otherwise.
        """

    @abc.abstractmethod
    def _get_observation(self) -> Observation:
        """
        Return the task observation.

        This method contains the logic of constructing the environment observation.
        It is called in the end of both gym.Env.reset and gym.Env.step methods.

        Returns:
            The task observation.
        """

    @abc.abstractmethod
    def _get_reward(self) -> Reward:
        """
        Return the task reward.

        This method contains the logic of computing the environment reward.
        It is called in the end of the gym.Env.step method.

        Returns:
            The scalar reward.
        """

    @abc.abstractmethod
    def _is_done(self) -> bool:
        """
        Returns the task termination flag.

        This method contains the logic of computing when the environment is terminated.
        Subsequent actions should be preceded by an environment reset.
        It is called in the end of the gym.Env.step method.

        Returns:
            True if the environment terminated, False otherwise.
        """

    # =================
    # gym.Env INTERFACE
    # =================

    def step(self, action):
        raise NotImplementedError

    def reset(self,):
        raise NotImplementedError

    def render(self, mode='human'):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError

    def seed(self, seed: int = None) -> SeedList:
        if not seed:
            seed = np.random.randint(2**32 - 1)

        # Get an instance of the random number generator from gym utils.
        # This is necessary to have an independent rng for each environment.
        self._np_random, new_seed = seeding.np_random(seed)

        # Seed the spaces
        self.action_space.seed(new_seed)
        self.observation_space.seed(new_seed)

        return SeedList([new_seed])
