# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym
import numpy as np
from scenario import core
from gym.utils import seeding
from typing import Dict, Tuple
from gym_ignition.utils.typing import ActionSpace, ObservationSpace
from gym_ignition.utils.typing import Action, Observation, Reward, SeedList


class Task(abc.ABC):
    """
    Interface to define a decision-making task.

    The Task is the central interface of each environment implementation.
    It defines the logic of the environment in a format that is agnostic of both the
    runtime (either simulated or real-time) and the models it operates on.

    :py:class:`~gym_ignition.base.runtime.Runtime` instances are the real objects returned
    to the users when they call :py:class:`gym.make`. Depending on the type of the
    runtime, it could contain one or more :py:class:`Task` objects.
    The :py:class:`~gym_ignition.base.runtime.Runtime` is a relay class that calls the
    logic of the :py:class:`Task` from its interface methods and implements the real
    :py:meth:`gym.Env.step`.
    Simulated runtimes step the physics engine, instead, real-time
    runtimes, enforce real-time execution.

    A :py:class:`Task` object is meant to be:

    - Independent from the selected :py:class:`~gym_ignition.base.runtime.Runtime`.
      In fact, it defines only the decision making logic;
    - Independent from the :py:class:`~scenario.core.Model` objects it operates on.
      This is achieved thanks to the model abstraction provided by
      :cpp:class:`scenario::core::Model`.

    The population of the world where the task operates is demanded to a
    :py:class:`gym.Wrapper` object, that acts as an environment randomizer.
    """

    action_space: gym.spaces.Space = None
    observation_space: gym.spaces.Space = None

    def __init__(self, agent_rate: float) -> None:

        # World object
        self._world = None

        #: Rate of the agent.
        #: It matches the rate at which the :py:class:`Gym.Env` methods are called.
        self.agent_rate = agent_rate

        #: RNG available to the object to ensure reproducibility.
        #: Use it for all the random resources.
        self.np_random: np.random.RandomState

        #: The seed of the task
        self.seed: int

        # Initialize the RNG and the seed
        self.np_random, self.seed = seeding.np_random()

    # ==========
    # PROPERTIES
    # ==========

    @property
    def world(self) -> core.World:
        """
        Get the world where the task is operating.

        Returns:
            The world object.
        """

        if self._world is not None:
            return self._world

        raise Exception("The world was never stored")

    @world.setter
    def world(self, world: core.World) -> None:

        if world is None or world.name == "":
            raise ValueError("World not valid")

        # Store the world
        self._world = world

    def has_world(self) -> bool:
        """
        Check if the world was stored.

        Returns:
            True if the task has a valid world, False otherwise.
        """

        return self._world is not None and self._world.name != ""

    # ==============
    # Task Interface
    # ==============

    @abc.abstractmethod
    def create_spaces(self) -> Tuple[ActionSpace, ObservationSpace]:
        """
        Create the action and observations spaces.

        Note:
            This method does not currently have access to the Models part of the
            environment. If the Task is meant to work on different models, we recommend
            using their URDF / SDF model to extract the information you need
            (e.g. number of DoFs, joint position limits, etc). Since actions and
            observations are often normalized, in many cases there's no need to extract
            a lot of information from the model file.

        Raises:
            RuntimeError: In case of failure.

        Returns:
            A tuple containing the action and observation spaces.
        """

    @abc.abstractmethod
    def reset_task(self) -> None:
        """
        Reset the task.

        This method contains the logic for resetting the task.
        It is called in the :py:meth:`gym.Env.reset` method of the corresponding
        environment.

        Raises:
            RuntimeError: In case of failure.
        """

    @abc.abstractmethod
    def set_action(self, action: Action) -> None:
        """
        Set the task action.

        This method contains the logic for setting the environment action.
        It is called in the beginning of the :py:meth:`gym.Env.step` method.

        Args:
            action: The action to set.

        Raises:
            RuntimeError: In case of failure.
        """

    @abc.abstractmethod
    def get_observation(self) -> Observation:
        """
        Return the task observation.

        This method contains the logic for constructing the environment observation.
        It is called in the end of both :py:meth:`gym.Env.reset` and
        :py:meth:`gym.Env.step` methods.

        Raises:
            RuntimeError: In case of failure.

        Returns:
            The task observation.
        """

    @abc.abstractmethod
    def get_reward(self) -> Reward:
        """
        Return the task reward.

        This method contains the logic for computing the environment reward.
        It is called in the end of the :py:meth:`gym.Env.step` method.

        Raises:
            RuntimeError: In case of failure.

        Returns:
            The scalar reward.
        """

    @abc.abstractmethod
    def is_done(self) -> bool:
        """
        Return the task termination flag.

        This method contains the logic for defining when the environment has terminated.
        Subsequent calls to :py:meth:`Task.set_action` should be preceded by a task
        reset through :py:meth:`Task.reset_task`.

        It is called in the end of the :py:meth:`gym.Env.step` method.

        Raises:
            RuntimeError: In case of failure.

        Returns:
            True if the environment terminated, False otherwise.
        """

    def get_info(self) -> Dict:
        """
        Return the info dictionary.

        Returns:
            A ``dict`` with extra information of the task.
        """
        return {}

    def seed_task(self, seed: int = None) -> SeedList:
        """
        Seed the task.

        This method configures the :py:attr:`Task.np_random` RNG.

        Args:
            seed: The seed number.

        Return:
            The list of seeds used by the task.
        """

        # Create the seed if not passed
        seed = np.random.randint(2**32 - 1) if seed is None else seed

        # Get an instance of the random number generator from gym utils.
        # This is necessary to have an independent rng for each environment.
        self.np_random, self.seed = seeding.np_random(seed)

        # Seed the spaces
        self.action_space.seed(self.seed)
        self.observation_space.seed(self.seed)

        return SeedList([self.seed])
