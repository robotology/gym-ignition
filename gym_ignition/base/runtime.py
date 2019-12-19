# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym
from gym_ignition.base.task import Task


class Runtime(gym.Wrapper, abc.ABC):
    """
    Base class for defining Task runtimes.

    The runtime is the executor of a Task. Tasks are supposed to be generic and are not
    tied to any specific Runtime. Implementations of the Runtime class contain all the
    logic to define how to execute the Task, allowing to reuse the same Task class
    e.g. on different simulators or in a real-time setting.

    Runtime objects are the real environments returned to the users when they use the
    gym.make factory method. Despite they inherit from gym.Wrapper, they are exposed to
    the user as gym.Env objects.
    """

    # Add a spec class variable to make the wrapper look like an environment.
    # This class variable overrides the gym.Wrapper.spec property and enables registering
    # runtime objects in the environment factory, despite they inherit from Wrapper.
    spec = None

    def __init__(self, task: Task, agent_rate: float):
        # Initialize the gym.Wrapper class
        super().__init__(env=task)

        # All runtimes provide to the user the nominal rate of their execution
        self.agent_rate = agent_rate

    @property
    def task(self):
        return self.env

    @property
    def unwrapped(self):
        """
        Expose Runtime objects as real gym.Env objects.

        This method disables the gym.Wrapper logic to returned the wrapped environment.
        In fact, the wrapped environment is a Task which contains not a implemented
        gym.Env interface.

        Returns:
            The gym.Env environment of the Runtime.
        """
        return self

    @abc.abstractmethod
    def timestamp(self) -> float:
        """
        Return the timestamp associated to the execution of the environment.

        In real-time environments, the timestamp is the time read from the host system.
        In simulated environments, the timestamp is the simulated time, which might not
        match the real-time in the case of RTF different than 1.

        Returns:
            A float indicating the current environment timestamp
        """
