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
        # TODO: should agent rate be the real one with rtf taken into account? Call it
        #  env_update_rate?

    # Redefine this magic method since the Wrapper.__getattr__ is not compatible with
    # the usage we want to achieve with Runtime objects. In fact, we want to resolve
    # attributes in the following order:
    #
    # 1. All non-private attributes of the runtime implementation (example: get the
    #    simulator object).
    # 2. Get all the non-private attributes of the environment calling the Wrapper's
    #    getattr implementation.
    #
    def __getattr__(self, name):
        if name in self.__dict__:
            # Hide private attributes
            if not name.startswith('_'):
                return __dict__[name]
        else:
            # Call the Wrapper getattr
            return super().__getattr__(name)

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
