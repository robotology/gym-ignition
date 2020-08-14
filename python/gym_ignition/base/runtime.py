# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym
from gym_ignition import base


class Runtime(gym.Env, abc.ABC):
    """
    Base class for defining executors of :py:class:`~gym_ignition.base.task.Task` objects.

    :py:class:`~gym_ignition.base.task.Task` classes are supposed to be generic and are
    not tied to any specific runtime. Implementations of a runtime class contain all the
    logic to define how to execute the task, allowing to run the same
    :py:class:`~gym_ignition.base.task.Task` class on different simulators or in a
    real-time setting.

    Runtimes are the real :py:class:`gym.Env` objects returned to the users when they call
    the :py:class:`gym.make` factory method.

    Args:
        task: the :py:class:`~gym_ignition.base.task.Task` object to handle.
        agent_rate: the rate at which the environment will be called. Sometimes tasks need
            to know this information.

    Example:
        Here is minimal example of how the :py:class:`Runtime`, :py:class:`gym.Env` and
        :py:class:`~gym_ignition.base.task.Task` could be integrated:

    .. code-block:: python

        class FooRuntime(Runtime):

            def __init__(self, task):
                super().__init__(task=task, agent_rate=agent_rate)
                self.action_space, self.observation_space = self.task.create_spaces()

            def reset(self):
                self.task.reset_task()
                return self.task.get_observation()

            def step(self, action):
                self.task.set_action(action)

                # [...] code that performs the real step [...]

                done = self.task.is_done()
                reward = self.task.get_reward()
                observation = self.task.get_observation()

                return observation, reward, done, {}

            def close(self):
                pass

    Note:
        Runtimes can handle only one :py:class:`~gym_ignition.base.task.Task` object.
    """

    def __init__(self, task: base.task.Task, agent_rate: float):

        #: Task handled by the runtime.
        self.task: base.task.Task = task

        #: Rate of environment execution.
        self.agent_rate = agent_rate

    @abc.abstractmethod
    def timestamp(self) -> float:
        """
        Return the timestamp associated to the execution of the environment.

        In real-time environments, the timestamp is the time read from the host system.
        In simulated environments, the timestamp is the simulated time, which might not
        match the real-time in the case of a real-time factor different than 1.

        Returns:
            The current environment timestamp.
        """
