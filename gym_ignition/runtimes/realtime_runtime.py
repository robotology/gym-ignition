# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym_ignition.base import runtime, task
from gym_ignition.utils.typing import State, Action, Observation, Done, Info


class RealTimeRuntime(runtime.Runtime):
    """
     Implementation of :py:class:`~gym_ignition.base.runtime.Runtime` for real-time
     execution.

     Warning:
         This class is not yet complete.
    """

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

        raise NotImplementedError

    # =================
    # Runtime interface
    # =================

    def timestamp(self) -> float:

        raise NotImplementedError

    # =================
    # gym.Env interface
    # =================

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
        reward = self.task.get_reward()
        assert reward, "Failed to get the reward"

        # Check termination
        done = self.task.is_done()

        return State((observation, reward, Done(done), Info({})))

    def reset(self) -> Observation:

        # Get the observation
        observation = self.task.get_observation()

        return Observation(observation)

    def render(self, mode: str = 'human', **kwargs) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError
