# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
from gym_ignition import base
from gym_ignition import scenario_bindings as bindings


class TaskRandomizer(abc.ABC):

    @abc.abstractmethod
    def randomize_task(self,
                       task: base.task.Task,
                       gazebo: bindings.GazeboSimulator,
                       **kwargs) -> None:
        """
        Randomize a :py:class:`~gym_ignition.base.task.Task` instance.

        Args:
            task: the task to randomize.
            gazebo: a :py:class:`~scenario_bindings.GazeboSimulator` instance.

        Note:
            Note that each task has a :py:attr:`~gym_ignition.base.task.Task.world`
            property that provides access to the simulated
            :py:class:`scenario_bindings.World`.
        """
        pass

    def seed_task_randomizer(self, seed: int) -> None:
        """
        Seed the randomizer to ensure reproducibility.

        Args:
            seed: The seed number.
        """
        pass
