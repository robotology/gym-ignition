# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym_ignition.base.task
from gym_ignition import randomizers
from scenario import gazebo as scenario


class DART(randomizers.abc.PhysicsRandomizer):
    """
    Class that configures the Gazebo World of a Task to use the DART physics engine.

    Note:
        This class does not apply any physics randomization.
    """

    def __init__(self):

        super().__init__()

    def get_engine(self):

        return scenario.PhysicsEngine_dart

    def randomize_physics(self, task: gym_ignition.base.task.Task) -> None:

        pass
