# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym_ignition.base.task
from scenario import gazebo as scenario
from gym_ignition.randomizers.base import physics


class DART(physics.PhysicsRandomizer):
    """
    Class that configures the Gazebo World of a Task to use the DART physics engine.

    Note:
        This class does not apply any physics randomization.
    """

    def __init__(self, seed: int = None):

        super().__init__()

        if seed is not None:
            self.seed_physics_randomizer(seed=seed)

    def randomize_physics(self, task: gym_ignition.base.task.Task) -> None:

        # Set Gazebo to use the DART physics engine
        if not task.world.to_gazebo().set_physics_engine(scenario.PhysicsEngine_dart):
            raise RuntimeError("Failed to insert the physics plugin")
