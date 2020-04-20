# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym_ignition.randomizers.base import physics
from gym_ignition import scenario_bindings as bindings


class DART(physics.PhysicsRandomizer):
    """
    Class that configures the Ignition Gazebo physics with the DART physics engine and
    no randomization.
    """

    def __init__(self, seed: int = None):

        super().__init__()

        if seed is not None:
            self.seed_physics_randomizer(seed=seed)

    def randomize_physics(self, world: bindings.World) -> None:

        # Insert the physics
        ok_physics = world.insertWorldPlugin("libPhysicsSystem.so",
                                             "scenario::plugins::gazebo::Physics")

        if not ok_physics:
            raise RuntimeError("Failed to insert the physics plugin")
