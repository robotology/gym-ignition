# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
from gym_ignition.base import gympp_env
from gym_ignition import gympp_bindings as bindings


class CartPoleDiscrete(gympp_env.GymppEnv):
    def __init__(self):
        # Initialize the parent class
        super().__init__()

    @property
    def _plugin_metadata(self) -> bindings.PluginMetadata:
        md = bindings.PluginMetadata()

        # Configure ignition environment
        md.setEnvironmentName("CartPole")
        md.setLibraryName("CartPolePlugin")
        md.setClassName("gympp::plugins::CartPole")
        md.setWorldFileName("DefaultEmptyWorld.world")
        md.setModelFileName("CartPole/CartPole.urdf")

        md.setAgentRate(1000)

        real_time_factor = 1E9
        max_physics_step_size = 0.001
        md.setPhysicsData(bindings.PhysicsData(real_time_factor, max_physics_step_size))

        # Configure the action space
        action_space_md = bindings.SpaceMetadata()
        action_space_md.setType(bindings.SpaceType_Discrete)
        action_space_md.setDimensions([2])

        # Configure the observation space
        observation_space_md = bindings.SpaceMetadata()
        observation_space_md.setType(bindings.SpaceType_Box)
        max_float = float(np.finfo(np.float32).max)
        observation_space_md.setLowLimit([-2.5, -max_float, -24, -max_float])
        observation_space_md.setHighLimit([2.5, max_float, 24, max_float])

        # Store the spaces information
        md.setActionSpaceMetadata(action_space_md)
        md.setObservationSpaceMetadata(observation_space_md)

        # Return the metadata
        assert md.isValid(), "The plugin metadata object is not valid"
        return md
