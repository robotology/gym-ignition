# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym_ignition import IgnitionEnv
from gympp import PluginMetadata, SpaceMetadata, SpaceType_Discrete, SpaceType_Box
import numpy as np


class CartPoleEnv(IgnitionEnv):
    def __init__(self):
        # Initialize the parent class
        super().__init__()

    @property
    def _plugin_metadata(self) -> PluginMetadata:
        md = PluginMetadata()

        # Configure ignition environment
        md.setEnvironmentName("CartPole")
        md.setLibraryName("CartPolePlugin")
        md.setClassName("gympp::plugins::CartPole")
        md.setWorldFileName("DefaultEmptyWorld.world")
        md.setModelFileName("CartPole/CartPole.sdf")
        md.setGazeboUpdateRate(1000000000)
        md.setEnvironmentUpdateRate(md.getGazeboUpdateRate() / 10)

        # Configure the action space
        action_space_md = SpaceMetadata()
        action_space_md.setType(SpaceType_Discrete)
        action_space_md.setDimensions([2])

        # Configure the observation space
        observation_space_md = SpaceMetadata()
        observation_space_md.setType(SpaceType_Box)
        max_float = float(np.finfo(np.float32).max)
        observation_space_md.setLowLimit([-2.5, -max_float, -24, -max_float])
        observation_space_md.setHighLimit([2.5, max_float, 24, max_float])

        # Store the spaces information
        md.setActionSpaceMetadata(action_space_md)
        md.setObservationSpaceMetadata(observation_space_md)

        # Return the metadata
        assert md.isValid(), "The plugin metadata object is not valid"
        return md
