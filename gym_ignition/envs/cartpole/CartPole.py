# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym_ignition import IgnitionEnv
from gympp import PluginMetadata, SpaceMetadata, SpaceType_Discrete, SpaceType_Box
import numpy as np


class CartPoleEnv(IgnitionEnv):
    def __init__(self):
        IgnitionEnv.__init__(self)

    def _plugin_metadata(self):
        md = PluginMetadata()
        md.setEnvironmentName("CartPole")
        md.setLibraryName("CartPolePlugin")
        md.setClassName("gympp::plugins::CartPole")
        md.setWorldFileName("CartPole.world")
        md.setModelNames(["cartpole_xacro"])

        action_space_md = SpaceMetadata()
        action_space_md.setType(SpaceType_Discrete)
        action_space_md.setDimensions([3])

        observation_space_md = SpaceMetadata()
        observation_space_md.setType(SpaceType_Box)
        max_float = float(np.finfo(np.float32).max)
        observation_space_md.setLowLimit([-2.4, -max_float, -24, -max_float])
        observation_space_md.setHighLimit([2.4, max_float, 24, max_float])

        md.setActionSpaceMetadata(action_space_md)
        md.setObservationSpaceMetadata(observation_space_md)

        return md
