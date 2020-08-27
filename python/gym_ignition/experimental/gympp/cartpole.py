# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
import gym_ignition_models
from gym_ignition.utils import misc
from gym_ignition.experimental import gympp
from gym_ignition.experimental.gympp import gympp_env


class CartPoleDiscrete(gympp_env.GymppEnv):

    def __init__(self):

        # Initialize the parent class
        super().__init__()

    @property
    def _plugin_metadata(self) -> "gympp_bindings.PluginMetadata":

        # Lazy module import
        import gympp_bindings as bindings

        # Get the empty world file
        empty_world_sdf = misc.string_to_file(gympp.get_empty_world())

        # Configure ignition environment
        md = bindings.PluginMetadata()
        md.set_environment_name("CartPole")
        md.set_library_name("CartPolePlugin")
        md.set_class_name("gympp::plugins::CartPole")
        md.set_world_file_name(empty_world_sdf)
        md.set_model_file_name(gym_ignition_models.get_model_file("cartpole"))
        md.set_agent_rate(1000)

        real_time_factor = 1E9
        max_physics_step_size = 0.001
        md.set_physics_data(bindings.PhysicsData(real_time_factor, max_physics_step_size))

        # Configure the action space
        action_space_md = bindings.SpaceMetadata()
        action_space_md.set_type(bindings.SpaceType_discrete)
        action_space_md.set_dimensions([2])

        # Configure the observation space
        observation_space_md = bindings.SpaceMetadata()
        observation_space_md.set_type(bindings.SpaceType_box)
        max_float = float(np.finfo(np.float32).max)
        observation_space_md.set_low_limit([-2.5, -max_float, -24, -max_float])
        observation_space_md.set_high_limit([2.5, max_float, 24, max_float])

        # Store the spaces information
        md.set_action_space_metadata(action_space_md)
        md.set_observation_space_metadata(observation_space_md)

        # Return the metadata
        assert md.is_valid(), "The plugin metadata object is not valid"
        return md
