# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import sys
import gym
from gym import spaces
import numpy as np

# Import gympp bindings
# See https://github.com/robotology/gym-ignition/issues/7
if sys.platform.startswith('linux') or sys.platform.startswith('darwin'):
    import os
    sys.setdlopenflags(sys.getdlopenflags() | os.RTLD_GLOBAL)
import gympp

class IgnitionEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):

        # Get the plugin metadata
        self.md = self._plugin_metadata()

        # Create the spaces
        self.action_space, self.act_dt = self._create_space(self.md.getActionSpaceMetadata())
        self.observation_space, self.obs_dt = self._create_space(self.md.getObservationSpaceMetadata())

        # Register the environment
        factory = gympp.GymFactory.Instance()
        registered = factory.registerPlugin(self.md)
        assert registered, "Failed to register the plugin environment"

        # Load the environment from gympp
        self.ignenv = factory.make(self.md.getEnvironmentName())
        assert self.ignenv, "Failed to create environment " + self.md.getEnvironmentName()

    def step(self, action):
        assert self.action_space.contains(action), "The action does not belong to the action space"

        if not isinstance(action, list):
            action_list = [action]

        # Create the gympp::Sample object
        action_buffer = getattr(gympp, 'Vector' + self.act_dt)(action_list)
        action_sample = gympp.Sample(action_buffer)

        # Execute the step and get the std::optional<gympp::State> object
        state_optional = self.ignenv.step(action_sample)
        assert state_optional.has_value()

        # Get the gympp::State
        state = state_optional.value()

        # Get the std::vector buffer of gympp::Observation
        observation_vector = getattr(state.observation, 'getBuffer' + self.obs_dt)()
        assert observation_vector, "Failed to get the observation buffer"
        assert observation_vector.size() > 0, "The observation does not contain elements"

        # Convert it to a numpy array (this is the only required copy)
        observation = np.array(observation_vector)
        assert self.observation_space.contains(observation), "The returned observation does not belong to the space"

        # Return the tuple
        return (observation, state.reward, state.done, state.info)

    def reset(self):
        # Get std::optional<gympp::Observation>
        obs_optional = self.ignenv.reset()
        assert obs_optional.has_value(), "The environment didn't return the observation"

        # Get gympp::Observation
        gympp_observation = obs_optional.value()

        # Get the std::vector buffer of gympp::Observation
        observation_vector = getattr(gympp_observation, 'getBuffer' + self.obs_dt)()
        assert observation_vector, "Failed to get the observation buffer"
        assert observation_vector.size() > 0, "The observation does not contain elements"

        # Convert it to a numpy array (this is the only required copy)
        observation = np.array(observation_vector)
        assert self.observation_space.contains(observation), "The returned observation does not belong to the space"

        # Return the list
        return observation

    def render(self, mode='human'):
        rendermode = {'human': gympp.Environment.RenderMode_HUMAN}
        ok = self.ignenv.render(rendermode[mode])
        assert ok, "Failed to render environment"

    def close(self):
        return

    def seed(self, seed=None):
        if seed:
            assert isinstance(seed, int), "The seed must be a positive integer"
            assert seed > 0, "The seed must be a positive integer"
        else:
            seed = 0

        vector_seeds = self.ignenv.seed(seed)
        return list(vector_seeds)

    def _plugin_metadata(self):
        raise NotImplementedError

    def _create_space(self, md=None):
        assert isinstance(md, gympp.SpaceMetadata), "Wrong type for method argument"

        space_type = md.getType()
        low = md.getLowLimit()
        high = md.getHighLimit()
        dims = md.getDimensions()

        if space_type is gympp.SpaceType_Box:
            if not dims:
                assert len(low) == len(high), "Sizes of low and high limits do not match"
                return spaces.Box(np.array(low), np.array(high)), "_d"
            else:
                assert len(low) == 1, "The size of the limit is not valid"
                assert len(high) == 1, "The size of the limit is not valid"
                return (spaces.Box(low[0], high[0], dims), "_d")

        elif space_type is gympp.SpaceType_Discrete:
            assert len(dims) == 1, "The specified space dimension is not valid"
            return (spaces.Discrete(dims[0]), "_i")

        else:
            assert False, "This space type is not supported"
