# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import sys
import gym
from gym import spaces
import numpy as np
from numbers import Number
from typing import List, Tuple, Union, NewType

# Import gympp bindings
# See https://github.com/robotology/gym-ignition/issues/7
if sys.platform.startswith('linux') or sys.platform.startswith('darwin'):
    import os
    sys.setdlopenflags(sys.getdlopenflags() | os.RTLD_GLOBAL)
import gympp

Action = NewType('Action', Union[float, np.ndarray, np.number])
Observation = NewType('Observation', np.array)
Reward = NewType('Reward', float)


class IgnitionEnv(gym.Env):
    """The main gym_ignition class. It encapsulates environments created in c++ with
    gympp and hides the swig bindings from the gym user, exposing only a regular
    OpenAI Gym interface.

    The environments that inherit from this class must implement the _plugin_metadata
    method. The information that it provides are then enough to load the ignition
    plugin and insert it in a Gazebo environment.
    """

    metadata = {'render.modes': ['human']}

    def __init__(self) -> None:

        # Get the plugin metadata
        self.md = self._plugin_metadata()

        # Create the spaces
        self.action_space, self.act_dt = IgnitionEnv._create_space(
            self.md.getActionSpaceMetadata())
        self.observation_space, self.obs_dt = IgnitionEnv._create_space(
            self.md.getObservationSpaceMetadata())

        # Register the environment
        factory = gympp.GymFactory.Instance()
        registered = factory.registerPlugin(self.md)
        assert registered, "Failed to register the plugin environment"

        # Load the environment from gympp
        self.ignenv = factory.make(self.md.getEnvironmentName())
        assert self.ignenv, "Failed to create environment " + self.md.getEnvironmentName()

    def step(self, action: Action) -> Tuple[Observation, Reward, bool, str]:
        assert self.action_space.contains(action), "The action does not belong to the action space"

        # The bindings do not accept yet numpy types as arguments. We need to covert
        # numpy variables to the closer python type.

        # Check if the input variable is a numpy type
        is_numpy = type(action).__module__ == np.__name__

        if is_numpy:
            if isinstance(action, np.ndarray):
                action = action.tolist()
            elif isinstance(action, np.number):
                action = action.item()
            else:
                assert False

        # Actions must be std::vector objects, so if the passed action is a scalar
        # we have to store it inside a list object before passing it to the bindings
        if isinstance(action, Number):
            action_list = [action]
        else:
            action_list = list(action)

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
        return observation, state.reward, state.done, state.info

    def reset(self) -> Observation:
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
        assert self.observation_space.contains(observation),\
            "The returned observation does not belong to the space"

        # Return the list
        return observation

    def render(self, mode: str = 'human') -> None:
        rendermode = {'human': gympp.Environment.RenderMode_HUMAN}
        ok = self.ignenv.render(rendermode[mode])
        assert ok, "Failed to render environment"

    def close(self) -> None:
        return

    def seed(self, seed: int = None) -> List[int]:
        if seed:
            assert isinstance(seed, int), "The seed must be a positive integer"
            assert seed > 0, "The seed must be a positive integer"
        else:
            seed = np.random.randint(low=1, high=1000)

        # TODO: it would be nice having the same behavior of the environment
        #       if executed from cpp and python. However, the spaces are
        #       different. We can obtain this only if we manage to map
        #       gym.Space objects out of gympp::Space objects.

        # Seed the environment
        vector_seeds = self.ignenv.seed(seed)

        # Seed numpy
        np.random.seed(seed)

        # Seed the spaces
        self.action_space.seed(seed)
        self.observation_space.seed(seed)

        return list(vector_seeds)

    def _plugin_metadata(self) -> gympp.PluginMetadata:
        """Return metadata of the gympp plugin

        Loading an environment created with gympp in python requires the knowledge of
        the plugin metadata that implements it. Every environment should implement this
        method.
        """
        raise NotImplementedError
        return gympp.PluginMetadata()

    @classmethod
    def _create_space(cls, md: gympp.SpaceMetadata = None) \
            -> Union[Tuple[gympp.Box, str], Tuple[gympp.Discrete, str]]:
        """Create an object of the gym.space package from gympp space metadata

        Note: In order to map the dynamically typed nature of python to C++, this class
              must know the data type of the gympp buffers to call the right C++ template
              instance stored in the swig bindings. For this reason, this method returns
              also a method suffix string (such as "_d" for double) that is appended to
              the calls of the swig bindings methods (e.g. obs.getBuffer_d()).
        """
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
