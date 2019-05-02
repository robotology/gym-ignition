# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import sys
import gym
from gym import spaces
from gym.utils import seeding
# import numpy as np
from numbers import Number
from gym_ignition.utils.typing import *

# Import gympp bindings
# See https://github.com/robotology/gym-ignition/issues/7
if sys.platform.startswith('linux') or sys.platform.startswith('darwin'):
    import ctypes
    sys.setdlopenflags(sys.getdlopenflags() | ctypes.RTLD_GLOBAL)
import gympp


class IgnitionEnv(gym.Env):
    """Class that exposes C++ Ignition environments

    This class encapsulates environments created as C++ plugins. Plugins that implement
    the provided gympp and ignition interfaces are inherently compatible with this
    class if they support the C++ factory.

    The users of this class need to implement the following method:

    _plugin_metadata

    The methods inherited from gym.Env are already implemented.
    """

    metadata = {'render.modes': ['human']}

    def __init__(self):
        # Private attributes
        self._env = None
        self._action_space = None
        self._observation_space = None
        self._act_dt = None
        self._obs_dt = None
        self._robot = None

        # Seed the environment
        self.seed()

        # Trigger the creation of the spaces
        action_space = self.action_space
        observation_space = self.observation_space
        assert(action_space and observation_space), "Failed to create spaces"

    @property
    def gympp_env(self) -> gympp.GazeboWrapper:
        if self._env:
            return self._env

        # Get the metadata
        md = self._plugin_metadata

        # Register the environment
        factory = gympp.GymFactory.Instance()
        factory.registerPlugin(md)

        # Load the environment from gympp
        self._env = factory.make(md.getEnvironmentName())
        assert self._env, "Failed to create environment " + md.getEnvironmentName()

        # Set the verbosity. Run the script as optimized (-O) to decrease the verbosity.
        gympp.GazeboWrapper.setVerbosity(2)
        assert (gympp.GazeboWrapper.setVerbosity(4) or True)

        # Return the gympp environment
        return self._env

    @property
    def action_space(self) -> gym.Space:
        if self._action_space:
            return self._action_space

        # Create the action space from the metadata
        md = self._plugin_metadata
        self._action_space, self._act_dt = self._create_space(md.getActionSpaceMetadata())

        return self._action_space

    @property
    def observation_space(self) -> gym.Space:
        if self._observation_space:
            return self._observation_space

        # Create the action space from the metadata
        md = self._plugin_metadata
        self._observation_space, self._obs_dt = self._create_space(
            md.getObservationSpaceMetadata())

        return self._observation_space

    @property
    def robot(self) -> gympp.Robot:
        if self._robot:
            assert self._robot.valid(), "The Robot object is not valid"
            return self._robot

        # Get the robot name
        gazebo_wrapper = gympp.envToGazeboWrapper(self.gympp_env)
        model_names = gazebo_wrapper.getModelNames()
        assert len(model_names) == 1, "The environment has more than one model"
        model_name = model_names[0]

        # Get the pointer to the Robot object
        self._robot = gympp.RobotSingleton_get().getRobot(model_name)
        assert self._robot, "Failed to get the Robot object"

        # Return the object
        return self._robot

    def step(self, action: Action) -> State:
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
        action_buffer = getattr(gympp, 'Vector' + self._act_dt)(action_list)
        action_sample = gympp.Sample(action_buffer)

        # Execute the step and get the std::optional<gympp::State> object
        state_optional = self.gympp_env.step(action_sample)
        assert state_optional.has_value()

        # Get the gympp::State
        state = state_optional.value()

        # Get the std::vector buffer of gympp::Observation
        observation_vector = getattr(state.observation, 'getBuffer' + self._obs_dt)()
        assert observation_vector, "Failed to get the observation buffer"
        assert observation_vector.size() > 0, "The observation does not contain elements"

        # Convert the observation to a numpy array (this is the only required copy)
        if isinstance(self.observation_space, gym.spaces.Box):
            observation = np.array(observation_vector)
        elif isinstance(self.observation_space, gym.spaces.Discrete):
            assert observation_vector.size() == 1, "The buffer has the wrong dimension"
            observation = observation_vector[0]
        else:
            assert False, "Space not supported"

        assert self.observation_space.contains(observation), \
            "The returned observation does not belong to the space"

        # Create the info dict
        info = {'gympp': state.info}

        # Return the tuple
        return State((observation, state.reward, state.done, info))

    def reset(self) -> Observation:
        # Get std::optional<gympp::Observation>
        obs_optional = self.gympp_env.reset()
        assert obs_optional.has_value(), "The environment didn't return the observation"

        # Get gympp::Observation
        gympp_observation = obs_optional.value()

        # Get the std::vector buffer of gympp::Observation
        observation_vector = getattr(gympp_observation, 'getBuffer' + self._obs_dt)()
        assert observation_vector, "Failed to get the observation buffer"
        assert observation_vector.size() > 0, "The observation does not contain elements"

        # Convert the observation to a numpy array (this is the only required copy)
        if isinstance(self.observation_space, gym.spaces.Box):
            observation = np.array(observation_vector)
        elif isinstance(self.observation_space, gym.spaces.Discrete):
            assert observation_vector.size() == 1, "The buffer has the wrong dimension"
            observation = observation_vector[0]
        else:
            assert False, "Space not supported"

        assert self.observation_space.contains(observation),\
            "The returned observation does not belong to the space"

        # Return the list
        return observation

    def render(self, mode: str = 'human') -> None:
        rendermode = {'human': gympp.Environment.RenderMode_HUMAN}
        ok = self.gympp_env.render(rendermode[mode])
        assert ok, "Failed to render environment"

    def close(self) -> None:
        return

    def seed(self, seed: int = None) -> SeedList:
        # Create the seed and the random numbers generator
        np_random, seed = seeding.np_random(seed)

        # Spaces need to be seeded. Apply the same logic contained in gym.seeding.
        short_seed = seeding._int_list_from_bigint(seeding.hash_seed(seed))

        # TODO: it would be nice having the same behavior of the environment
        #       if executed from cpp and python. However, the spaces are
        #       different. We can obtain this only if we manage to map
        #       gym.Space objects out of gympp::Space objects.

        # Seed the environment
        vector_seeds = self.gympp_env.seed(short_seed[0])

        # Seed the spaces
        self.action_space.seed(short_seed)
        self.observation_space.seed(short_seed)

        return SeedList(list(vector_seeds))

    @property
    def _plugin_metadata(self) -> gympp.PluginMetadata:
        """Return metadata of the gympp plugin

        Loading an environment created with gympp in python requires the knowledge of
        the plugin metadata that implements it. Every environment should implement this
        method.

        Returns:
            gympp.PluginMetadata: The metadata of the C++ environment plugin
        """
        raise NotImplementedError

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