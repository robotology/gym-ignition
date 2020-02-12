# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import gym
import pytest
import numpy as np
from numpy import pi
from pathlib import Path
from gym_ignition.utils import logger
from gym_ignition import gympp_bindings as bindings

# Set verbosity
logger.set_level(gym.logger.DEBUG)


@pytest.fixture
def create_std_vector():
    python_list = [1.0, 2.0, 3.0]
    vector = bindings.Vector_d(python_list)
    return vector


@pytest.fixture
def create_space_box_md():
    md = bindings.SpaceMetadata()
    md.setType(bindings.SpaceType_Box)
    max_float = float(np.finfo(np.float32).max)
    md.setLowLimit([-2.5, -max_float, -24, -max_float])
    md.setHighLimit([2.5, max_float, 24, max_float])
    return md


@pytest.fixture
def create_space_discrete_md():
    md = bindings.SpaceMetadata()
    md.setType(bindings.SpaceType_Discrete)
    md.setDimensions([2])
    return md


def test_vectors():
    python_list = [1.0, 2.0, 3.0]
    vector = bindings.Vector_d(python_list)
    for i in range(0, vector.size()-1):
        assert python_list[i] == vector[i], "Vectors do not match"


def test_sample(create_std_vector):
    vector = create_std_vector
    sample = bindings.Sample(vector)

    for i in range(0, vector.size()-1):
        assert sample.getBuffer_d()[i] == vector[i], \
            "Sample object does not contain correct data"

    sample.getBuffer_d()[2] = 42
    assert sample.get_d(2).value() == 42, "Failed to insert data in the Sample object"
    assert sample.getBuffer_d()[2] == 42, "Failed to update data of a Sample object"


def test_range():
    gympp_range = bindings.Range(-5.0, 10.0)
    assert gympp_range.contains(0), "Range object failed to verify if it containes a value"
    assert gympp_range.contains(-5), "Range object failed to verify if it containes a value"
    assert gympp_range.contains(10), "Range object failed to verify if it containes a value"
    assert not gympp_range.contains(-6), "Range object failed to verify if it containes a value"
    assert not gympp_range.contains(10.1), "Range object failed to verify if it containes a value"


def test_discrete_space():
    discrete_space = bindings.Discrete(12)

    assert not discrete_space.contains(bindings.Sample([-1])), \
        "Discrete object failed to verify if a sample belongs to its space"
    assert not discrete_space.contains(bindings.Sample([13])), \
        "Discrete object failed to verify if a sample belongs to its space"

    for n in range(50):
        sample = discrete_space.sample()
        assert sample.getBuffer_i().size() == 1, \
            "Wrong size of the sample extracted from a discrete space"
        assert isinstance(sample.getBuffer_i()[0], int), \
            "Wrong data type of the sample extracted from a discrete space"
        assert discrete_space.contains(sample), \
            "Sampled data is not contained in the discrete space object that created it"


def test_box_space():
    size = 4
    box = bindings.Box(-1, 42, [size])

    # By default the data precision of python list is float. Force double.
    assert box.contains(bindings.Sample(bindings.Vector_d([0, pi, 12, 42]))), \
        "Box object failed to verify if a sample belongs to its space"
    assert not box.contains(bindings.Sample(bindings.Vector_d([0, pi, 12, 43]))), \
        "Box object failed to verify if a sample belongs to its space"
    assert not box.contains(bindings.Sample(bindings.Vector_d([0]))), \
        "Box object failed to verify if a sample belongs to its space"
    assert not box.contains(bindings.Sample(bindings.Vector_d([0, pi, 12, 43, 0]))), \
        "Box object failed to verify if a sample belongs to its space"

    for n in range(50):
        sample = box.sample()
        assert sample.getBuffer_d().size() == size, \
            "Wrong size of the sample extracted from a box space"
        assert isinstance(sample.getBuffer_d()[0], float), \
            "Wrong data type of the sample extracted from the box space"
        assert box.contains(sample), \
            "Sampled data is not contained in the box space object that created it"


def test_space_box_metadata(create_space_box_md):
    md = create_space_box_md
    assert md.isValid(), "The space is not valid"
    assert md.getType() == bindings.SpaceType_Box, "The space type is not correct"


def test_space_discrete_metadata(create_space_discrete_md):
    md = create_space_discrete_md
    assert md.isValid(), "The space is not valid"
    assert md.getType() == bindings.SpaceType_Discrete, "The space type is not correct"


def test_metadata():
    md = bindings.PluginMetadata()
    assert not md.isValid(), "The metadata should not be valid"

    environment_name = "EnvironmentName"
    md.setEnvironmentName(environment_name)
    assert md.getEnvironmentName() == environment_name, "Failed to store the " \
                                                        "environment name"
    library_name = "plugin_foo"
    md.setLibraryName(library_name)
    assert md.getLibraryName() == library_name, "Failed to store the library name"

    class_name = "foo::bar::GymppPlugin"
    md.setClassName(class_name)
    assert md.getClassName() == class_name, "Failed to store the class name"

    model_name = "RobotModel.sdf"
    md.setModelFileName(model_name)
    assert md.getModelFileName() == model_name, "Failed to store the model file name"

    world_name = "Environment.world"
    md.setWorldFileName(world_name)
    assert md.getWorldFileName() == world_name, "Failed to store the world file name"

    agent_rate = 1000
    md.setAgentRate(agent_rate)
    assert md.getAgentRate() == agent_rate, "Failed to store the agent rate"

    real_time_factor = 1E9
    max_physics_step_size = 0.001
    md.setPhysicsData(bindings.PhysicsData(real_time_factor, max_physics_step_size))
    physics_data = md.getPhysicsData()
    assert physics_data.rtf == real_time_factor, "Failed to store RTF"
    assert physics_data.maxStepSize == max_physics_step_size, "Failed to store the " \
                                                              "max physics step size"


def test_gymfactory():
    # Get the factory
    factory = bindings.GymFactory.Instance()

    # Register a plugin with empty metadata
    md = bindings.PluginMetadata()
    assert not factory.registerPlugin(md), "The empty plugin metadata should not be valid"

    # Get a not registered environment
    env = factory.make("foo")
    assert not env, "The environment should not be valid"

    # Check that the CartPole plugin exists
    assert "IGN_GAZEBO_SYSTEM_PLUGIN_PATH" in os.environ, \
        "Variable IGN_GAZEBO_SYSTEM_PLUGIN_PATH not set in the environment"
    directories = os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'].split(os.pathsep)

    found = False
    for directory in directories:
        matching_plugins = list(Path(directory).glob("*CartPole*"))
        found = found or (len(matching_plugins) is not 0)
    assert found, "Failed to find CartPole plugin"

    # Create the metadata
    md = bindings.PluginMetadata()
    md.setEnvironmentName("CartPole")
    md.setLibraryName("CartPolePlugin")
    md.setClassName("gympp::plugins::CartPole")
    md.setWorldFileName("DefaultEmptyWorld.world")
    md.setModelFileName("CartPole/CartPole.urdf")
    md.setAgentRate(1000)
    md.setPhysicsData(bindings.PhysicsData(1.0, 0.001))
    action_space_md = bindings.SpaceMetadata()
    action_space_md.setType(bindings.SpaceType_Discrete)
    action_space_md.setDimensions([2])
    observation_space_md = bindings.SpaceMetadata()
    observation_space_md.setType(bindings.SpaceType_Box)
    max_float = float(np.finfo(np.float32).max)
    observation_space_md.setLowLimit([-2.5, -max_float, -24, -max_float])
    observation_space_md.setHighLimit([2.5, max_float, 24, max_float])
    md.setActionSpaceMetadata(action_space_md)
    md.setObservationSpaceMetadata(observation_space_md)
    assert md.isValid(), "Metadata for CartPole environment is not valid"

    # Register the metadata
    assert factory.registerPlugin(md), "Failed to register the plugin metadata in the " \
                                       "factory"

    # Get the environment
    env = factory.make("CartPole")
    assert env, "Failed to create CartPoleIgnition environment from the factory"

    # Get the gazebo wrapper
    gazebo = bindings.envToGazeboWrapper(env)
    assert gazebo, "Failed to get gazebo wrapper"

    # Get the ignition environment
    ign_env = bindings.envToIgnEnv(env)
    assert ign_env, "Failed to get the ignition environment"

    # Set verbosity
    bindings.GazeboWrapper.setVerbosity(4)

    # Use the environment
    env.reset()
    action = env.action_space.sample()
    state_opt = env.step(action)
    assert state_opt.has_value()

    state = state_opt.value()
    assert list(state.observation.getBuffer_d())

    observation = list(state.observation.getBuffer_d())
    assert len(observation) == 4

    # Try to register another environment
    assert factory.registerPlugin(md), "Failed to re-register the plugin metadata in " \
                                       "the factory"

    # Try to get another environment
    env2 = factory.make("CartPole")
    assert env2, "Failed to create CartPoleIgnition environment from the factory"
    assert env != env2, "Environment created from the factory are the same"
    env2.reset()
    env2.step(action)
