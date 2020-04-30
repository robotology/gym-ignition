# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
pytestmark = pytest.mark.gympp

import os
import numpy as np
from pathlib import Path
import gym_ignition_models
from gym_ignition.utils import misc
from gym_ignition import scenario_bindings
from gym_ignition.experimental import gympp

try:
    from gym_ignition import gympp_bindings as bindings
except ImportError:
    pytest.skip("gympp bindings not found", allow_module_level=True)


# Set verbosity
scenario_bindings.set_verbosity(4)


def test_metadata():

    md = bindings.PluginMetadata()
    assert not md.isValid()

    environment_name = "EnvironmentName"
    md.setEnvironmentName(environment_name)
    assert md.getEnvironmentName() == environment_name

    library_name = "plugin_foo"
    md.setLibraryName(library_name)
    assert md.getLibraryName() == library_name

    class_name = "foo::bar::GymppPlugin"
    md.setClassName(class_name)
    assert md.getClassName() == class_name

    model_name = "RobotModel.sdf"
    md.setModelFileName(model_name)
    assert md.getModelFileName() == model_name

    world_name = "Environment.world"
    md.setWorldFileName(world_name)
    assert md.getWorldFileName() == world_name

    agent_rate = 1000
    md.setAgentRate(agent_rate)
    assert md.getAgentRate() == agent_rate

    real_time_factor = 1E9
    max_physics_step_size = 0.001
    md.setPhysicsData(bindings.PhysicsData(real_time_factor, max_physics_step_size))
    physics_data = md.getPhysicsData()
    assert physics_data.rtf == real_time_factor
    assert physics_data.maxStepSize == max_physics_step_size


def test_gym_factory():

    # Get the factory
    factory = bindings.GymFactory.Instance()

    # Register a plugin with empty metadata
    md = bindings.PluginMetadata()
    assert not factory.registerPlugin(md)

    # Get a not registered environment
    env = factory.make("foo")
    assert not env

    # Check that the CartPole plugin exists
    assert "IGN_GAZEBO_SYSTEM_PLUGIN_PATH" in os.environ, \
        "Variable IGN_GAZEBO_SYSTEM_PLUGIN_PATH not set in the environment"
    directories = os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'].split(os.pathsep)

    found = False
    for directory in directories:
        matching_plugins = list(Path(directory).glob("*CartPole*"))
        found = found or (len(matching_plugins) is not 0)
    assert found

    world_file = misc.string_to_file(gympp.get_empty_world())
    cartpole_urdf = gym_ignition_models.get_model_file("cartpole")

    # Create the metadata
    md = bindings.PluginMetadata()
    md.setEnvironmentName("CartPole")
    md.setLibraryName("CartPolePlugin")
    md.setClassName("gympp::plugins::CartPole")
    md.setWorldFileName(world_file)
    md.setModelFileName(cartpole_urdf)
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
    assert md.isValid()

    # Register the metadata
    assert factory.registerPlugin(md)

    # Get the environment
    env = factory.make("CartPole")
    assert env

    # Get the gazebo wrapper
    gazebo = bindings.envToGazeboWrapper(env)
    assert gazebo

    # Get the ignition environment
    ign_env = bindings.envToGazeboEnvironment(env)
    assert ign_env

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
    assert factory.registerPlugin(md)

    # Try to get another environment
    env2 = factory.make("CartPole")
    assert env2, "Failed to create CartPoleIgnition environment from the factory"
    assert env != env2, "Environment created from the factory are the same"
    env2.reset()
    env2.step(action)
