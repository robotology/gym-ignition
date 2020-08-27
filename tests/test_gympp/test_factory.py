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
from scenario import gazebo as scenario
from gym_ignition.experimental import gympp

try:
    import gympp_bindings as bindings
except ImportError:
    pytest.skip("gympp bindings not found", allow_module_level=True)


# Set verbosity
scenario.set_verbosity(4)


def test_metadata():

    md = bindings.PluginMetadata()
    assert not md.is_valid()

    environment_name = "EnvironmentName"
    md.set_environment_name(environment_name)
    assert md.get_environment_name() == environment_name

    library_name = "plugin_foo"
    md.set_library_name(library_name)
    assert md.get_library_name() == library_name

    class_name = "foo::bar::GymppPlugin"
    md.set_class_name(class_name)
    assert md.get_class_name() == class_name

    model_name = "RobotModel.sdf"
    md.set_model_file_name(model_name)
    assert md.get_model_file_name() == model_name

    world_name = "Environment.world"
    md.set_world_file_name(world_name)
    assert md.get_world_file_name() == world_name

    agent_rate = 1000
    md.set_agent_rate(agent_rate)
    assert md.get_agent_rate() == agent_rate

    real_time_factor = 1E9
    max_physics_step_size = 0.001
    md.set_physics_data(bindings.PhysicsData(real_time_factor, max_physics_step_size))
    physics_data = md.get_physics_data()
    assert physics_data.rtf == real_time_factor
    assert physics_data.max_step_size == max_physics_step_size


def test_gym_factory():

    # Get the factory
    factory = bindings.GymFactory.Instance()

    # Register a plugin with empty metadata
    md = bindings.PluginMetadata()
    assert not factory.register_plugin(md)

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
    md.set_environment_name("CartPole")
    md.set_library_name("CartPolePlugin")
    md.set_class_name("gympp::plugins::CartPole")
    md.set_world_file_name(world_file)
    md.set_model_file_name(cartpole_urdf)
    md.set_agent_rate(1000)
    md.set_physics_data(bindings.PhysicsData(1.0, 0.001))
    action_space_md = bindings.SpaceMetadata()
    action_space_md.set_type(bindings.SpaceType_discrete)
    action_space_md.set_dimensions([2])
    observation_space_md = bindings.SpaceMetadata()
    observation_space_md.set_type(bindings.SpaceType_box)
    max_float = float(np.finfo(np.float32).max)
    observation_space_md.set_low_limit([-2.5, -max_float, -24, -max_float])
    observation_space_md.set_high_limit([2.5, max_float, 24, max_float])
    md.set_action_space_metadata(action_space_md)
    md.set_observation_space_metadata(observation_space_md)
    assert md.is_valid()

    # Register the metadata
    assert factory.register_plugin(md)

    # Get the environment
    env = factory.make("CartPole")
    assert env

    # Get the gazebo wrapper
    gazebo = bindings.env_to_gazebo_simulator(env)
    assert gazebo

    # Get the ignition environment
    ign_env = bindings.env_to_gazebo_environment(env)
    assert ign_env

    # Use the environment
    env.reset()
    action = env.action_space.sample()
    state_opt = env.step(action)
    assert state_opt.has_value()

    state = state_opt.value()
    assert list(state.observation.get_buffer_d())

    observation = list(state.observation.get_buffer_d())
    assert len(observation) == 4

    # Try to register another environment
    assert factory.register_plugin(md)

    # Try to get another environment
    env2 = factory.make("CartPole")
    assert env2, "Failed to create CartPoleIgnition environment from the factory"
    assert env != env2, "Environment created from the factory are the same"
    env2.reset()
    env2.step(action)
