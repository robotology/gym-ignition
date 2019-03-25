#!/usr/bin/env python3

# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
import gympp
from math import pi
from pathlib import Path
import os

@pytest.fixture
def create_std_vector():
    python_list = [1.0, 2.0, 3.0]
    vector = gympp.Vector_d(python_list)
    return vector

def test_vectors():
    python_list = [1.0, 2.0, 3.0]
    vector = gympp.Vector_d(python_list)
    for i in range(0, vector.size()-1):
        assert python_list[i] == vector[i], "Vectors do not match"

def test_sample(create_std_vector):
    vector = create_std_vector
    sample = gympp.Sample(vector)

    for i in range(0, vector.size()-1):
        assert sample.getBuffer_d()[i] == vector[i], "Sample object does not contain correct data"

    sample.getBuffer_d()[2] = 42
    assert sample.get_d(2).value() == 42, "Failed to insert data in the Sample object"
    assert sample.getBuffer_d()[2] == 42, "Failed to update data of a Sample object"

def test_range():
    gympp_range = gympp.RangeDouble(-5.0, 10.0)
    assert gympp_range.contains(0), "Range object failed to verify if it containes a value"
    assert gympp_range.contains(-5), "Range object failed to verify if it containes a value"
    assert gympp_range.contains(10), "Range object failed to verify if it containes a value"
    assert not gympp_range.contains(-6), "Range object failed to verify if it containes a value"
    assert not gympp_range.contains(10.1), "Range object failed to verify if it containes a value"
    
def test_discrete_space():
    discrete_space = gympp.Discrete(12)

    assert not discrete_space.contains(gympp.Sample([-1])), "Discrete object failed to verify if a sample belongs to its space"
    assert not discrete_space.contains(gympp.Sample([13])), "Discrete object failed to verify if a sample belongs to its space"

    for n in range(50):
        sample = discrete_space.sample()
        assert sample.getBuffer_i().size() == 1, "Wrong size of the sample extracted from a discrete space"
        assert type(sample.getBuffer_i()[0]) is int, "Wrong data type of the sample extracted from a discrete space"
        assert discrete_space.contains(sample), "Sampled data is not contained in the discrete space object that created it"

def test_box_space():
    size = 4
    box = gympp.Box_d(-1, 42, [size])

    # By default the data precision of python list is float. Force double.
    assert box.contains(gympp.Sample(gympp.Vector_d([0, pi, 12, 42]))), "Box object failed to verify if a sample belongs to its space"
    assert not box.contains(gympp.Sample(gympp.Vector_d([0, pi, 12, 43]))), "Box object failed to verify if a sample belongs to its space"
    assert not box.contains(gympp.Sample(gympp.Vector_d([0]))), "Box object failed to verify if a sample belongs to its space"
    assert not box.contains(gympp.Sample(gympp.Vector_d([0, pi, 12, 43, 0]))), "Box object failed to verify if a sample belongs to its space"

    for n in range(50):
        sample = box.sample()
        assert sample.getBuffer_d().size() == size, "Wrong size of the sample extracted from a box space"
        assert type(sample.getBuffer_d()[0]) is float, "Wrong data type of the sample extracted from the box space"
        assert box.contains(sample), "Sampled data is not contained in the box space object that created it"
    
    # TODO: test the other constructor version

def test_gymfactory():
    assert "IGN_GAZEBO_SYSTEM_PLUGIN_PATH" in os.environ, "Variable IGN_GAZEBO_SYSTEM_PLUGIN_PATH not set in the environment"
    directories = os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'].split(os.pathsep)
    
    found = False
    for directory in directories:
        matching_plugins = list(Path(directory).glob("*CartPole*"))
        found = found or (len(matching_plugins) is not 0)
    assert found, "Failed to find CartPole plugin"

    env = gympp.GymFactory_make("CartPole")
    assert env, "Failed to create CartPole environment from the factory"
    
    action = env.action_space.sample()
    state_opt = env.step(action)
    assert state_opt.has_value()
    
    state = state_opt.value()
    assert list(state.observation.getBuffer_d())
    
    observation = list(state.observation.getBuffer_d())
    assert len(observation) == 2
