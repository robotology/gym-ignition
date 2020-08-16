# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
pytestmark = pytest.mark.gympp

import numpy as np
from numpy import pi
from scenario import gazebo as scenario


try:
    import gympp_bindings as bindings
except ImportError:
    pytest.skip("gympp bindings not found", allow_module_level=True)


# Set verbosity
scenario.set_verbosity(4)


@pytest.fixture
def create_std_vector():

    python_list = [1.0, 2.0, 3.0]
    vector = bindings.VectorD(python_list)

    return vector


@pytest.fixture
def create_space_box_md():

    md = bindings.SpaceMetadata()
    md.set_type(bindings.SpaceType_box)
    max_float = float(np.finfo(np.float32).max)
    md.set_low_limit([-2.5, -max_float, -24, -max_float])
    md.set_high_limit([2.5, max_float, 24, max_float])

    return md


@pytest.fixture
def create_space_discrete_md():

    md = bindings.SpaceMetadata()
    md.set_type(bindings.SpaceType_discrete)
    md.set_dimensions([2])

    return md


def test_vectors():

    python_list = [1.0, 2.0, 3.0]
    vector = bindings.VectorD(python_list)

    for i in range(0, vector.size()-1):
        assert python_list[i] == vector[i]


def test_sample(create_std_vector):

    vector = create_std_vector
    sample = bindings.Sample(vector)

    for i in range(0, vector.size()-1):
        assert sample.get_buffer_d()[i] == vector[i]

    sample.get_buffer_d()[2] = 42
    assert sample.get_d(2).value() == 42
    assert sample.get_buffer_d()[2] == 42


def test_range():

    gympp_range = bindings.Range(-5.0, 10.0)

    assert gympp_range.contains(0)
    assert gympp_range.contains(-5)
    assert gympp_range.contains(10)
    assert not gympp_range.contains(-6)
    assert not gympp_range.contains(10.1)


def test_discrete_space():

    discrete_space = bindings.Discrete(12)

    assert not discrete_space.contains(bindings.Sample([-1]))
    assert not discrete_space.contains(bindings.Sample([13]))

    for n in range(50):
        sample = discrete_space.sample()
        assert sample.get_buffer_i().size() == 1
        assert isinstance(sample.get_buffer_i()[0], int)
        assert discrete_space.contains(sample)


def test_box_space():

    size = 4
    box = bindings.Box(-1, 42, [size])

    # By default the data precision of python list is float. Force double.
    assert box.contains(bindings.Sample(bindings.VectorD([0, pi, 12, 42])))
    assert not box.contains(bindings.Sample(bindings.VectorD([0, pi, 12, 43])))
    assert not box.contains(bindings.Sample(bindings.VectorD([0])))
    assert not box.contains(bindings.Sample(bindings.VectorD([0, pi, 12, 43, 0])))

    for n in range(50):

        sample = box.sample()

        assert sample.get_buffer_d().size() == size
        assert isinstance(sample.get_buffer_d()[0], float)
        assert box.contains(sample)


def test_space_box_metadata(create_space_box_md):

    md = create_space_box_md

    assert md.is_valid()
    assert md.get_type() == bindings.SpaceType_box


def test_space_discrete_metadata(create_space_discrete_md):

    md = create_space_discrete_md

    assert md.is_valid()
    assert md.get_type() == bindings.SpaceType_discrete
