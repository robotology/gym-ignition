# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.scenario

import gym_ignition_models

from scenario import gazebo as scenario

from ..common import utils
from ..common.utils import gazebo_fixture as gazebo

# Set the verbosity
scenario.set_verbosity(scenario.Verbosity_debug)


@pytest.mark.parametrize(
    "gazebo",
    [
        (0.001, 1.0, 1),
        (0.1, 5.0, 5),
        (0.001, 0.0, 1),
        (0.001, -1.0, 1),
        (0.001, 1.0, 0),
        (0, 1.0, 1),
    ],
    indirect=True,
    ids=utils.id_gazebo_fn,
)
def test_initialization(gazebo: scenario.GazeboSimulator):

    ok = gazebo.initialize()

    rtf = gazebo.real_time_factor()
    step_size = gazebo.step_size()
    iterations = gazebo.steps_per_run()

    if step_size <= 0:
        assert not ok
        assert not gazebo.initialized()
        assert not gazebo.run()

    if rtf <= 0:
        assert not ok
        assert not gazebo.initialized()
        assert not gazebo.run()

    if iterations <= 0:
        assert not ok
        assert not gazebo.initialized()
        assert not gazebo.run()

    if rtf > 0 and iterations > 0 and step_size > 0:
        assert ok
        assert gazebo.initialized()


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_run(gazebo: scenario.GazeboSimulator):

    assert gazebo.initialize()
    assert gazebo.run(paused=True)
    assert gazebo.run()


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_pause(gazebo: scenario.GazeboSimulator):

    assert gazebo.initialize()
    assert not gazebo.running()
    assert gazebo.pause()

    assert gazebo.initialize()

    assert not gazebo.running()
    assert gazebo.pause()

    assert gazebo.run(paused=True)
    assert not gazebo.running()
    assert gazebo.pause()

    assert gazebo.run(paused=False)
    assert not gazebo.running()
    assert gazebo.pause()


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_paused_step(gazebo: scenario.GazeboSimulator):

    assert gazebo.initialize()

    world = gazebo.get_world().to_gazebo()
    assert world.insert_model(gym_ignition_models.get_model_file("ground_plane"))
    assert "ground_plane" in world.model_names()
    gazebo.run(paused=True)

    world.remove_model("ground_plane")

    assert "ground_plane" in world.model_names()
    gazebo.run(paused=True)
    assert "ground_plane" not in world.model_names()
    assert world.time() == 0.0


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_load_default_world(gazebo: scenario.GazeboSimulator):

    assert gazebo.initialize()
    assert gazebo.world_names()
    assert len(gazebo.world_names()) == 1

    world1 = gazebo.get_world()
    assert world1
    assert world1.name() in gazebo.world_names()

    world2 = gazebo.get_world(gazebo.world_names()[0])
    assert world2

    assert world1.id() == world2.id()
    assert world1.name() == world2.name()

    # TODO: understand how to compare shared ptr returned by swig with nullptr
    # world3 = gazebo.get_world("foo")
    # assert world3
