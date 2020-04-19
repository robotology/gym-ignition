# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
from .. import utils
from ..utils import gazebo_fixture as gazebo
from gym_ignition import scenario_bindings as bindings

# Set the verbosity
bindings.setVerbosity(4)


@pytest.mark.parametrize("gazebo",
                         [
                             (0.001, 1.0, 1),
                             (0.1, 5.0, 5),
                             (0.001, 0.0, 1),
                             (0.001, -1.0, 1),
                             (0.001, 1.0, 0),
                             (0, 1.0, 1),
                         ], indirect=True, ids=utils.id_gazebo_fn)
def test_initialization(gazebo: bindings.GazeboSimulator):

    ok = gazebo.initialize()

    rtf = gazebo.realTimeFactor()
    step_size = gazebo.stepSize()
    iterations = gazebo.stepsPerRun()

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


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_run(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()
    assert gazebo.run(paused=True)
    assert gazebo.run()


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_pause(gazebo: bindings.GazeboSimulator):

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


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_load_default_world(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()
    assert gazebo.worldNames()
    assert len(gazebo.worldNames()) == 1

    world1 = gazebo.getWorld()
    assert world1
    assert world1.name() in gazebo.worldNames()

    world2 = gazebo.getWorld(gazebo.worldNames()[0])
    assert world2

    assert world1.id() == world2.id()
    assert world1.name() == world2.name()

    # TODO: understand how to compare shared ptr returned by swig with nullptr
    # world3 = gazebo.getWorld("foo")
    # assert world3
