# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
pytestmark = pytest.mark.scenario

from ..common import utils
from ..common.utils import gazebo_fixture as gazebo
from gym_ignition import scenario_bindings as bindings

# Set the verbosity
bindings.setVerbosity(4)


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_insert_multiple_worlds(gazebo: bindings.GazeboSimulator):

    empty_world_sdf = utils.get_empty_world_sdf()
    assert gazebo.insertWorldFromSDF(empty_world_sdf, "myWorld1")

    assert not gazebo.insertWorldFromSDF(empty_world_sdf, "myWorld1")
    assert gazebo.insertWorldFromSDF(empty_world_sdf, "myWorld2")

    assert gazebo.initialize()
    assert "myWorld1" in gazebo.worldNames()
    assert "myWorld2" in gazebo.worldNames()

    my_world1 = gazebo.getWorld("myWorld1")
    my_world2 = gazebo.getWorld("myWorld2")

    assert my_world1.id() != my_world2.id()
    assert my_world1.name() == "myWorld1"
    assert my_world2.name() == "myWorld2"


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_insert_multiple_world(gazebo: bindings.GazeboSimulator):

    multi_world_sdf = utils.get_multi_world_sdf_file()

    assert gazebo.insertWorldsFromSDF(multi_world_sdf)
    assert gazebo.initialize()

    assert "world1" in gazebo.worldNames()
    assert "world2" in gazebo.worldNames()

    world1 = gazebo.getWorld("world1")
    world2 = gazebo.getWorld("world2")

    assert world1.name() == "world1"
    assert world2.name() == "world2"

    assert world1.id() != world2.id()


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_insert_multiple_world_rename(gazebo: bindings.GazeboSimulator):

    multi_world_sdf = utils.get_multi_world_sdf_file()

    assert not gazebo.insertWorldsFromSDF(multi_world_sdf, ["only_one_name"])
    assert gazebo.insertWorldsFromSDF(multi_world_sdf, ["myWorld1", "myWorld2"])
    assert gazebo.initialize()

    assert "myWorld1" in gazebo.worldNames()
    assert "myWorld2" in gazebo.worldNames()

    world1 = gazebo.getWorld("myWorld1")
    world2 = gazebo.getWorld("myWorld2")

    assert world1.name() == "myWorld1"
    assert world2.name() == "myWorld2"

    assert world1.id() != world2.id()


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_insert_world_multiple_calls(gazebo: bindings.GazeboSimulator):

    single_world_sdf = utils.get_empty_world_sdf()

    assert gazebo.insertWorldFromSDF(single_world_sdf)
    assert not gazebo.insertWorldFromSDF(single_world_sdf)
    assert gazebo.insertWorldFromSDF(single_world_sdf, "world2")
    assert gazebo.initialize()

    assert "default" in gazebo.worldNames()
    assert "world2" in gazebo.worldNames()

    world1 = gazebo.getWorld("default")
    world2 = gazebo.getWorld("world2")

    assert world1.name() == "default"
    assert world2.name() == "world2"

    assert world1.id() != world2.id()
