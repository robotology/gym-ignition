# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
pytestmark = pytest.mark.scenario

import numpy as np
from typing import Tuple
from ..common import utils
import gym_ignition_models
from gym_ignition.utils import misc
from scenario import core as scenario_core
from scenario import gazebo as scenario_gazebo
from ..common.utils import gazebo_fixture as gazebo

# Set the verbosity
scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_debug)


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def _test_insert_multiple_worlds(gazebo: scenario_gazebo.GazeboSimulator):

    empty_world_sdf = utils.get_empty_world_sdf()
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "myWorld1")

    assert not gazebo.insert_world_from_sdf(empty_world_sdf, "myWorld1")
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "myWorld2")

    assert gazebo.initialize()
    assert "myWorld1" in gazebo.world_names()
    assert "myWorld2" in gazebo.world_names()

    my_world1 = gazebo.get_world("myWorld1")
    my_world2 = gazebo.get_world("myWorld2")

    assert my_world1.id() != my_world2.id()
    assert my_world1.name() == "myWorld1"
    assert my_world2.name() == "myWorld2"


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def _test_insert_multiple_world(gazebo: scenario_gazebo.GazeboSimulator):

    multi_world_sdf = utils.get_multi_world_sdf_file()

    assert gazebo.insert_worlds_from_sdf(multi_world_sdf)
    assert gazebo.initialize()

    assert "world1" in gazebo.world_names()
    assert "world2" in gazebo.world_names()

    world1 = gazebo.get_world("world1")
    world2 = gazebo.get_world("world2")

    assert world1.name() == "world1"
    assert world2.name() == "world2"

    assert world1.id() != world2.id()


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def _test_insert_multiple_world_rename(gazebo: scenario_gazebo.GazeboSimulator):

    multi_world_sdf = utils.get_multi_world_sdf_file()

    assert not gazebo.insert_worlds_from_sdf(multi_world_sdf, ["only_one_name"])
    assert gazebo.insert_worlds_from_sdf(multi_world_sdf, ["myWorld1", "myWorld2"])
    assert gazebo.initialize()

    assert "myWorld1" in gazebo.world_names()
    assert "myWorld2" in gazebo.world_names()

    world1 = gazebo.get_world("myWorld1")
    world2 = gazebo.get_world("myWorld2")

    assert world1.name() == "myWorld1"
    assert world2.name() == "myWorld2"

    assert world1.id() != world2.id()


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def _test_insert_world_multiple_calls(gazebo: scenario_gazebo.GazeboSimulator):

    single_world_sdf = utils.get_empty_world_sdf()

    assert gazebo.insert_world_from_sdf(single_world_sdf)
    assert not gazebo.insert_world_from_sdf(single_world_sdf)
    assert gazebo.insert_world_from_sdf(single_world_sdf, "world2")
    assert gazebo.initialize()

    assert "default" in gazebo.world_names()
    assert "world2" in gazebo.world_names()

    world1 = gazebo.get_world("default")
    world2 = gazebo.get_world("world2")

    assert world1.name() == "default"
    assert world2.name() == "world2"

    assert world1.id() != world2.id()


def multi_world_simulation_id(val):

    if isinstance(val, tuple) and len(val) == 3:
        return f"(step={val[0]}|rtf={val[1]}|iters={val[2]})"

    if isinstance(val, tuple) and len(val) ==2:
        return f"({val[0]}|{val[1]})"


# This test is flaky
# @pytest.mark.xfail(strict=False)
@pytest.mark.parametrize("gazebo, physics1, physics2",
                         [
                          # ODE / ODE
                          # ((0.001, 2.0, 1), ("ode", "dantzig"), ("ode", "dantzig")),
                          # ((0.001, 2.0, 1), ("ode", "pgs"), ("ode", "pgs")),
                          # ((0.001, 2.0, 1), ("ode", "dantzig"), ("ode", "pgs")),
                          # BULLET / BULLET
                          ((0.001, 2.0, 1), ("bullet", "pgs"), ("bullet", "pgs")),
                          # ((0.001, 2.0, 1), ("bullet", "dantzig"), ("bullet", "dantzig")),
                          # ((0.001, 2.0, 1), ("bullet", "pgs"), ("bullet", "dantzig")),
                          # FCL / FCL
                          # ((0.001, 2.0, 1), ("fcl", "dantzig"), ("fcl", "dantzig")),
                          # ((0.001, 2.0, 1), ("fcl", "pgs"), ("fcl", "pgs")),
                          # ((0.001, 2.0, 1), ("fcl", "dantzig"), ("fcl", "pgs")),
                          # DART / DART
                          # ((0.001, 2.0, 1), ("dart", "dantzig"), ("dart", "dantzig")),
                          # ((0.001, 2.0, 1), ("dart", "pgs"), ("dart", "pgs")),
                          # ((0.001, 2.0, 1), ("dart", "dantzig"), ("dart", "pgs")),
                          # ODE / BULLET
                          # ((0.001, 2.0, 1), ("ode", "dantzig"), ("bullet", "dantzig")),
                          # ((0.001, 2.0, 1), ("ode", "pgs"), ("bullet", "pgs")),
                          # ((0.001, 2.0, 1), ("ode", "dantzig"), ("bullet", "pgs")),
                          ],
                         indirect=["gazebo"],
                         ids=multi_world_simulation_id)
def test_multi_world_simulation(gazebo: scenario_gazebo.GazeboSimulator,
                                physics1: Tuple[str, str],
                                physics2: Tuple[str, str]):

    detector1, solver1 = physics1
    detector2, solver2 = physics2

    # Empty DART world with configurable collision detector and solver
    get_world_sdf_string = lambda detector, solver: f"""
    <?xml version="1.0" ?>
    <sdf version="1.7">
        <world name="default">
            <physics default="true" type="dart">
                <dart>
                    <collision_detector>{detector}</collision_detector>
                    <solver>
                        <solver_type>{solver}</solver_type>
                    </solver>
                </dart>
            </physics>
        </world>
    </sdf>
    """

    # Create tmp files from the SDF string
    world1_sdf_file = misc.string_to_file(string=get_world_sdf_string(detector1, solver1))
    world2_sdf_file = misc.string_to_file(string=get_world_sdf_string(detector2, solver2))

    # Load two different worlds
    assert gazebo.insert_world_from_sdf(world1_sdf_file, "dart1")
    assert gazebo.insert_world_from_sdf(world2_sdf_file, "dart2")

    # Initialize the simulator
    assert gazebo.initialize()

    # gazebo.gui()
    # import time
    # time.sleep(1)
    # gazebo.run(paused=True)

    sphere_urdf_string = utils.SphereURDF(restitution=0.8).urdf()
    sphere_urdf = misc.string_to_file(sphere_urdf_string)
    ground_plane_urdf = gym_ignition_models.get_model_file(robot_name="ground_plane")

    # Pose of the sphere
    sphere_pose = scenario_core.Pose([0, 0, 0.5], [1, 0, 0, 0])

    # Populate the first DART world
    world1 = gazebo.get_world("dart1")
    world1.set_physics_engine(scenario_gazebo.PhysicsEngine_dart)
    world1.insert_model(ground_plane_urdf)
    world1.insert_model(sphere_urdf, sphere_pose)
    sphere1 = world1.get_model(model_name="sphere_model")

    # Populate the second DART world
    world2 = gazebo.get_world("dart2")
    world2.set_physics_engine(scenario_gazebo.PhysicsEngine_dart)
    world2.insert_model(ground_plane_urdf)
    world2.insert_model(sphere_urdf, sphere_pose)
    sphere2 = world2.get_model(model_name="sphere_model")

    # Integration time
    dt = gazebo.step_size() * gazebo.steps_per_run()

    # Enable contacts
    sphere1.enable_contacts(True)
    sphere2.enable_contacts(True)

    for _ in np.arange(start=0.0, stop=5.0, step=dt):

        # Run the simulator
        assert gazebo.run()

        # Check that both worlds evolve similarly
        assert sphere1.base_position() == \
               pytest.approx(sphere1.base_position(), abs=0.001)
