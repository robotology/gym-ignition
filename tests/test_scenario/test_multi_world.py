# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.scenario

import gym_ignition_models
import numpy as np
from gym_ignition.utils import misc

from scenario import core as scenario_core
from scenario import gazebo as scenario_gazebo

from ..common import utils
from ..common.utils import gazebo_fixture as gazebo

# Set the verbosity
scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_debug)


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_insert_multiple_worlds(gazebo: scenario_gazebo.GazeboSimulator):

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


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_insert_multiple_world(gazebo: scenario_gazebo.GazeboSimulator):

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


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_insert_multiple_world_rename(gazebo: scenario_gazebo.GazeboSimulator):

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


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_insert_world_multiple_calls(gazebo: scenario_gazebo.GazeboSimulator):

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


# # This test is flaky
# @pytest.mark.xfail(strict=False)
# @pytest.mark.parametrize("gazebo, solver",
#                          [((0.001, 2.0, 1), "pgs"),
#                           ((0.001, 2.0, 1), "dantzig")],
#                          indirect=["gazebo"],
#                          ids=utils.id_gazebo_fn)
# def test_multi_world_simulation(gazebo: scenario_gazebo.GazeboSimulator,
#                                 solver: str):
#
#     # Empty DART world with bullet as collision detector.
#     # It should prevent ODE crashes in a multi-world setting due to its static variables.
#     world_sdf_string = f"""
#     <?xml version="1.0" ?>
#     <sdf version="1.7">
#         <world name="default">
#             <physics default="true" type="dart">
#                 <dart>
#                     <collision_detector>bullet</collision_detector>
#                     <solver>
#                         <solver_type>{solver}</solver_type>
#                     </solver>
#                 </dart>
#             </physics>
#             <light type="directional" name="sun">
#                 <cast_shadows>true</cast_shadows>
#                 <pose>0 0 10 0 0 0</pose>
#                 <diffuse>1 1 1 1</diffuse>
#                 <specular>0.5 0.5 0.5 1</specular>
#                 <attenuation>
#                     <range>1000</range>
#                     <constant>0.9</constant>
#                     <linear>0.01</linear>
#                     <quadratic>0.001</quadratic>
#                 </attenuation>
#                 <direction>-0.5 0.1 -0.9</direction>
#             </light>
#         </world>
#     </sdf>
#     """
#
#     # Create a tmp file from the SDF string
#     world_sdf_file = misc.string_to_file(string=world_sdf_string)
#
#     # Load two different worlds
#     assert gazebo.insert_world_from_sdf(world_sdf_file, "dart1")
#     assert gazebo.insert_world_from_sdf(world_sdf_file, "dart2")
#
#     # Initialize the simulator
#     assert gazebo.initialize()
#
#     # gazebo.gui()
#     # import time
#     # time.sleep(1)
#     # gazebo.run(paused=True)
#
#     sphere_urdf_string = utils.SphereURDF(restitution=0.8).urdf()
#     sphere_urdf = misc.string_to_file(sphere_urdf_string)
#     ground_plane_urdf = gym_ignition_models.get_model_file(robot_name="ground_plane")
#
#     # Pose of the sphere
#     sphere_pose = scenario_core.Pose([0, 0, 0.5], [1, 0, 0, 0])
#
#     # Populate the first DART world
#     world1 = gazebo.get_world("dart1")
#     world1.set_physics_engine(scenario_gazebo.PhysicsEngine_dart)
#     world1.insert_model(ground_plane_urdf)
#     world1.insert_model(sphere_urdf, sphere_pose)
#     sphere1 = world1.get_model(model_name="sphere_model")
#
#     # Populate the second DART world
#     world2 = gazebo.get_world("dart2")
#     world2.set_physics_engine(scenario_gazebo.PhysicsEngine_dart)
#     world2.insert_model(ground_plane_urdf)
#     world2.insert_model(sphere_urdf, sphere_pose)
#     sphere2 = world2.get_model(model_name="sphere_model")
#
#     # Integration time
#     dt = gazebo.step_size() * gazebo.steps_per_run()
#
#     # Enable contacts
#     sphere1.enable_contacts(True)
#     sphere2.enable_contacts(True)
#
#     for _ in np.arange(start=0.0, stop=5.0, step=dt):
#
#         # Run the simulator
#         assert gazebo.run()
#
#         # Check that both worlds evolve similarly
#         assert sphere1.base_position() == \
#                pytest.approx(sphere2.base_position(), abs=0.001)
#         assert sphere1.base_world_linear_velocity() == \
#                pytest.approx(sphere2.base_world_linear_velocity(), abs=0.001)
#         assert sphere1.base_world_angular_velocity() == \
#                pytest.approx(sphere2.base_world_angular_velocity())
