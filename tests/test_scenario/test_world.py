# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.scenario

from scenario import core
from scenario import gazebo as scenario

from ..common import utils
from ..common.utils import gazebo_fixture as gazebo

# Set the verbosity
scenario.set_verbosity(scenario.Verbosity_debug)


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_load_default_world(gazebo: scenario.GazeboSimulator):

    assert gazebo.initialize()
    world = gazebo.get_world()

    assert world.id() != 0
    assert world.time() == 0.0
    assert world.name() == "default"


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_load_default_world_from_file(gazebo: scenario.GazeboSimulator):

    empty_world_sdf = utils.get_empty_world_sdf()

    assert gazebo.insert_world_from_sdf(empty_world_sdf)

    assert gazebo.initialize()
    world = gazebo.get_world()

    assert world.id() != 0
    assert world.time() == 0.0
    assert world.name() == "default"


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_rename_default_world(gazebo: scenario.GazeboSimulator):

    empty_world_sdf = utils.get_empty_world_sdf()
    assert gazebo.insert_world_from_sdf(empty_world_sdf, "myWorld")

    assert gazebo.initialize()
    world = gazebo.get_world()

    assert world.id() != 0
    assert world.time() == 0.0
    assert world.name() == "myWorld"

    world1 = gazebo.get_world("myWorld")

    assert world1.id() == world.id()
    assert world1.time() == 0.0
    assert world1.name() == "myWorld"


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_world_api(gazebo: scenario.GazeboSimulator):

    assert gazebo.initialize()
    world = gazebo.get_world()

    gravity = [0, 0, 10.0]
    assert world.set_gravity(gravity)
    assert world.gravity() == pytest.approx(gravity)

    assert len(world.model_names()) == 0

    # Insert a model from an non existent file
    assert not world.insert_model("")

    # Insert first cube with default name and pose
    cube_urdf = utils.get_cube_urdf()
    assert world.insert_model(cube_urdf)
    assert len(world.model_names()) == 1

    default_model_name = scenario.get_model_name_from_sdf(cube_urdf)
    assert default_model_name in world.model_names()
    cube1 = world.get_model(default_model_name)
    # TODO: assert cube1

    assert cube1.name() == default_model_name
    assert cube1.base_position() == pytest.approx([0, 0, 0])
    assert cube1.base_orientation() == pytest.approx([1, 0, 0, 0])

    # Inserting a model with the same name should fail
    assert not world.insert_model(cube_urdf)
    assert len(world.model_names()) == 1

    # Insert second cube with custom name and pose
    custom_model_name = "other_cube"
    assert custom_model_name != default_model_name
    custom_model_pose = core.Pose([1, 1, 0], [0, 0, 0, 1])

    assert world.insert_model(cube_urdf, custom_model_pose, custom_model_name)
    assert custom_model_name in world.model_names()
    assert len(world.model_names()) == 2

    cube2 = world.get_model(custom_model_name)
    assert cube1 != cube2
    # TODO: assert cube2

    assert cube2.name() == custom_model_name
    assert cube2.base_position() == pytest.approx(custom_model_pose.position)
    assert cube2.base_orientation() == pytest.approx(custom_model_pose.orientation)

    # insert a cube from urdf string
    cube_3_pose = core.Pose([1, 0, 0], [0, 0, 0, 1])
    assert world.insert_model_from_string(
        utils.get_cube_urdf_string(), cube_3_pose, "cube3"
    )
    assert "cube3" in world.model_names()

    # insert a cube from sdf string
    cube_4_pose = core.Pose([2, 0, 0], [0, 0, 0, 1])
    assert world.insert_model_from_string(
        utils.get_cube_sdf_string(), cube_4_pose, "cube4"
    )
    assert "cube4" in world.model_names()

    # Remove the first model (requires either a paused or unpaused step)
    assert world.remove_model(default_model_name)
    assert len(world.model_names()) == 4
    gazebo.run(paused=True)
    assert len(world.model_names()) == 3

    # Without the physics system, the time should not increase
    gazebo.run()
    gazebo.run()
    gazebo.run()
    assert world.time() == 0.0


# See: https://github.com/robotology/gym-ignition/issues/366
@pytest.mark.xfail(strict=False)
@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_world_physics_plugin(gazebo: scenario.GazeboSimulator):

    assert gazebo.initialize()
    world = gazebo.get_world()

    dt = gazebo.step_size()

    assert world.time() == 0

    # Insert a cube
    cube_urdf = utils.get_cube_urdf()
    cube_name = "my_cube"
    cube_pose = core.Pose([0, 0, 1.0], [1, 0, 0, 0])
    assert world.insert_model(cube_urdf, cube_pose, cube_name)
    assert cube_name in world.model_names()

    cube = world.get_model(cube_name)

    # There's no physics, the cube should not move
    for _ in range(10):
        gazebo.run()
        assert cube.base_position() == cube_pose.position

    assert world.time() == 0

    # Insert the Physics system
    assert world.set_physics_engine(scenario.PhysicsEngine_dart)

    # After the first step, the physics catches up with time
    gazebo.run()
    assert world.time() == pytest.approx(11 * dt)

    # The cube should start falling
    assert cube.base_position()[2] < cube_pose.position[2]

    gazebo.run()
    assert world.time() == pytest.approx(12 * dt)

    # Paused steps do not increase the time
    gazebo.run(paused=True)
    assert world.time() == pytest.approx(12 * dt)


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1), (1e-9, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_sim_time_starts_from_zero(gazebo: scenario.GazeboSimulator):

    assert gazebo.initialize()
    world = gazebo.get_world()

    dt = gazebo.step_size()

    assert world.time() == 0
    assert world.set_physics_engine(scenario.PhysicsEngine_dart)
    assert world.time() == 0

    gazebo.run(paused=True)
    assert world.time() == 0

    gazebo.run(paused=False)
    assert world.time() == dt

    gazebo.run(paused=False)
    assert world.time() == 2 * dt

    gazebo.run(paused=False)
    assert world.time() == pytest.approx(3 * dt)
