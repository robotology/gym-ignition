# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
from . import utils
from .utils import gazebo_fixture as gazebo
from gym_ignition import scenario_bindings as bindings

# Set the verbosity
bindings.setVerbosity(4)


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_load_default_world(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()
    world = gazebo.getWorld()

    assert world.id() != 0
    assert world.time() == 0.0
    assert world.name() == "default"


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def _test_load_default_world_from_file(gazebo: bindings.GazeboSimulator):

    empty_world_sdf = utils.get_empty_world_sdf()
    print(empty_world_sdf)
    assert gazebo.insertWorldFromSDF(empty_world_sdf)

    assert gazebo.initialize()
    world = gazebo.getWorld()

    assert world.id() != 0
    assert world.time() == 0.0
    assert world.name() == "default"


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def _test_rename_default_world(gazebo: bindings.GazeboSimulator):

    empty_world_sdf = utils.get_empty_world_sdf()
    assert gazebo.insertWorldFromSDF(empty_world_sdf, "myWorld")

    assert gazebo.initialize()
    world = gazebo.getWorld()

    assert world.id() != 0
    assert world.time() == 0.0
    assert world.name() == "myWorld"

    world1 = gazebo.getWorld("myWorld")

    assert world1.id() == world.id()
    assert world1.time() == 0.0
    assert world1.name() == "myWorld"


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_world_api(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()
    world = gazebo.getWorld()

    gravity = [0, 0, 10.0]
    assert world.setGravity(gravity)
    assert world.gravity() == pytest.approx(gravity)

    assert len(world.modelNames()) == 0

    # Insert a model from an non existent file
    assert not world.insertModel("")

    # Insert first cube with default name and pose
    cube_urdf = utils.get_cube_urdf()
    assert world.insertModel(cube_urdf)
    assert len(world.modelNames()) == 1

    default_model_name = bindings.getModelNameFromSdf(cube_urdf, 0)
    assert default_model_name in world.modelNames()
    cube1 = world.getModel(default_model_name)
    # TODO: assert cube1

    assert cube1.name() == default_model_name
    assert cube1.basePosition() == pytest.approx([0, 0, 0])
    assert cube1.baseOrientation() == pytest.approx([1, 0, 0, 0])

    # Inserting a model with the same name should fail
    assert not world.insertModel(cube_urdf)
    assert len(world.modelNames()) == 1

    # Insert second cube with custom name and pose
    custom_model_name = "other_cube"
    assert custom_model_name != default_model_name
    custom_model_pose = bindings.Pose([1, 1, 0], [0, 0, 0, 1])

    assert world.insertModel(cube_urdf, custom_model_pose, custom_model_name)
    assert custom_model_name in world.modelNames()
    assert len(world.modelNames()) == 2

    cube2 = world.getModel(custom_model_name)
    assert cube1 != cube2
    # TODO: assert cube2

    assert cube2.name() == custom_model_name
    assert cube2.basePosition() == pytest.approx(custom_model_pose.position)
    assert cube2.baseOrientation() == pytest.approx(custom_model_pose.orientation)

    # Remove the first model (requires either a paused or unpaused step)
    assert world.removeModel(default_model_name)
    assert len(world.modelNames()) == 2
    gazebo.run(paused=True)
    assert len(world.modelNames()) == 1

    # Without the physics system, the time should not increase
    gazebo.run()
    gazebo.run()
    gazebo.run()
    assert world.time() == 0.0


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_world_physics_plugin(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()
    world = gazebo.getWorld()

    dt = gazebo.stepSize()

    assert world.time() == 0

    # Insert a cube
    cube_urdf = utils.get_cube_urdf()
    cube_name = "my_cube"
    cube_pose = bindings.Pose([0, 0, 1.0], [1, 0, 0, 0])
    assert world.insertModel(cube_urdf, cube_pose, cube_name)
    assert cube_name in world.modelNames()

    cube = world.getModel(cube_name)

    # There's no physics, the cube should not move
    for _ in range(10):
        gazebo.run()
        assert cube.basePosition() == cube_pose.position

    assert world.time() == 0

    # Insert the Physics system
    world.insertWorldPlugin("libPhysicsSystem.so", "scenario::plugins::gazebo::Physics")

    # After the first step, the physics catches up with time
    gazebo.run()
    assert world.time() == pytest.approx(11 * dt)

    # The cube should start falling
    assert cube.basePosition()[2] < cube_pose.position[2]

    gazebo.run()
    assert world.time() == pytest.approx(12 * dt)

    # Paused steps do not increase the time
    gazebo.run(paused=True)
    assert world.time() == pytest.approx(12 * dt)


@pytest.mark.parametrize("gazebo",
                         [(0.001, 1.0, 1)],
                         indirect=True,
                         ids=utils.id_gazebo_fn)
def test_sim_time_starts_from_zero(gazebo: bindings.GazeboSimulator):

    assert gazebo.initialize()
    world = gazebo.getWorld()

    dt = gazebo.stepSize()

    assert world.time() == 0
    world.insertWorldPlugin("libPhysicsSystem.so", "scenario::plugins::gazebo::Physics")
    assert world.time() == 0

    gazebo.run(paused=True)
    assert world.time() == 0

    gazebo.run(paused=False)
    assert world.time() == dt

    gazebo.run(paused=False)
    assert world.time() == 2 * dt

    gazebo.run(paused=False)
    assert world.time() == 3 * dt
