# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest
pytestmark = pytest.mark.scenario

import numpy as np
from typing import Tuple
from ..common import utils
from scenario import core as scenario_core
from scenario import gazebo as scenario_gazebo
from gym_ignition.context import sensors, shapes
from ..common.utils import default_world_fixture as default_world

# Set the verbosity
scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_debug)


@pytest.mark.parametrize("default_world", [(1.0 / 1_000, 1.0, 1)], indirect=True)
def test_depth_camera(default_world: Tuple[scenario_gazebo.GazeboSimulator,
                                           scenario_gazebo.World]):

    # Get the simulator and the world
    gazebo, world = default_world

    # Insert a cube floating in the air
    cube_pose = scenario_core.Pose([0, 0, 2.], [1, 0, 0, 0])
    cube_shape = shapes.BoxURDF(mass=1.0, edge=0.1)
    cube_file = utils.misc.string_to_file(cube_shape.urdf())
    assert world.insert_model(cube_file, cube_pose)

    # Get the cube model
    assert cube_shape.name in world.model_names()
    cube = world.get_model(model_name=cube_shape.name)

    # Enable the sensors system
    assert world.enable_sensors(enable=True)

    # Configure depth camera context
    camera_name = "my_camera"
    camera_context = sensors.DepthCamera(name=camera_name)

    # Insert a down-facing camera above the cube
    camera_pose = scenario_core.Pose([0, 0, 3.], [0.707, 0, 0.707, 0])
    camera_file = utils.misc.string_to_file(camera_context.to_xml())
    assert world.insert_model(camera_file, camera_pose)

    # Get the model that contains the camera
    assert camera_context.model_name in world.model_names()
    camera_model = world.get_model(model_name=camera_context.model_name)
    assert camera_model.valid()

    # Getting the camera without a run (possibly paused) would run an exception.
    # The sensor has to be processed before being able to gather it.
    with pytest.raises(RuntimeError):
        _ = camera_model.to_gazebo().depth_camera(name=camera_context.name)

    # Trigger camera insertion with a paused step
    assert gazebo.run(paused=True)

    # Get the depth camera
    depth_camera = camera_model.to_gazebo().depth_camera(name=camera_context.name)

    # The camera will generate an image only after the first unpaused step
    assert depth_camera.image() == tuple()

    # Run the simulator, triggering the generation of an image
    assert gazebo.run()

    # Get the image
    img = depth_camera.image()
    assert len(img) == camera_context.height * camera_context.width

    # Reshape the image to a numpy matrix
    img = np.reshape(img, newshape=(camera_context.height, camera_context.width))

    # Check that the edges of the image are as distant as the ground plane...
    camera_z_position = camera_pose.position[2]
    assert img[0, 0] == pytest.approx(camera_z_position, abs=0.01)
    assert img[0, -1] == pytest.approx(camera_z_position, abs=0.01)
    assert img[-1, 0] == pytest.approx(camera_z_position, abs=0.01)
    assert img[-1, -1] == pytest.approx(camera_z_position, abs=0.01)

    # ... and the center as distant as the cube
    center = int(camera_context.height / 2), int(camera_context.width / 2)
    cube_upper_face_z = cube.base_position()[2] + cube_shape.edge / 2
    assert img[center] == pytest.approx(camera_z_position - cube_upper_face_z, abs=0.01)

    # Make the cube fall on the ground
    cube.enable_contacts(enable=True)

    # TODO: check efficiency, should it be almost one second? we got ~3 now w/o logs in release
    for _ in range(1_000):
        gazebo.run()

    # Check that the cube is on the ground
    assert len(cube.links_in_contact()) != 0

    # Get and reshape the image
    img = depth_camera.image()
    assert len(img) == camera_context.height * camera_context.width
    img = np.reshape(img, newshape=(camera_context.height, camera_context.width))

    # Check that the edges of the image are as distant as the ground plane...
    camera_z_position = camera_pose.position[2]
    assert img[0, 0] == pytest.approx(camera_z_position, abs=0.01)
    assert img[0, -1] == pytest.approx(camera_z_position, abs=0.01)
    assert img[-1, 0] == pytest.approx(camera_z_position, abs=0.01)
    assert img[-1, -1] == pytest.approx(camera_z_position, abs=0.01)

    # ... and the center as distant as the cube, this time not using the base position
    cube_upper_face_z = cube_shape.edge
    center = int(camera_context.height / 2), int(camera_context.width / 2)
    assert img[center] == pytest.approx(camera_z_position - cube_upper_face_z, abs=0.01)
