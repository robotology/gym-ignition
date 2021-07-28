# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import tempfile

import numpy as np

from . import utils


def test_contacts():
    # Get the simulator
    gazebo = utils.Gazebo(physics_rate=1000, iterations=1, rtf=100)

    # Create the first cube and insert it in the simulation
    cube1 = utils.CubeGazeboRobot(
        gazebo=gazebo.simulator, initial_position=np.array([0, 0, 1.0])
    )

    # Create the second cube and insert it in the simulation
    cube2 = utils.CubeGazeboRobot(
        gazebo=gazebo.simulator, initial_position=np.array([0, 0, 2.5])
    )

    # Execute the first simulation step
    gazebo.step()

    # The cubes should be falling without contacts
    assert len(cube1.links_in_contact()) == 0
    assert len(cube2.links_in_contact()) == 0

    # Perform 500 steps.
    for _ in range(500):
        gazebo.step()

    # Cube1 should be touching ground
    assert len(cube1.links_in_contact()) == 1
    assert cube1.links_in_contact()[0] == "cube"
    contact_data1 = cube1.contact_data("cube")
    assert len(contact_data1) > 0
    assert contact_data1[0].bodyA == cube1.name() + "::cube_collision"
    assert contact_data1[0].bodyB == "ground_plane::collision"

    # Cube 2 should be still floating
    assert len(cube2.links_in_contact()) == 0
    assert len(cube2.contact_data("cube")) == 0

    # Perform 500 steps.
    for _ in range(500):
        gazebo.step()

    # Now cube2 should be in contact with cube1
    assert len(cube2.links_in_contact()) == 1
    assert cube2.links_in_contact()[0] == "cube"
    contact_data2 = cube2.contact_data("cube")
    assert len(contact_data2) > 0
    assert contact_data2[0].bodyA == cube2.name() + "::cube_collision"
    assert contact_data2[0].bodyB == cube1.name() + "::cube_collision"

    # And cube1 should be in contact with cube2 and ground_plane
    assert len(cube1.links_in_contact()) == 1
    assert cube1.links_in_contact()[0] == "cube"
    contact_data1 = cube1.contact_data("cube")
    assert len(contact_data1) == 2

    for contact in contact_data1:
        assert contact.bodyA == cube1.name() + "::cube_collision"
        assert (
            contact.bodyB == cube2.name() + "::cube_collision"
            or contact.bodyB == "ground_plane::collision"
        )
