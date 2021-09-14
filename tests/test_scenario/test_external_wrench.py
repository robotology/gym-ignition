# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.scenario

from typing import Tuple

import numpy as np

from scenario import core
from scenario import gazebo as scenario

from ..common import utils
from ..common.utils import default_world_fixture as default_world

# Set the verbosity
scenario.set_verbosity(scenario.Verbosity_debug)


@pytest.mark.parametrize("default_world", [(1.0 / 1_000, 1.0, 1)], indirect=True)
def test_fixed_base(default_world: Tuple[scenario.GazeboSimulator, scenario.World]):

    # Get the simulator and the world
    gazebo, world = default_world

    # Insert a sphere
    pose = core.Pose([0, 0.0, 1.0], [1.0, 0, 0, 0])
    assert world.insert_model_from_string(
        utils.SphereURDF(mass=10.0).urdf(), pose, "sphere"
    )
    assert "sphere" in world.model_names()

    # Get the sphere
    sphere = world.get_model("sphere")
    link = sphere.get_link("sphere").to_gazebo()

    # Get the initial z position
    initial_height = link.position()[2]

    # Apply a vertical force that counterbalances gravity
    assert link.apply_world_force(
        force=-np.array(world.gravity()) * sphere.total_mass(), duration=0.5
    )

    # Run the simulation for a while
    for _ in range(500):
        gazebo.run()
        assert link.position()[2] == pytest.approx(initial_height)

    # Run the simulation for a while
    for _ in range(500):
        gazebo.run()
        assert link.position()[2] < initial_height
