# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.scenario

from pathlib import Path

from scenario import core
from scenario import gazebo as scenario

from ..common import utils
from ..common.utils import gazebo_fixture as gazebo

# Set the verbosity
scenario.set_verbosity(scenario.Verbosity_debug)


# See https://github.com/robotology/gym-ignition/pull/339#issuecomment-828300490
@pytest.mark.xfail(strict=False)
@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_download_model_from_fuel(gazebo: scenario.GazeboSimulator):

    assert gazebo.initialize()

    # Get the default world
    world = gazebo.get_world()

    # Download a model from Fuel (testing a name with spaces)
    model_name = "Electrical Box"
    model_sdf = scenario.get_model_file_from_fuel(
        f"https://fuel.ignitionrobotics.org/openrobotics/models/{model_name}", False
    )
    assert model_sdf

    assert world.insert_model(model_sdf, core.Pose_identity())
    assert model_name in world.model_names()

    # Insert another model changing its name
    other_model_name = "my_box"
    other_model_pose = core.Pose([3.0, 0.0, 0.0], [1.0, 0, 0, 0])
    assert world.insert_model(model_sdf, other_model_pose, other_model_name)
    assert other_model_name in world.model_names()

    assert gazebo.run()


@pytest.mark.parametrize(
    "gazebo", [(0.001, 1.0, 1)], indirect=True, ids=utils.id_gazebo_fn
)
def test_fuel_world(gazebo: scenario.GazeboSimulator):
    # (setup) load a world that includes a fuel model
    worlds_folder = Path(__file__) / ".." / ".." / "assets" / "worlds"
    world_file = worlds_folder / "fuel_support.sdf"
    assert gazebo.insert_world_from_sdf(str(world_file.resolve()))
    assert gazebo.initialize()
    assert gazebo.run(paused=True)

    # the actual test
    assert "ground_plane" in gazebo.get_world().model_names()
