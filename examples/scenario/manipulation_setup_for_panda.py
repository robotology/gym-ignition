# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import numpy as np
import gym_ignition_models
from scenario import core
from scenario import gazebo as scenario
from gym_ignition.utils.scenario import init_gazebo_sim
from typing import List
from scipy.spatial.transform import Rotation


# Define a class to handle conversions from Rotation to Quaternions
class Quaternion(abc.ABC):

    @staticmethod
    def to_wxyz(xyzw: np.ndarray) -> np.ndarray:

        if xyzw.shape != (4,):
            raise ValueError(xyzw)

        return xyzw[[3, 0, 1, 2]]

    @staticmethod
    def to_xyzw(wxyz: np.ndarray) -> np.ndarray:

        if wxyz.shape != (4,):
            raise ValueError(wxyz)

        return wxyz[[1, 2, 3, 0]]

    @staticmethod
    def to_rotation(quaternion: np.ndarray) -> np.ndarray:

        if quaternion.shape != (4,):
            raise ValueError(quaternion)

        xyzw = Quaternion.to_xyzw(quaternion)

        return Rotation.from_quat(xyzw).as_matrix()


# Define a helper class to simplify model insertion.
class Panda(core.Model):

    def __init__(self,
                 world: scenario.World,
                 position: List[float] = (0., 0, 0),
                 orientation: List[float] = (1., 0, 0, 0)):

        # Get the model file
        urdf = gym_ignition_models.get_model_file("panda")

        # Insert the model in the world
        name = "panda_manipulator"
        pose = core.Pose(position, orientation)
        world.insert_model(urdf, pose, name)

        # Get and store the model from the world
        self.model = world.get_model(model_name=name)

    def __getattr__(self, name):
        return getattr(self.model, name)


# Set the verbosity
scenario.set_verbosity(scenario.Verbosity_warning)

# Get the default simulator and the default empty world
gazebo, world = init_gazebo_sim()

# Insert the table from fuel
# Notice that not all the models from fuel are rendered properly (for instance for "Table" no texture is shown)
model_name = "Cafe Table"
model_sdf = scenario.get_model_file_from_fuel(
    f"https://fuel.ignitionrobotics.org/openrobotics/models/{model_name}", False)
world.insert_model(model_sdf, core.Pose_identity())

# Insert a coke on the table
# Notice that not all the models from fuel are rendered properly (for instance for "Beer" no texture is shown)
model_name = "Coke"
model_sdf = scenario.get_model_file_from_fuel(
    f"https://fuel.ignitionrobotics.org/openrobotics/models/{model_name}", False)
beer_position = [0.3, -0.3, 0.76]
beer_quaternions = [1., 0, 0, 0]
beer_pose = core.Pose(beer_position, beer_quaternions)
world.insert_model(model_sdf, beer_pose)

# Insert a Panda using the class
panda_position = [-0.2, 0, 0.76]
panda_yaw = 0 # change this angle if you want to rotate the panda along the vertical axes
panda_quaternion = list(Quaternion.to_wxyz(Rotation.from_euler('z', panda_yaw).as_quat()))
panda = Panda(world=world, position=panda_position, orientation=panda_quaternion)

# Show the GUI
import time
gazebo.gui()
gazebo.run(paused=True)
time.sleep(3)

# peform 10 seconds of simulation with the Panda falling by effect of gravity
for i in range(10000):
    gazebo.run()

time.sleep(3)
