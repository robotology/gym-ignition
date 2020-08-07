# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import numpy as np
import gym_ignition_models
from gym_ignition import scenario_bindings as scenario
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
class Panda(scenario.Model):

    def __init__(self,
                 world: scenario.World,
                 position: List[float] = (0., 0, 0),
                 orientation: List[float] = (1., 0, 0, 0)):
        # Initialize the base class
        super().__init__()

        # Get the model file
        urdf = gym_ignition_models.get_model_file("panda")

        # Insert the model in the world
        name = "panda_manipulator"
        pose = scenario.Pose(position, orientation)
        world.insert_model(urdf, pose, name)

        # Get and store the model from the world
        self.model = world.get_model(model_name=name)

    def __getattr__(self, name):
        print("getting", name)
        return getattr(self.model, name)


# Set the verbosity
scenario.set_verbosity(level=2)

# Get the default simulator and the default empty world
gazebo, world = init_gazebo_sim()

# Insert a Panda using the class
panda_position = [0, 0, 0]
panda_yaw = np.pi
panda_quaternion = list(Quaternion.to_wxyz(Rotation.from_euler('z', panda_yaw).as_quat()))
panda = Panda(world=world, position=panda_position, orientation=panda_quaternion)

# Show the GUI
import time
gazebo.gui()
gazebo.run(paused=True)
time.sleep(3)

# Disable self-collisions
panda.model.enable_self_collisions(False)

# List the joints to reset
joints_no_fingers = [j for j in panda.model.joint_names() if j.startswith("panda_joint")]
nr_of_joints = len(joints_no_fingers)

# Step the simulator for a couple of seconds with the Panda falling under gravity
for _ in range(1000):
    gazebo.run()

# Reset the listed joints at the initial pose
q0 = [np.deg2rad(0)] * nr_of_joints
dq0 = [0] * nr_of_joints
panda.model.reset_joint_positions(q0, joints_no_fingers)
panda.model.reset_joint_velocities(dq0, joints_no_fingers)

# Step the simulator for a couple of seconds with the Panda falling under gravity
for _ in range(1000):
    gazebo.run()

# Reset the listed joints at different positions
q0 = np.deg2rad([45, -30, 0, -60, 50, 90, 0])
dq0 = [0] * nr_of_joints
panda.model.reset_joint_positions(q0, joints_no_fingers)
panda.model.reset_joint_velocities(dq0, joints_no_fingers)

# Step the simulator for a couple of seconds with the Panda falling under gravity
for _ in range(1000):
    gazebo.run()

time.sleep(3)