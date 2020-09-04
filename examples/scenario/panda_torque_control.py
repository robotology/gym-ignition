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
from gym_ignition.controllers.gazebo import computed_torque_fixed_base as context


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

# Insert a Panda using the class
panda_position = [0, 0, 0]
panda_yaw = 0 # change this angle if you want to rotate the panda along the vertical axes
panda_quaternion = list(Quaternion.to_wxyz(Rotation.from_euler('z', panda_yaw).as_quat()))
panda = Panda(world=world, position=panda_position, orientation=panda_quaternion)

# Show the GUI
import time
gazebo.gui()
gazebo.run(paused=True)
time.sleep(3)

# Disable self-collisions
panda.enable_self_collisions(False)

# Set the controller period
controller_period = 0.001
panda.set_controller_period(controller_period)

# Create the controller context
controller_context = context.ComputedTorqueFixedBaseContext(
    name="ComputedTorqueFixedBase",
    kp=[10.0] * panda.dofs(),
    ki=[0.0] * panda.dofs(),
    kd=[8.0] * panda.dofs(),
    urdf=gym_ignition_models.get_model_file("panda"),
    joints=panda.joint_names(),
    gravity=world.gravity())

# Insert the controller
panda.to_gazebo().insert_model_plugin("ControllerRunner",
                                      "scenario::plugins::gazebo::ControllerRunner",
                                      controller_context.to_xml())

# Set the references
panda.set_joint_position_targets([0.0] * panda.dofs())
panda.set_joint_velocity_targets([0.0] * panda.dofs())
panda.set_joint_acceleration_targets([0.0] * panda.dofs())

# List the controlled joints
joints_no_fingers = [j for j in panda.joint_names() if j.startswith("panda_joint")]
nr_of_joints = len(joints_no_fingers)

# Reset the joints state
q0 = [np.deg2rad(45)] * nr_of_joints
dq0 = [0.1] * nr_of_joints
panda.to_gazebo().reset_joint_positions(q0, joints_no_fingers)
panda.to_gazebo().reset_joint_velocities(dq0, joints_no_fingers)

# Step the simulator for a couple of seconds with the Panda reaching the reference positions from the initial state
for _ in range(5000):
    gazebo.run()

# Apply an external force
panda.get_link("panda_link4").apply_world_force([100.0, 0, 0], 0.5)

# Step the simulator for a couple of seconds with the Panda recovering the reference positions
for _ in range(5000):
    assert gazebo.run()

# Apply another external force
panda.get_link("panda_link4").apply_world_force([0, -200, 0], 0.8)

# Step the simulator for a couple of seconds with the Panda recovering the reference positions
for _ in range(5000):
    assert gazebo.run()

# Change the reference positions
panda.set_joint_position_targets([0.5] * panda.dofs())

# Step the simulator for a couple of seconds with the Panda reaching the new reference positions
for _ in range(5000):
    assert gazebo.run()

time.sleep(3)