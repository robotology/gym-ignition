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

# Set the controller period
controller_period = 0.001
panda.model.set_controller_period(controller_period)

# List the controlled joints
joints = panda.model.joint_names()

# Switch the controlled joints in position mode
panda.model.set_joint_control_mode(scenario.JointControlMode_position, list(joints))

# Define PID gains (from https://github.com/mkrizmancic/franka_gazebo/commit/4a88ce6ca7d3d7bb87bd3fbe91e12873cc42f9b9)
panda_pid_gains = {
    'panda_joint1': {'p': 50, 'i': 0, 'd': 20},
    'panda_joint2': {'p': 10000, 'i': 0, 'd': 500},
    'panda_joint3': {'p': 100, 'i': 0, 'd': 10},
    'panda_joint4': {'p': 1000, 'i': 0, 'd': 50},
    'panda_joint5': {'p': 100, 'i': 0, 'd': 10},
    'panda_joint6': {'p': 100, 'i': 0, 'd': 10},
    'panda_joint7': {'p': 10, 'i': 0.5, 'd': 0.1},
    'panda_finger_joint1': {'p': 100, 'i': 0, 'd': 50},
    'panda_finger_joint2': {'p': 100, 'i': 0, 'd': 50},
    }

# Configure the PIDs
for joint_name in joints:
    pid = scenario.PID()
    pid_gains = panda_pid_gains[joint_name]
    pid.p, pid.i, pid.d = pid_gains['p'], pid_gains['i'], pid_gains['d']
    panda.model.get_joint(joint_name).set_pid(pid)

# Insert the joint controller plugin
ok_controller = panda.model.insert_model_plugin(
    "libJointController.so", "scenario::plugins::gazebo::JointController")

# Set the references
panda.model.set_joint_position_targets([0.0] * panda.model.dofs())

# List the joints to reset
joints_no_fingers = [j for j in panda.model.joint_names() if j.startswith("panda_joint")]
nr_of_joints = len(joints_no_fingers)

# Reset the listed joints
q0 = [np.deg2rad(15)] * nr_of_joints
dq0 = [0.05] * nr_of_joints
panda.model.reset_joint_positions(q0, joints_no_fingers)
panda.model.reset_joint_velocities(dq0, joints_no_fingers)

# Step the simulator for a couple of seconds with the Panda reaching the reference positions from the initial state
for _ in range(5000):
    gazebo.run()

# Apply an external force
panda.model.get_link("panda_link4").apply_world_force([400.0, 0, 0], 0.8)

# Step the simulator for a couple of seconds with the Panda recovering the reference positions
for _ in range(5000):
    assert gazebo.run()

# Apply another external force
panda.model.get_link("panda_link4").apply_world_force([0, -400, 0], 0.8)

# Step the simulator for a couple of seconds with the Panda recovering the reference positions
for _ in range(5000):
    assert gazebo.run()

# Change the reference positions
panda.model.set_joint_position_targets([0.1] * panda.model.dofs())

# Step the simulator for a couple of seconds with the Panda reaching the new reference positions
for _ in range(5000):
    assert gazebo.run()

time.sleep(3)