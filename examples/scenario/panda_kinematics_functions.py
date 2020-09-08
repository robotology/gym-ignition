# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import math
import numpy as np
import gym_ignition_models
from scenario import core
from scenario import gazebo as scenario
from gym_ignition.utils.scenario import init_gazebo_sim
from typing import List
from scipy.spatial.transform import Rotation
from typing import Tuple
from gym_ignition.utils.inverse_kinematics_nlp import TargetType
from gym_ignition.utils.inverse_kinematics_nlp import InverseKinematicsNLP
from gym_ignition.controllers.gazebo import computed_torque_fixed_base as context



############################################################################################## USER INPUT

# Decide whether to use a torque controller (torque_control=True) or just reset the joints (torque_control=False)
# in order to track the Cartesian reference for the end-effector of the panda robot. The torque controller will track
# the desired trajectory at a lower frequency. Directly resetting the joints of the robot, i.e. ignoring physics
# simulation, is faster and testifies alone the goodness of the solution returned by the inverse kinematics class.
torque_control = True



############################################################################################## AUXILIARY FUNCTIONS

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


# Define a class to handle transformations
class Transform(abc.ABC):

    @staticmethod
    def from_position_and_quaternion(position: np.ndarray,
                                     quaternion: np.ndarray) -> np.ndarray:

        if quaternion.size != 4:
            raise ValueError("Quaternion array must have 4 elements")

        rotation = Quaternion.to_rotation(quaternion)
        transform = Transform.from_position_and_rotation(position=position,
                                                         rotation=rotation)

        return transform

    @staticmethod
    def from_position_and_rotation(position: np.ndarray,
                                   rotation: np.ndarray) -> np.ndarray:

        if position.size != 3:
            raise ValueError("Position array must have 3 elements")

        if rotation.shape != (3, 3):
            raise ValueError("Rotation must be a square 3x3 matrix")

        transform = np.eye(4)
        transform[0:3, 3] = position
        transform[0:3, 0:3] = rotation

        return transform

    @staticmethod
    def get_position_and_rotation(transform: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:

        if transform.shape != (4, 4):
            raise ValueError("Transform must be a 4x4 matrix")

        position = transform[0:3, 3]
        rotation = transform[0:3, 0:3]

        return position, rotation


# Define a helper class to generate a circular trajectory on the y-z plan for the end effector of the panda robot
class CircularTrajectoryGenerator:

    def __init__(self,
                 initial_point: np.ndarray,
                 circle_ray: float,
                 steps_per_circle: int):

        self.current_point = initial_point
        self.circle_ray = circle_ray
        self.steps_per_circle = steps_per_circle
        self.circle_center = initial_point - np.array([0,0,circle_ray])
        self.theta = math.pi/2
        self.delta_theta = 2*math.pi/self.steps_per_circle

    def get_new_point(self) -> np.ndarray:
        self.theta += self.delta_theta
        self.current_point = self.circle_center + [0,self.circle_ray*math.cos(self.theta),self.circle_ray*math.sin(self.theta)]
        return self.current_point


# Define a helper class to generate a rhombus-shaped trajectory on the y-z plan for the end effector of the panda robot
class RhombusTrajectoryGenerator:

    def __init__(self,
                 initial_point: np.ndarray,
                 rhombus_side: float,
                 steps_per_rhombus_side: int):

        self.current_point = initial_point
        self.rhombus_side = rhombus_side
        self.steps_per_rhombus_side = steps_per_rhombus_side
        self.delta = self.rhombus_side/self.steps_per_rhombus_side
        self.t = -1

    def get_new_point(self) -> np.ndarray:
        self.t += 1
        modular_t = self.t%(4*self.steps_per_rhombus_side)
        if modular_t < self.steps_per_rhombus_side:
            self.current_point += np.array([0,self.delta,-self.delta])
        elif modular_t < 2 * self.steps_per_rhombus_side:
            self.current_point += np.array([0,-self.delta,-self.delta])
        elif modular_t < 3 * self.steps_per_rhombus_side:
            self.current_point += np.array([0,-self.delta,self.delta])
        else:
            self.current_point += np.array([0,self.delta,self.delta])
        return self.current_point


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



############################################################################################## MODEL INSERTION

# Set the verbosity
scenario.set_verbosity(scenario.Verbosity_warning)

# Get the default simulator and the default empty world
gazebo, world = init_gazebo_sim()

# Insert a Panda using the class
panda_position = [0, 0, 0]
panda_yaw = 0 # change this angle if you want to rotate the panda along the vertical axes
panda_quaternion = list(Quaternion.to_wxyz(Rotation.from_euler('z', panda_yaw).as_quat()))
panda = Panda(world=world, position=panda_position, orientation=list(panda_quaternion))

# Reset the listed joints at different positions
joints_no_fingers = [j for j in panda.joint_names() if j.startswith("panda_joint")]
nr_of_joints = len(joints_no_fingers)
q0 = [-0.10159305, -0.28819963,  0.07327405, -1.36992404,  0.06682928, 2.65289244,  0.74718812]
dq0 = [0] * nr_of_joints
panda.to_gazebo().reset_joint_positions(q0, joints_no_fingers)
panda.to_gazebo().reset_joint_velocities(dq0, joints_no_fingers)

# Show the GUI
import time
gazebo.gui()
gazebo.run(paused=True)
time.sleep(5)

# Disable self-collisions
panda.enable_self_collisions(enable=False)



############################################################################################## TORQUE CONTROLLER SETUP

if torque_control:

    # Set the controller period
    controller_period = 0.001
    panda.set_controller_period(controller_period)

    # Create the controller context
    controller_context = context.ComputedTorqueFixedBaseContext(
        name="ComputedTorqueFixedBase",
        kp=[30.0] * panda.dofs(),
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
    panda.set_joint_position_targets(panda.joint_positions())
    panda.set_joint_velocity_targets([0.0] * panda.dofs())
    panda.set_joint_acceleration_targets([0.0] * panda.dofs())




############################################################################################## INVERSE KINEMATICS SETUP

# Get the controlled joints (no fingers)
controlled_joints = [j for j in panda.joint_names() if "_finger_" not in j]

# Create the IK object
ik = InverseKinematicsNLP(urdf_filename=gym_ignition_models.get_model_file("panda"),
                          considered_joints=controlled_joints)

# Initialize IK
ik.initialize(verbosity=1,
              cost_tolerance=1e-50,
              floating_base=False)

# Add the cartesian target of the end effector
end_effector = "end_effector_frame"
ik.add_target(frame_name=end_effector,
              target_type=TargetType.POSE)



############################################################################################## MAIN LOOP

# Define a circular and a rhombus-shaped Cartesian trajectory for the end-effector of the panda robot
initial_point = [0.5, 0, 1.0]

rhombus_side = 0.02
steps_per_rhombus_side = 10
rhombus_generator = RhombusTrajectoryGenerator(initial_point,rhombus_side,steps_per_rhombus_side)

circle_ray = 0.02
steps_per_circle = 40
circle_generator = CircularTrajectoryGenerator(initial_point,circle_ray,steps_per_circle)

# Let the end-effector of the panda robot follow:
# - a rhombus-shaped trajectory 3 times in a row
# - a circular trajectory 3 times in a row (in the opposite direction)
for j in range(6):

    if j < 3:
        # rhombus-shaped trajectory
        steps_per_cycle = 4*steps_per_rhombus_side
        trajectory_generator = rhombus_generator
    else:
        # circular trajectory
        steps_per_cycle = steps_per_circle
        trajectory_generator = circle_generator

    for i in range(steps_per_cycle):

        # New end-effector target in world coordinates
        world_target_ee_position = list(trajectory_generator.get_new_point())

        # New end-effector target in base coordinates (as required by the inverse kinematics module)
        homogeneous_world_target_ee_position = world_target_ee_position.copy()
        homogeneous_world_target_ee_position.append(1)
        base_transform = Transform.from_position_and_quaternion(np.array(panda_position),np.array(panda_quaternion))
        base_target_ee_position =  np.linalg.inv(base_transform).dot(homogeneous_world_target_ee_position)[0:3]
        base_target_ee_quaternion = [1, 0, 0, 0]

        # Set the target transform
        ik.update_transform_target(target_name=end_effector,
                                   position=np.array(base_target_ee_position),
                                   quaternion=np.array(base_target_ee_quaternion))

        # Solve the IK problem
        ik.solve()

        # Get the solution
        ik_solution = ik.get_solution()

        if torque_control:
            # Set position references for the controller
            for joint_name, position in zip(controlled_joints, ik_solution.joint_configuration):
                panda.get_joint(joint_name).set_position_target(position)
            for _ in range(200):
                gazebo.run()
        else:
            # Directly reset the joints
            panda.to_gazebo().reset_joint_positions(ik_solution.joint_configuration, controlled_joints)
            gazebo.run(paused=True)

        # Evaluate the tracking performances
        ee_position = panda.get_link(link_name=end_effector).position()
        error = math.dist(world_target_ee_position,ee_position)
        print("Tracking error =", round(error,3))
