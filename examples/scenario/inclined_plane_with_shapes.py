# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import numpy as np
from scenario import core
from scenario import gazebo as scenario
from scipy.spatial.transform import Rotation
import gym_ignition_models
from dataclasses import dataclass
from typing import List

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


# Define cube urdf with parametric mass, edge size and static friction coefficient mu
@dataclass
class CubeURDF:

    mass: float = 5.0
    edge: float = 0.2
    mu: float = 1.0

    def urdf(self) -> str:

        i = 1.0 / 12 * self.mass * (self.edge ** 2 + self.edge ** 2)
        urdf = f"""
            <robot name="cube_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
                <link name="cube">
                    <inertial>
                      <origin rpy="0 0 0" xyz="0 0 0"/>
                      <mass value="{self.mass}"/>
                      <inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/>
                    </inertial>
                    <visual>
                      <geometry>
                        <box size="{self.edge} {self.edge} {self.edge}"/>
                      </geometry>
                      <origin rpy="0 0 0" xyz="0 0 0"/>
                    </visual>
                    <collision>
                      <geometry>
                        <box size="{self.edge} {self.edge} {self.edge}"/>
                      </geometry>
                      <origin rpy="0 0 0" xyz="0 0 0"/>
                    </collision>
                </link>
                <gazebo reference="cube">
                  <collision>
                    <surface>
                      <friction>
                        <ode>
                          <mu>{self.mu}</mu>
                          <mu2>{self.mu}</mu2>
                        </ode>
                      </friction>
                    </surface>
                  </collision>
                </gazebo>
            </robot>"""

        return urdf


# Define sphere urdf with parametric mass, radius and static friction coefficient mu
@dataclass
class SphereURDF:

    mass: float = 5.0
    radius: float = 0.2
    mu: float = 1.0

    def urdf(self) -> str:

        i = 2.0 / 5 * self.mass * self.radius * self.radius
        urdf = f"""
            <robot name="sphere_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
                <link name="sphere">
                    <inertial>
                      <origin rpy="0 0 0" xyz="0 0 0"/>
                      <mass value="{self.mass}"/>
                      <inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/>
                    </inertial>
                    <visual>
                      <geometry>
                        <sphere radius="{self.radius}"/>
                      </geometry>
                      <origin rpy="0 0 0" xyz="0 0 0"/>
                    </visual>
                    <collision>
                      <geometry>
                        <sphere radius="{self.radius}"/>
                      </geometry>
                      <origin rpy="0 0 0" xyz="0 0 0"/>
                    </collision>
                </link>
                <gazebo reference="sphere">
                  <collision>
                    <surface>
                      <friction>
                        <ode>
                          <mu>{self.mu}</mu>
                          <mu2>{self.mu}</mu2>
                        </ode>
                      </friction>
                    </surface>
                  </collision>
                </gazebo>
            </robot>"""

        return urdf


# Define cylinder urdf with parametric mass, radius, length and static friction coefficient mu
@dataclass
class CylinderURDF:

    mass: float = 5.0
    radius: float = 0.2
    length: float = 1.0
    mu: float = 1.0

    def urdf(self) -> str:

        ixx = 1.0 / 12 * self.mass * self.length * self.length + 1.0 / 4 * self.mass * self.radius * self.radius
        iyy = 1.0 / 12 * self.mass * self.length * self.length + 1.0 / 4 * self.mass * self.radius * self.radius
        izz = 1.0 / 2 * self.mass * self.radius * self.radius
        urdf = f"""
            <robot name="cylinder_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
                <link name="cylinder">
                    <inertial>
                      <origin rpy="0 0 0" xyz="0 0 0"/>
                      <mass value="{self.mass}"/>
                      <inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{iyy}" iyz="0" izz="{izz}"/>
                    </inertial>
                    <visual>
                      <geometry>
                        <cylinder radius="{self.radius}" length="{self.length}"/>
                      </geometry>
                      <origin rpy="0 0 0" xyz="0 0 0"/>
                    </visual>
                    <collision>
                      <geometry>
                        <cylinder radius="{self.radius}" length="{self.length}"/>
                      </geometry>
                      <origin rpy="0 0 0" xyz="0 0 0"/>
                    </collision>
                </link>
                <gazebo reference="cylinder">
                  <collision>
                    <surface>
                      <friction>
                        <ode>
                          <mu>{self.mu}</mu>
                          <mu2>{self.mu}</mu2>
                        </ode>
                      </friction>
                    </surface>
                  </collision>
                </gazebo>
            </robot>"""

        return urdf


# Define a shape
class Shape:

    def __init__(self,
                 world: scenario.World,
                 position: List[float] = (0, 0, 0),
                 orientation: List[float] = (1, 0, 0, 0),
                 velocity: List[float] = None,
                 model_string: str = SphereURDF(mass=1.0, radius=0.02).urdf(),
                 model_file: str = None,
                 physics_dt: float = 0.001) -> object:

        if model_string is None and model_file is None:
            raise ValueError("You must pass either the model string or file")

        if model_string is not None:
            from gym_ignition.utils import misc
            self.sdf = misc.string_to_file(model_string)

        if model_file is not None:
            from gym_ignition.utils import misc
            sdf_string = scenario.urdfstring_to_sdfstring(model_file)
            self.sdf = misc.string_to_file(sdf_string)

        default_name = scenario.get_model_name_from_sdf(self.sdf)
        name = default_name
        index = 0

        while name in world.model_names():
            name = f"{name}{index}"

        assert world.insert_model(self.sdf, core.Pose(position, orientation), name)
        self.model: scenario.Model = world.get_model(name)
        self.world: scenario.World = world

        if self.model.nr_of_links() != 1:
            raise ValueError("Models with just one link are currently supported")

        if velocity is not None:
            mass = self.model.total_mass()
            base: scenario.Link = self.model.get_link(self.model.base_frame())

            force_mag = mass * np.linalg.norm(velocity) / physics_dt
            force_direction = np.array(velocity) / np.linalg.norm(velocity)

            print(force_mag * force_direction)
            base.apply_world_force(force_mag * force_direction)


# Set the verbosity
scenario.set_verbosity(scenario.Verbosity_warning)

# Create the simulator
step_size = 0.001
steps_per_run = 1
real_time_factor = 1.0
gazebo = scenario.GazeboSimulator(step_size=step_size,
                                  rtf=real_time_factor,
                                  steps_per_run=steps_per_run)

# Initialize the simulator
gazebo.initialize()

# Get the default empty world
world = gazebo.get_world()

# Insert the physics with the DART backend
world.set_physics_engine(engine=scenario.PhysicsEngine_dart)

# Insert the inclined ground plane
ground_plane_pitch = -0.25 * np.pi
ground_plane_quaternion = Quaternion.to_wxyz(Rotation.from_euler('y', ground_plane_pitch).as_quat())
ground_plane_position = [0.0, 0.0, 0.0]
ground_plane_pose = core.Pose(ground_plane_position, ground_plane_quaternion)
world.insert_model(gym_ignition_models.get_model_file("ground_plane"),
                   ground_plane_pose)

# Define common parameters for all the shapes (we wnat to compare shapes with similar characteristics)
mass = 10
mu = 0.8

# Insert a cube
cube_pitch = -0.25 * np.pi
cube_quaternion = list(Quaternion.to_wxyz(Rotation.from_euler('y', ground_plane_pitch).as_quat()))
cube_edge = 0.2
cube_position = [6.0, 0.0, 6.0+cube_edge/2]
cube = Shape(world=world,
             model_string=CubeURDF(mass=mass, edge=cube_edge, mu=mu).urdf(),
             position=cube_position,
             orientation=cube_quaternion)

# Insert a sphere
sphere_quaternion = [1,0,0,0]
sphere_radius = 0.1
sphere_position = [6.0, 0.5, 6.0+sphere_radius]
sphere = Shape(world=world,
               model_string=SphereURDF(mass=mass, radius=sphere_radius, mu=mu).urdf(),
               position=sphere_position,
               orientation=sphere_quaternion)

# Insert a cylinder
cylinder_roll = 0.5 * np.pi
cylinder_quaternion = list(Quaternion.to_wxyz(Rotation.from_euler('x', cylinder_roll).as_quat()))
cylinder_radius = 0.1
cylinder_length = 0.5
cylinder_position = [6.0, 1.0, 6.0+cylinder_radius]
cylinder = Shape(world=world,
                 model_string=CylinderURDF(mass=mass, radius=cylinder_radius, length=cylinder_length, mu=mu).urdf(),
                 position=cylinder_position,
                 orientation=cylinder_quaternion)

# Print the models of the world
print(world.model_names())

# Show the GUI
import time
gazebo.gui()
gazebo.run(paused=True)
time.sleep(3)

# peform 2 seconds of simulation with the shapes falling down on the inclined plane
for i in range(2000):
    gazebo.run()

time.sleep(5)