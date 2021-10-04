# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import tempfile

import numpy as np
import pybullet as p
import pybullet_data
from gym_ignition import gympp_bindings as bindings
from gym_ignition.robots import gazebo_robot, sim
from gym_ignition.utils import resource_finder
from pybullet_utils import bullet_client


class Simulator(abc.ABC):
    simulator = None
    simulator_name: str = ""

    @abc.abstractmethod
    def step(self):
        pass

    @abc.abstractmethod
    def close(self):
        pass


class Gazebo(Simulator):
    simulator_name = "gazebo"

    def __init__(
        self,
        physics_rate: float,
        iterations: int = int(1),
        rtf=float(np.finfo(np.float32).max),
    ):
        self.simulator = bindings.GazeboSimulator(iterations, rtf, physics_rate)
        assert self.simulator

        self.simulator.setVerbosity(4)

        empty_world = resource_finder.find_resource("DefaultEmptyWorld.world")
        ok_world = self.simulator.setupGazeboWorld(empty_world)
        assert ok_world

        ok_initialize = self.simulator.initialize()
        assert ok_initialize

    def step(self):
        assert self.simulator.initialized()

        ok_run = self.simulator.run()
        assert ok_run

    def close(self):
        ok_close = self.simulator.close()
        assert ok_close


class PyBullet(Simulator):
    simulator_name = "pybullet"

    def __init__(self, physics_rate: float):
        self.simulator = bullet_client.BulletClient(p.DIRECT)
        assert self.simulator

        self.simulator.setRealTimeSimulation(0)
        self.simulator.setGravity(0, 0, -9.81)
        self.simulator.setTimeStep(1.0 / physics_rate)

        self.simulator.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = self.simulator.loadURDF("plane_implicit.urdf")

    def step(self):
        self.simulator.stepSimulation()

    def close(self):
        pass


# TODO: we need urdf for the controllers
def get_pendulum(simulator: Simulator, **kwargs):
    if simulator.simulator_name == "gazebo":
        return sim.gazebo.pendulum.PendulumGazeboRobot(
            gazebo=simulator.simulator, **kwargs
        )
    elif simulator.simulator_name == "pybullet":
        return sim.pybullet.pendulum.PendulumPyBulletRobot(
            p=simulator.simulator, plane_id=simulator.plane_id, **kwargs
        )
    else:
        raise Exception(f"Simulator {simulator.simulator_name} not recognized")


def get_cartpole(simulator: Simulator, **kwargs):
    if simulator.simulator_name == "gazebo":
        return sim.gazebo.cartpole.CartPoleGazeboRobot(
            gazebo=simulator.simulator, **kwargs
        )
    elif simulator.simulator_name == "pybullet":
        return sim.pybullet.cartpole.CartPolePyBulletRobot(
            p=simulator.simulator, plane_id=simulator.plane_id, **kwargs
        )
    else:
        raise Exception(f"Simulator {simulator.simulator_name} not recognized")


class CubeGazeboRobot(gazebo_robot.GazeboRobot):
    def __init__(self, gazebo, initial_position: np.ndarray, model_file: str = None):

        if model_file is None:
            # Serialize the cube urdf
            handle, model_file = tempfile.mkstemp()
            with open(handle, "w") as f:
                f.write(get_cube_urdf())

        # Initialize base class
        super().__init__(model_file=model_file, gazebo=gazebo)

        ok_floating = self.set_as_floating_base(True)
        assert ok_floating, "Failed to set the robot as floating base"

        # Initial base position and orientation
        base_position = np.array(initial_position)
        base_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        ok_base_pose = self.set_initial_base_pose(base_position, base_orientation)
        assert ok_base_pose, "Failed to set base pose"

        # Insert the model in the simulation
        _ = self.gympp_robot


def get_cube_urdf() -> str:
    mass = 5.0
    edge = 0.2
    i = 1 / 12 * mass * (edge ** 2 + edge ** 2)
    cube_urdf = f"""
    <robot name="cube_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
        <link name="cube">
            <inertial>
              <origin rpy="0 0 0" xyz="0 0 0"/>
              <mass value="{mass}"/>
              <inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/>
            </inertial>
            <visual>
              <geometry>
                <box size="{edge} {edge} {edge}"/>
              </geometry>
              <origin rpy="0 0 0" xyz="0 0 0"/>
            </visual>
            <collision>
              <geometry>
                <box size="{edge} {edge} {edge}"/>
              </geometry>
              <origin rpy="0 0 0" xyz="0 0 0"/>
            </collision>
        </link>
    </robot>"""
    return cube_urdf
