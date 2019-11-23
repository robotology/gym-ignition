# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import numpy as np
import pybullet_data
import pybullet as p
import gympp_bindings as bindings
from gym_ignition.robots import sim
from pybullet_utils import bullet_client
from gym_ignition.utils import resource_finder


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

    def __init__(self,
                 physics_rate: float,
                 iterations: int = int(1),
                 rtf=float(np.finfo(np.float32).max)):
        self.simulator = bindings.GazeboWrapper(iterations, rtf, physics_rate)
        assert self.simulator

        self.simulator.setVerbosity(4)

        empty_world = resource_finder.find_resource("DefaultEmptyWorld.world")
        ok_world = self.simulator.setupGazeboWorld(worldFile=empty_world)
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
            model_file="Pendulum/Pendulum.urdf",
            gazebo=simulator.simulator,
            **kwargs)
    elif simulator.simulator_name == "pybullet":
        return sim.pybullet.pendulum.PendulumPyBulletRobot(
            model_file="Pendulum/Pendulum.urdf",
            p=simulator.simulator,
            plane_id=simulator.plane_id,
            **kwargs)
    else:
        raise Exception(f"Simulator {simulator.simulator_name} not recognized")


def get_cartpole(simulator: Simulator, **kwargs):
    if simulator.simulator_name == "gazebo":
        return sim.gazebo.cartpole.CartPoleGazeboRobot(
            model_file="CartPole/CartPole.urdf",
            gazebo=simulator.simulator,
            **kwargs)
    elif simulator.simulator_name == "pybullet":
        return sim.pybullet.cartpole.CartPolePyBulletRobot(
            model_file="CartPole/CartPole.urdf",
            p=simulator.simulator,
            plane_id=simulator.plane_id,
            **kwargs)
    else:
        raise Exception(f"Simulator {simulator.simulator_name} not recognized")
