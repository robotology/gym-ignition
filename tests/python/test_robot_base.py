# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import pytest
import numpy as np
import pybullet_data
import pybullet as p
from pybullet_utils import bullet_client
from gym_ignition.utils import resource_finder
from gym_ignition import gympp_bindings as bindings
from gym_ignition.robots.sim import gazebo, pybullet


class Simulator(abc.ABC):
    simulator_name: str = ""
    @abc.abstractmethod
    def step(self): ...


class Gazebo(Simulator):
    simulator_name = "gazebo"

    def __init__(self, physics_rate: float):
        rtf = 1.0
        iterations = 1
        self._gazebo = bindings.GazeboWrapper(iterations, rtf, physics_rate)
        assert self._gazebo

        self._gazebo.setVerbosity(4)

        empty_world = resource_finder.find_resource("DefaultEmptyWorld.world")
        ok_world = self._gazebo.setupGazeboWorld(worldFile=empty_world)
        assert ok_world

        ok_initialize = self._gazebo.initialize()
        assert ok_initialize

    def step(self):
        self._gazebo.run()


class PyBullet(Simulator):
    simulator_name = "pybullet"

    def __init__(self, physics_rate: float):
        self._pybullet = bullet_client.BulletClient(p.DIRECT)
        assert self._pybullet

        self._pybullet.setRealTimeSimulation(0)
        self._pybullet.setGravity(0, 0, -9.81)
        self._pybullet.setTimeStep(1.0 / physics_rate)

        self._pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = self._pybullet.loadURDF("plane_implicit.urdf")

    def step(self):
        self._pybullet.stepSimulation()


def get_simulator(simulator_name: str) -> Simulator:
    if simulator_name == "gazebo":
        return Gazebo(500.0)
    elif simulator_name == "pybullet":
        return PyBullet(500.0)
    else:
        raise Exception(f"Simulator {simulator_name} not recognized")


def get_robot(simulator: Simulator, **kwargs):
    if simulator.simulator_name == "gazebo":
        return gazebo.pendulum.PendulumGazeboRobot(model_file="Pendulum/Pendulum.urdf",
                                                   gazebo=simulator._gazebo,
                                                   **kwargs)
    elif simulator.simulator_name == "pybullet":
        return pybullet.pendulum.PendulumPyBulletRobot(
            model_file="Pendulum/Pendulum.urdf",
            p=simulator._pybullet,
            plane_id=simulator.plane_id,
            **kwargs)
    else:
        raise Exception(f"Simulator {simulator.simulator_name} not recognized")


@pytest.mark.parametrize("simulator_name", ["gazebo", "pybullet"])
def test_robot_fixed_base(simulator_name: str):
    simulator = get_simulator(simulator_name)
    base_position = np.array([-3, 3, 2])
    robot = get_robot(simulator, base_position=base_position, floating=False)

    # Robot should not fall due to gravity
    for _ in range(100):
        simulator.step()

    assert np.allclose(robot.base_pose()[0], base_position, atol=1e-3)
    assert np.allclose(robot.base_pose()[1], np.array([1.0, 0, 0, 0]), atol=1e-3)


@pytest.mark.parametrize("simulator_name", ["gazebo", "pybullet"])
def test_robot_floating_base(simulator_name: str):
    simulator = get_simulator(simulator_name)
    base_position = np.array([-3, 3, 2])
    robot = get_robot(simulator, base_position=base_position, floating=True)

    assert np.allclose(robot.base_pose()[0], base_position, atol=1e-3)
    assert np.allclose(robot.base_pose()[1], np.array([1.0, 0, 0, 0]), atol=1e-3)

    old_pos_z = base_position[2]

    # Robot will fall due to gravity
    for _ in range(100):
        simulator.step()
        current_pos_z = robot.base_pose()[0][2]
        assert current_pos_z < old_pos_z
        old_pos_z = current_pos_z


@pytest.mark.parametrize("simulator_name", ["pybullet"])
def test_robot_floating_to_fixed(simulator_name: str):
    simulator = get_simulator(simulator_name)
    base_position = np.array([-3, 3, 2])
    robot = get_robot(simulator, base_position=base_position, floating=True)

    # Let's make the robot fall a little
    for _ in range(10):
        simulator.step()

    current_base_position, current_base_orientation = robot.base_pose()
    assert current_base_position[2] < base_position[2]

    ok_fixed_base = robot.set_as_floating_base(False)
    assert ok_fixed_base, "Failed to set the robot as fixed base"

    ok_reset_base_pose = robot.reset_base_pose(current_base_position,
                                               current_base_orientation)
    assert ok_reset_base_pose, "Failed to reset the base pose"

    # Robot should now not fall due to gravity
    for _ in range(100):
        simulator.step()

    assert np.allclose(robot.base_pose()[0], current_base_position, atol=1e-3)
    assert np.allclose(robot.base_pose()[1], current_base_orientation, atol=1e-3)
