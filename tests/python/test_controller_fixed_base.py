# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import pytest
import numpy as np
from . import utils
from gym_ignition.utils import sinusoidal_trajectory_generator
from gym_ignition.controllers import computed_torque_fixed_base
from gym_ignition.controllers import computed_torque_fixed_base_cpp
from gym_ignition.base.robot import robot_joints, robot_abc, feature_detector
from gym_ignition.base.controllers import Controller, PositionControllerReferences


@feature_detector
class RobotFeatures(robot_abc.RobotABC, robot_joints.RobotJoints, abc.ABC):
    pass


def get_pybullet_cartpole_resources():
    pybullet = utils.PyBullet(physics_rate=100)
    cart_pole = utils.get_cartpole(pybullet)

    controller = computed_torque_fixed_base.ComputedTorqueFixedBase(
        robot=cart_pole, urdf=cart_pole.model_file,
        controlled_joints=cart_pole.joint_names(),
        kp=np.array([1500.0, 1500.0]), kd=np.array([10.0, 10.0]))

    return pybullet, controller, cart_pole


def get_gazebo_cartpole_resources_python():
    gazebo = utils.Gazebo(physics_rate=100)
    cart_pole = utils.get_cartpole(gazebo)

    controller = computed_torque_fixed_base.ComputedTorqueFixedBase(
        robot=cart_pole, urdf=cart_pole.model_file,
        controlled_joints=cart_pole.joint_names(),
        kp=np.array([1500.0, 1500.0]), kd=np.array([10.0, 10.0]))

    return gazebo, controller, cart_pole


def get_gazebo_cartpole_resources_cpp():
    gazebo = utils.Gazebo(physics_rate=100)
    cart_pole = utils.get_cartpole(gazebo)

    controller = computed_torque_fixed_base_cpp.ComputedTorqueFixedBaseCpp(
        robot=cart_pole, urdf=cart_pole.model_file,
        controlled_joints=cart_pole.joint_names(),
        kp=np.array([1500.0, 1500.0]), kd=np.array([10.0, 10.0]))

    return gazebo, controller, cart_pole


@pytest.mark.parametrize(
    "simulator,controller,cart_pole",
    [
        get_pybullet_cartpole_resources(),
        get_gazebo_cartpole_resources_python(),
        get_gazebo_cartpole_resources_cpp(),
    ])
def test_controller_fixed_base(simulator: utils.Simulator,
                               controller: Controller,
                               cart_pole: RobotFeatures):
    dt = 0.01
    generator = sinusoidal_trajectory_generator.SinusoidalTrajectoryGenerator(
        dt=dt,
        robot=cart_pole,
        initial_positions=np.array([0.25, np.deg2rad(120.0)]),
        f=1.0,
        sine_amplitude=np.array([0.5, np.deg2rad(45.0)]))

    ok_init = controller.initialize()
    assert ok_init, "Failed to initialize controller"

    simulation_duration = 5
    num_steps = int(simulation_duration / dt)

    for ts in range(num_steps):
        position_reference = generator.get_references()
        references = PositionControllerReferences(
            position=position_reference,
            velocity=np.zeros_like(position_reference),
            acceleration=np.zeros_like(position_reference),
        )
        controller.set_control_references(references=references)

        torques = controller.step()

        for idx, joint_name in enumerate(cart_pole.joint_names()):
            ok_torque = cart_pole.set_joint_force(joint_name, torques[idx])
            assert ok_torque

        simulator.step()
        positions_after_step = cart_pole.joint_positions()

        # Skip initial transient
        if ts > 100:
            assert np.allclose(position_reference, positions_after_step, atol=0.03)

    simulator.close()
