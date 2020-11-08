# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
from . import utils
from gym_ignition.robots import sim
import gym_ignition_models as models
from gym_ignition.utils.inverse_kinematics_nlp import TargetType
from gym_ignition.base.robot.robot_joints import JointControlMode, PID
from gym_ignition.utils.inverse_kinematics_nlp import InverseKinematicsNLP

# From: https://github.com/mkrizmancic/franka_gazebo/blob/master/config/default.yaml
# These gains work well with a physics step of 1000 Hz.
pid_gains_1000_hz = {
    'panda_joint1': PID(p=50, i=0, d=20),
    'panda_joint2': PID(p=10000, i=0, d=500),
    'panda_joint3': PID(p=100, i=0, d=10),
    'panda_joint4': PID(p=1000, i=0, d=50),
    'panda_joint5': PID(p=100, i=0, d=10),
    'panda_joint6': PID(p=100, i=0, d=10),
    'panda_joint7': PID(p=10, i=0.5, d=0.1),
    'panda_finger_joint1': PID(p=100, i=0, d=50),
    'panda_finger_joint2': PID(p=100, i=0, d=50),
}


def test_inverse_kinematics():
    # Get the simulator
    physics_rate = 1000.0
    gazebo = utils.Gazebo(physics_rate=physics_rate)

    # Get the robot
    panda = sim.gazebo.panda.PandaRobot(gazebo=gazebo.simulator)

    # Set the PID gains
    for joint_name, pid in pid_gains_1000_hz.items():
        ok_pid = panda.set_joint_pid(joint_name=joint_name, pid=pid)
        assert ok_pid

    # Configure the PID controllers step time
    ok_dt = panda.set_dt(step_size=(1.0 / physics_rate))
    assert ok_dt

    # Get the controlled joints (no fingers)
    controlled_joints = [j for j in panda.joint_names() if "_finger_" not in j]

    # Control the robot in position
    for joint_name in panda.joint_names():
        ok_mode = panda.set_joint_control_mode(joint_name, JointControlMode.POSITION)
        assert ok_mode, f"Failed to control joint {joint_name} in position"

    # Create the IK object
    ik = InverseKinematicsNLP(urdf_filename=models.get_model_file("panda"),
                              considered_joints=controlled_joints)

    # Initialize IK
    ik.initialize(verbosity=1,
                  cost_tolerance=1e-10,
                  floating_base=False)

    # There should be no active targets
    assert len(ik.get_active_target_names()) == 0

    # Add the cartesian target of the end effector
    end_effector = "end_effector_frame"
    ik.add_target(frame_name=end_effector,
                  target_type=TargetType.POSE)

    assert set(ik.get_active_target_names()) == {end_effector}
    assert ik.get_target_data(target_name=end_effector).type == TargetType.POSE

    target_ee_position = np.array([0.3, 0, 0.8])
    target_ee_quaternion = np.array([0.0, 0.0, 0.0, 1.0])

    # Set the target transform
    ik.update_transform_target(target_name=end_effector,
                               position=target_ee_position,
                               quaternion=target_ee_quaternion)

    # Solve the IK problem
    ik.solve()

    # Get the solution
    ik_solution = ik.get_solution()

    assert ik_solution.joint_configuration.size == len(controlled_joints)
    assert np.allclose(ik_solution.base_position, np.array([0.0, 0, 0]))
    assert np.allclose(ik_solution.base_quaternion, np.array([1.0, 0, 0, 0]))

    # Set the desired joint configuration
    for joint_name, position in zip(controlled_joints, ik_solution.joint_configuration):
        ok_pos = panda.set_joint_position(joint_name, position)
        assert ok_pos

    for _ in range(1000):
        gazebo.step()

    # Get the end effector pose
    ee_position, ee_quaternion = panda.link_pose(link_name=end_effector)

    assert np.allclose(target_ee_position, ee_position, atol=0.02)
    assert np.allclose(target_ee_quaternion, ee_quaternion, atol=0.03)

    gazebo.close()
