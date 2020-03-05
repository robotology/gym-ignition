# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
import gym_ignition_models
from gym_ignition.robots import gazebo_robot
from gym_ignition.base.robot import robot_joints


class PandaRobot(gazebo_robot.GazeboRobot):
    def __init__(self, gazebo, model_file: str = None, **kwargs):

        # Get the model
        if model_file is None:
            model_file = gym_ignition_models.get_model_file("panda")

        # Initialize base class
        super().__init__(model_file=model_file,
                         gazebo=gazebo,
                         controller_rate=kwargs.get("controller_rate"))

        base_position = np.array([0., 0., 0.]) \
            if "base_position" not in kwargs else kwargs["base_position"]

        base_orientation = np.array([1., 0., 0., 0.]) \
            if "base_orientation" not in kwargs else kwargs["base_orientation"]

        ok_base_pose = self.set_initial_base_pose(base_position, base_orientation)
        assert ok_base_pose, "Failed to set initial base pose"

        # Insert the model in the simulation
        _ = self.gympp_robot

        # From:
        # https://github.com/mkrizmancic/franka_gazebo/blob/master/config/default.yaml
        pid_gains_1000hz = {
            'panda_joint1': robot_joints.PID(p=50, i=0, d=20),
            'panda_joint2': robot_joints.PID(p=10000, i=0, d=500),
            'panda_joint3': robot_joints.PID(p=100, i=0, d=10),
            'panda_joint4': robot_joints.PID(p=1000, i=0, d=50),
            'panda_joint5': robot_joints.PID(p=100, i=0, d=10),
            'panda_joint6': robot_joints.PID(p=100, i=0, d=10),
            'panda_joint7': robot_joints.PID(p=10, i=0.5, d=0.1),
            'panda_finger_joint1': robot_joints.PID(p=100, i=0, d=50),
            'panda_finger_joint2': robot_joints.PID(p=100, i=0, d=50),
        }

        assert set(self.joint_names()) == set(pid_gains_1000hz.keys())

        for joint_name, pid in pid_gains_1000hz.items():
            ok_pid = self.set_joint_pid(joint_name=joint_name, pid=pid)
            assert ok_pid
