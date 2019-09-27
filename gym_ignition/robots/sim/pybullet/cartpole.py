# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym_ignition.robots import pybullet_robot


class CartPolePyBulletRobot(pybullet_robot.PyBulletRobot):
    def __init__(self, **kwargs):
        # Initialize base class
        super().__init__(robot_id=kwargs["robot_id"],
                         p=kwargs["pybullet"])
