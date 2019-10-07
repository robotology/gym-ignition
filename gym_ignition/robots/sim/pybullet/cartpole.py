# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pybullet
from gym_ignition.robots import pybullet_robot


class CartPolePyBulletRobot(pybullet_robot.PyBulletRobot):
    def __init__(self, p: pybullet, model_file: str, plane_id: int, **kwargs):
        # Initialize base class
        super().__init__(
            p=p,
            model_file=model_file,
            plane_id=plane_id,
            keep_fixed_joints=False)
