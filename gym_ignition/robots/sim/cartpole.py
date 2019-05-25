# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym_ignition.robots import factory_robot


class CartPoleRobot(factory_robot.FactoryRobot):
    def __init__(self, **kwargs):
        # Initialize base class
        super().__init__(robot_name=kwargs["robot_name"],
                         controller_rate=kwargs.get("controller_rate"))
