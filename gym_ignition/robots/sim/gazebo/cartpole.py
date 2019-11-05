# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym_ignition.robots import gazebo_robot


class CartPoleGazeboRobot(gazebo_robot.GazeboRobot):
    def __init__(self, model_file: str, gazebo, **kwargs):
        # Initialize base class
        super().__init__(model_file=model_file,
                         gazebo=gazebo,
                         controller_rate=kwargs.get("controller_rate"))

        # Insert the model in the simulation
        _ = self.gympp_robot
