# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
import gym_ignition_models
from gym_ignition.robots import gazebo_robot


class CartPoleGazeboRobot(gazebo_robot.GazeboRobot):

    def __init__(self, gazebo, model_file: str = None, **kwargs):

        # Get the model
        if model_file is None:
            model_file = gym_ignition_models.get_model_file("cartpole")

        # Initialize base class
        super().__init__(model_file=model_file,
                         gazebo=gazebo,
                         controller_rate=kwargs.get("controller_rate"))

        # Initial base position
        base_position = np.array([0., 0., 0.]) \
            if "base_position" not in kwargs else kwargs["base_position"]

        # Initial base orientation
        base_orientation = np.array([1., 0., 0., 0.]) \
            if "base_orientation" not in kwargs else kwargs["base_orientation"]

        # Set the base pose
        ok_base_pose = self.set_initial_base_pose(base_position, base_orientation)
        assert ok_base_pose, "Failed to set base pose"

        # Insert the model in the simulation
        _ = self.gympp_robot
