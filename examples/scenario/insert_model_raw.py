# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym_ignition_models
from gym_ignition import scenario_bindings as scenario
from gym_ignition.utils.scenario import init_gazebo_sim

# Set the verbosity
scenario.set_verbosity(level=2)

# Get the default simulator and the default empty world
gazebo, world = init_gazebo_sim()

# Insert a Panda manipulator specifying the pose and the model name
model_name = "my_panda"
default_pose = scenario.Pose([1.0, 1.0, 0.0], [1., 0, 0, 0])
world.insert_model(gym_ignition_models.get_model_file("panda"),
                   default_pose,
                   model_name)

# Get the model
panda = world.get_model(model_name=model_name)
