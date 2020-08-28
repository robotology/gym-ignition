# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from scenario import core as scenario_core
from scenario import gazebo as scenario_gazebo
from gym_ignition.utils.scenario import init_gazebo_sim
import time

# Set the verbosity
scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_warning)

# Get the default simulator and the default empty world
gazebo, world = init_gazebo_sim()

# Insert a cube from URDF file specifying the pose and the model name
model_name = "cube"
default_pose = scenario_core.Pose([1.0, 1.0, 5.0], [1., 0, 0, 0])
world.insert_model("/home/kammo/git/gym-ignition/examples/scenario/urdf/cube/cube.urdf",
                   default_pose,
                   model_name)

# Get the model
cube = world.get_model(model_name=model_name)

gazebo.gui()
time.sleep(1)
gazebo.run()
time.sleep(5)

# Execute simulation
for i in range(10000):
    gazebo.run()