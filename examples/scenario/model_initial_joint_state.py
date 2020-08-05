# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym_ignition_environments import models
from gym_ignition import scenario_bindings as scenario
from gym_ignition.utils.scenario import init_gazebo_sim


# Set the verbosity
scenario.set_verbosity(level=4)

# Get the default simulator and the default empty world
gazebo, world = init_gazebo_sim()

# Create a Panda class that automatically inserts the model in the world
panda = models.panda.Panda(world=world)

# Print some model information
print(f"Name: {panda.name()}")
print(f"Position: {panda.base_position()}")
print(f"Orientation: {panda.base_orientation()}")
print(f"Joints: {panda.joint_names()}")
