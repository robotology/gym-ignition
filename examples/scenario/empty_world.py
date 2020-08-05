# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym_ignition_models
from gym_ignition import scenario_bindings as scenario

# Set the verbosity
scenario.set_verbosity(level=2)

step_size = 0.001
steps_per_run = 1
real_time_factor = 1.0

# Create the simulator
gazebo = scenario.GazeboSimulator(step_size=step_size,
                                  rtf=real_time_factor,
                                  steps_per_run=steps_per_run)

# Initialize the simulator
gazebo.initialize()

# Get the default empty world
world = gazebo.get_world()

# Insert the ground plane
world.insert_model(gym_ignition_models.get_model_file("ground_plane"))

# Insert the physics with the DART backend
world.set_physics_engine(engine=scenario.PhysicsEngine_dart)

# Print the models of the world (just the flat ground plane)
print(world.model_names())
