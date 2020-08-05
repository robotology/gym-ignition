# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import List
import gym_ignition_models
from gym_ignition import scenario_bindings as scenario
from gym_ignition.utils.scenario import init_gazebo_sim


# Set the verbosity
scenario.set_verbosity(level=4)

# Get the default simulator and the default empty world
gazebo, world = init_gazebo_sim()


# Define a helper class to simplify model insertion.
# A more complete class is stored in gym_ignition_environments.model.panda.
class Panda(scenario.Model):

    def __init__(self,
                 world: scenario.World,
                 position: List[str] = (0., 0, 0),
                 orientation: List[str] = (1., 0, 0, 0)):

        # Initialize the base class
        super().__init__()

        # Get the model file
        urdf = gym_ignition_models.get_model_file("panda")

        # Insert the model in the world
        name = "panda_manipulator"
        pose = scenario.Pose(position, orientation)
        world.insert_model(urdf, pose, name)

        # Get and store the model from the world
        self.model = world.get_model(model_name=name)

    def __getattr__(self, name):
        print("getting", name)
        return getattr(self.model, name)


# Create a Panda class that automatically inserts the model in the world
panda = Panda(world=world)

print("Valid:", panda.valid())

# Print some model information
print(f"Name: {panda.name()}")
print(f"Position: {panda.base_position()}")
print(f"Orientation: {panda.base_orientation()}")
print(f"Joints: {panda.joint_names()}")
