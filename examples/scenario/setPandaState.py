# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym_ignition_environments import models
from gym_ignition import scenario_bindings as scenario
from gym_ignition.utils.scenario import init_gazebo_sim
import gym_ignition_models
import time
from typing import List

# Set the verbosity
scenario.set_verbosity(level=1)

# Get the default simulator and the default empty world
gazebo, world = init_gazebo_sim()

# Use GUI
gazebo.gui()
# gazebo.run(paused=True)
time.sleep(3)


# Create a Panda class that automatically inserts the model in the world
class Panda(scenario.Model):

    def __init__(self,
                 world: scenario.World,
                 position: List[float] = (0., 0, 0),
                 orientation: List[float] = (1., 0, 0, 0)):
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

    # def __getattr__(self, name):
    #     print("getting", name)
    #     return getattr(self.model, name)


panda_pos = [0, 0, 0]
panda_ori = [1, 0, 0, 0]  # quaternion: w,x,y,z
panda = Panda(world=world, position=panda_pos, orientation=panda_ori)

# des_jnt_pos = [0.0] * panda.model.dofs()
# panda.model.reset_joint_positions(des_jnt_pos)
# assert panda.model.reset_joint_positions(des_jnt_pos)

# step a few seconds of simulation
for i in range(3000):
    gazebo.run()

time.sleep(5)
