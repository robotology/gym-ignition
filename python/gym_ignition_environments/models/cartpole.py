# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import List
from scenario import core as scenario
from gym_ignition.utils.scenario import get_unique_model_name
from gym_ignition.scenario import model_wrapper, model_with_file


class CartPole(model_wrapper.ModelWrapper,
               model_with_file.ModelWithFile):

    def __init__(self,
                 world: scenario.World,
                 position: List[float] = (0.0, 0.0, 0.0),
                 orientation: List[float] = (1.0, 0, 0, 0),
                 model_file: str = None):

        # Get a unique model name
        model_name = get_unique_model_name(world, "cartpole")

        # Initial pose
        initial_pose = scenario.Pose(position, orientation)

        # Get the model file (URDF or SDF) and allow to override it
        if model_file is None:
            model_file = CartPole.get_model_file()

        # Insert the model
        ok_model = world.to_gazebo().insert_model(model_file,
                                                  initial_pose,
                                                  model_name)

        if not ok_model:
            raise RuntimeError("Failed to insert model")

        # Get the model
        model = world.get_model(model_name)

        # Initialize base class
        super().__init__(model=model)

    @classmethod
    def get_model_file(cls) -> str:

        import gym_ignition_models
        return gym_ignition_models.get_model_file("cartpole")
