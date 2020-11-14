# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym_ignition.base.task
from scenario import core as scenario_core


class ModelRandomizer(abc.ABC):

    @abc.abstractmethod
    def randomize_model(self, task: gym_ignition.base.task.Task) -> scenario_core.Model:
        """
        Randomize the model.

        Args:
            task: The task that operates on the model to randomize.

        Return:
            The randomized model.
        """
        pass

    def seed_model_randomizer(self, seed: int) -> None:
        """
        Seed the randomizer to ensure reproducibility.

        Args:
            seed: The seed number.
        """
        pass


class ModelDescriptionRandomizer(abc.ABC):

    @abc.abstractmethod
    def randomize_model_description(self, task: gym_ignition.base.task.Task) -> str:
        """
        Randomize the model description.

        Args:
            task: The task that operates on the model description to randomize.

        Return:
            A string with the randomized model description.
        """
        pass

    def seed_model_description_randomizer(self, seed: int) -> None:
        """
        Seed the randomizer to ensure reproducibility.

        Args:
            seed: The seed number.
        """
        pass
