# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc


class ModelRandomizer(abc.ABC):

    @abc.abstractmethod
    def randomize_model(self) -> str:
        """
        Randomize the model.

        Return:
            A string with the randomized model.
        """
        pass

    def seed_model_randomizer(self, seed: int) -> None:
        """
        Seed the randomizer to ensure reproducibility.

        Args:
            seed: The seed number.
        """
        pass
