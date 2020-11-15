# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import Union
from gym_ignition_environments import tasks
from gym_ignition_environments.models import cartpole
from gym_ignition.randomizers import gazebo_env_randomizer
from gym_ignition.randomizers.gazebo_env_randomizer import MakeEnvCallable

# Tasks that are supported by this randomizer. Used for type hinting.
SupportedTasks = Union[tasks.cartpole_discrete_balancing.CartPoleDiscreteBalancing,
                       tasks.cartpole_continuous_swingup.CartPoleContinuousSwingup,
                       tasks.cartpole_continuous_balancing.CartPoleContinuousBalancing]


class CartpoleEnvNoRandomizations(gazebo_env_randomizer.GazeboEnvRandomizer):
    """
    Dummy environment randomizer for cartpole tasks.

    Check :py:class:`~gym_ignition_environments.randomizers.cartpole.CartpoleRandomizersMixin`
    for an example that randomizes the task, the physics, and the model.
    """

    def __init__(self, env: MakeEnvCallable):

        super().__init__(env=env)

    def randomize_task(self, task: SupportedTasks, **kwargs) -> None:
        """
        Prepare the scene for cartpole tasks. It simply removes the cartpole of the
        previous rollout and inserts a new one in the default state. Then, the active
        Task will reset the state of the cartpole depending on the implemented
        decision-making logic.
        """

        if "gazebo" not in kwargs:
            raise ValueError("gazebo kwarg not passed to the task randomizer")

        gazebo = kwargs["gazebo"]

        # Remove the model from the simulation
        if task.model_name is not None and task.model_name in task.world.model_names():

            if not task.world.to_gazebo().remove_model(task.model_name):
                raise RuntimeError("Failed to remove the cartpole from the world")

        # Execute a paused run to process model removal
        if not gazebo.run(paused=True):
            raise RuntimeError("Failed to execute a paused Gazebo run")

        # Insert a new cartpole model
        model = cartpole.CartPole(world=task.world)

        # Store the model name in the task
        task.model_name = model.name()

        # Execute a paused run to process model insertion
        if not gazebo.run(paused=True):
            raise RuntimeError("Failed to execute a paused Gazebo run")
