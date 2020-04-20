# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import Tuple
import gym_ignition_models
from gym_ignition import scenario_bindings as bindings


def get_unique_model_name(world: bindings.World, model_name: str) -> str:
    """
    Get a unique model name given a world configuration.

    This function find the first available model name starting from the argument and
    appending a integer postfix until the resulting name is unique in the world.

    Tentatives example: `cartpole`, `cartpole1`, `cartpole2`, ...

    Args:
        world: An initialized world.
        model_name: The first model name attempt.

    Raises:
        ValueError: If the world is not valid.

    Returns:
        The unique model name calculated from the original name.
    """

    if world.id() == 0:
        raise ValueError("The world is not valid")

    postfix = 0
    model_name_tentative = f"{model_name}"

    while model_name_tentative in world.modelNames():

        postfix += 1
        model_name_tentative = f"{model_name}{postfix}"

    return model_name_tentative


def get_unique_world_name(world_name: str) -> str:

    postfix = 0
    world_name_tentative = f"{world_name}"
    ecm_singleton = bindings.ECMSingleton_Instance()

    while world_name_tentative in ecm_singleton.worldNames():
        postfix += 1
        world_name_tentative = f"{world_name}{postfix}"

    return world_name_tentative


def init_gazebo_sim(step_size: float = 0.001,
                    real_time_factor: float = 1.0,
                    steps_per_run: int = 1) -> Tuple[bindings.GazeboSimulator,
                                                     bindings.World]:
    """
    Initialize a Gazebo simulation with an empty world and default physics.

    Args:
        step_size: Gazebo step size.
        real_time_factor: The desired real time factor of the simulation.
        steps_per_run: How many steps should be executed at each Gazebo run.

    Raises:
        RuntimeError: If the initialization of either the simulator or the world fails.

    Returns:
        * **gazebo** -- The Gazebo simulator.
        * **world** -- The default world.
    """

    # Create the simulator
    gazebo = bindings.GazeboSimulator(step_size, real_time_factor, steps_per_run)

    # Initialize the simulator
    ok_initialize = gazebo.initialize()

    if not ok_initialize:
        raise RuntimeError("Failed to initialize Gazebo")

    # Get the world
    world = gazebo.getWorld()

    # Insert the ground plane
    ok_ground = world.insertModel(gym_ignition_models.get_model_file("ground_plane"))

    if not ok_ground:
        raise RuntimeError("Failed to insert the ground plane")

    # Insert the physics
    ok_physics = world.insertWorldPlugin("libPhysicsSystem.so",
                                         "scenario::plugins::gazebo::Physics")

    if not ok_physics:
        raise RuntimeError("Failed to insert the physics plugin")

    return gazebo, world
