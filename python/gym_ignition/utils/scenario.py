# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym.spaces
import numpy as np
from scenario import core
import gym_ignition_models
from typing import List, Tuple, Union
from scenario import gazebo as scenario


def get_unique_model_name(world: scenario.World, model_name: str) -> str:
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

    while model_name_tentative in world.model_names():

        postfix += 1
        model_name_tentative = f"{model_name}{postfix}"

    return model_name_tentative


def get_unique_world_name(world_name: str) -> str:

    postfix = 0
    world_name_tentative = f"{world_name}"
    ecm_singleton = scenario.ECMSingleton_instance()

    while world_name_tentative in ecm_singleton.world_names():
        postfix += 1
        world_name_tentative = f"{world_name}{postfix}"

    return world_name_tentative


def init_gazebo_sim(step_size: float = 0.001,
                    real_time_factor: float = 1.0,
                    steps_per_run: int = 1) -> Tuple[scenario.GazeboSimulator,
                                                     Union[scenario.World, core.World]]:
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
    gazebo = scenario.GazeboSimulator(step_size, real_time_factor, steps_per_run)

    # Initialize the simulator
    ok_initialize = gazebo.initialize()

    if not ok_initialize:
        raise RuntimeError("Failed to initialize Gazebo")

    # Get the world
    world = gazebo.get_world()

    # Insert the ground plane
    ok_ground = world.insert_model(gym_ignition_models.get_model_file("ground_plane"))

    if not ok_ground:
        raise RuntimeError("Failed to insert the ground plane")

    ok_physics = world.set_physics_engine(scenario.PhysicsEngine_dart)

    if not ok_physics:
        raise RuntimeError("Failed to insert the physics plugin")

    return gazebo, world


def get_joint_positions_space(model: scenario.Model,
                              considered_joints: List[str] = None) -> gym.spaces.Box:
    """
    Build a Box space from the joint position limits.

    Args:
        model: The model from which generating the joint space.
        considered_joints: List of considered joints. It is helpful to restrict the set
            of joints and to enforce a custom joint serialization.

    Returns:
        A box space created from the model's joint position limits.
    """

    if considered_joints is None:
        considered_joints = model.joint_names()

    # Get the joint limits
    joint_limits = model.joint_limits(considered_joints)

    # Build the space
    space = gym.spaces.Box(low=np.array(joint_limits.min),
                           high=np.array(joint_limits.max))

    return space
