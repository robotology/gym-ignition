#!/usr/bin/env python3

# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
import time
import itertools
import numpy as np
import gym_ignition
from typing import Tuple
from gym_ignition.utils import logger
from gym_ignition import gympp_bindings as bindings

# Set verbosity
logger.set_level(gym.logger.ERROR)


def get_gazebo_and_robot(rtf: float,
                         physics_rate: float,
                         agent_rate: float,
                         controller_rate: float = None,
                         ) \
        -> Tuple[bindings.GazeboWrapper, bindings.Robot]:

    lib_name = "RobotController"
    class_name = "gympp::plugins::RobotController"

    model_sdf = "CartPole/CartPole.sdf"
    world_sdf = "CartPole.world"

    num_of_iterations_per_gazebo_step = physics_rate / agent_rate
    assert num_of_iterations_per_gazebo_step == int(num_of_iterations_per_gazebo_step)

    # Create the gazebo wrapper
    gazebo = bindings.GazeboWrapper(int(num_of_iterations_per_gazebo_step),
                                    rtf,
                                    physics_rate)
    assert gazebo, "Failed to get the gazebo wrapper"

    # Set verbosity
    logger.set_level(gym.logger.MIN_LEVEL)

    # Initialize the world
    world_ok = gazebo.setupGazeboWorld(world_sdf)
    assert world_ok, "Failed to initialize the gazebo world"

    # Initialize the model
    model_ok = gazebo.setupGazeboModel(model_sdf)
    assert model_ok, "Failed to initialize the gazebo model"

    # Initialize the plugin
    wrapper_ok = gazebo.setupIgnitionPlugin(lib_name, class_name)
    assert wrapper_ok, "Failed to setup the ignition plugin"

    # Initialize the ignition gazebo wrapper
    gazebo_initialized = gazebo.initialize()
    assert gazebo_initialized, "Failed to initialize ignition gazebo"

    # Get the robot name
    model_names = gazebo.getModelNames()
    assert len(model_names) == 1, "The environment has more than one model"
    model_name = model_names[0]

    # Get the robot object
    robot = bindings.RobotSingleton_get().getRobot(model_name)
    assert robot, "Failed to get the Robot object"
    assert robot.valid(), "The Robot object is not valid"

    # Set the joint controller period
    robot.setdt(0.001)

    # Return the simulator and the robot object
    return gazebo, robot


def almost_equal(first, second, epsilon=None) -> bool:
    if not epsilon:
        epsilon = first * 0.05

    res = np.abs(first - second) <= epsilon

    if not res:
        print("----------------")
        print("ASSERTION FAILED")
        print("----------------")
        print("#1={}".format(first))
        print("#2={}".format(second))
        print("Error={}".format(np.abs(first-second)))
        print("Tolerance={}".format(epsilon))
        print("----------------")

    return res


def template_test(rtf: float,
                  physics_rate: float,
                  agent_rate: float,
                  controller_rate: float = None,
                  ) -> None:

    # Get the simulator and the robot objects
    gazebo, robot = get_gazebo_and_robot(rtf=rtf,
                                         physics_rate=physics_rate,
                                         agent_rate=agent_rate,
                                         controller_rate=None,  # Does not matter
                                         )

    # Trajectory specifications
    trajectory_dt = 1 / agent_rate
    tot_simulated_seconds = 2

    # Generate the cart trajectory. This is analogous of setting an action containing
    # the cart references, hence it is related to the agent rate.
    cart_ref = np.fromiter(
        (0.2 * np.sin(2 * np.pi * 0.5 * t) for t in np.arange(0, tot_simulated_seconds,
                                                              trajectory_dt)),
        dtype=np.float)

    avg_time_per_step = 0.0
    start = time.time()

    for (i, ref) in enumerate(cart_ref):
        # Set the references
        ok_ref = robot.setJointPositionTarget("linear", ref)
        assert ok_ref, "Failed to set joint references"

        # Step the simulator
        now = time.time()
        gazebo.run()
        this_step = time.time() - now
        avg_time_per_step = avg_time_per_step + 0.1 * (this_step - avg_time_per_step)

    stop = time.time()
    elapsed_time = stop - start

    # Compute useful quantities
    # simulation_dt = 1 / simulation_rate
    number_of_actions = tot_simulated_seconds / trajectory_dt
    physics_iterations_per_run = physics_rate / agent_rate
    simulation_dt = 1 / (physics_rate * rtf)

    assert number_of_actions == cart_ref.size

    print()
    print("Elapsed time = {}".format(elapsed_time))
    print("Average step time = {}".format(avg_time_per_step))
    print("Number of actions: = {}".format(tot_simulated_seconds / trajectory_dt))
    print("Physics iteration per simulator step = {}".format(physics_iterations_per_run))
    print()

    # Check if the average time per step matches what expected
    assert almost_equal(first=avg_time_per_step,
                        second=simulation_dt * physics_iterations_per_run,
                        epsilon=avg_time_per_step * 0.5)

    # Check if the total time of the simulation matched what expected
    assert almost_equal(first=elapsed_time,
                        second=number_of_actions * simulation_dt * physics_iterations_per_run,
                        epsilon=elapsed_time * 0.5)


def test_rates():
    # Test matrix
    rtf_list = [0.5, 1, 2, 5, 10]
    agent_rate_list = [100, 1000]
    physics_rate_list = [100, 500, 1000, 2000]

    for agent_rate in agent_rate_list:

        # Compute all the combinations of simulation rate and physics rate
        combinations = list(itertools.product(rtf_list, physics_rate_list))

        for rtf, physics_rate in combinations:

            # Remove some combination too demanding for a single process
            if physics_rate * rtf >= 5000:
                continue

            # This case is not allowed
            if agent_rate > physics_rate:
                continue

            print("========")
            print("Testing:")
            print("========")
            print("RTF = {}".format(rtf))
            print("Agent rate = {}".format(agent_rate))
            print("Physics rate = {}".format(physics_rate))

            # Test this combination
            template_test(rtf=rtf,
                          physics_rate=physics_rate,
                          controller_rate=None,
                          agent_rate=agent_rate)
