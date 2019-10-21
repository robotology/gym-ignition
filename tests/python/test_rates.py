# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
import time
import pytest
import string
import random
import itertools
import numpy as np
import gym_ignition
from typing import List, Tuple
from gym_ignition import gympp_bindings as bindings
from gym_ignition.utils import logger, resource_finder

# Set verbosity
logger.set_level(gym.logger.ERROR)


def get_gazebo_and_robot(rtf: float,
                         physics_rate: float,
                         agent_rate: float,
                         # controller_rate: float = None,
                         ) \
        -> Tuple[bindings.GazeboWrapper, bindings.Robot]:

    # ==========
    # PARAMETERS
    # ==========

    model_sdf = "CartPole/CartPole.sdf"
    world_sdf = "DefaultEmptyWorld.world"

    plugin_data = bindings.PluginData()
    plugin_data.setLibName("RobotController")
    plugin_data.setClassName("gympp::plugins::RobotController")

    # Find and load the model SDF file
    model_sdf_file = resource_finder.find_resource(model_sdf)
    with open(model_sdf_file, "r") as stream:
        model_sdf_string = stream.read()

    # Initialize the model data
    model_data = bindings.ModelInitData()
    model_data.setSdfString(model_sdf_string)

    # Create a unique model name
    letters_and_digits = string.ascii_letters + string.digits
    prefix = ''.join(random.choice(letters_and_digits) for _ in range(6))

    # Get the model name
    model_name = bindings.GazeboWrapper.getModelNameFromSDF(model_sdf_string)
    model_data.modelName = prefix + "::" + model_name
    robot_name = model_data.modelName

    num_of_iterations_per_gazebo_step = physics_rate / agent_rate
    assert num_of_iterations_per_gazebo_step == int(num_of_iterations_per_gazebo_step)

    # =============
    # CONFIGURATION
    # =============

    # Create the gazebo wrapper
    gazebo = bindings.GazeboWrapper(int(num_of_iterations_per_gazebo_step),
                                    rtf,
                                    physics_rate)
    assert gazebo, "Failed to get the gazebo wrapper"

    # Set verbosity
    logger.set_level(gym.logger.DEBUG)

    # Initialize the world
    world_ok = gazebo.setupGazeboWorld(world_sdf)
    assert world_ok, "Failed to initialize the gazebo world"

    # Initialize the ignition gazebo wrapper (creates a paused simulation)
    gazebo_initialized = gazebo.initialize()
    assert gazebo_initialized, "Failed to initialize ignition gazebo"

    # Insert the model
    model_ok = gazebo.insertModel(model_data, plugin_data)
    assert model_ok, "Failed to insert the model in the simulation"

    assert bindings.RobotSingleton_get().exists(robot_name), \
        "The robot interface was not registered in the singleton"

    # Extract the robot interface from the singleton
    robot_weak_ptr = bindings.RobotSingleton_get().getRobot(robot_name)
    assert not robot_weak_ptr.expired(), "The Robot object has expired"
    assert robot_weak_ptr.lock(), \
        "The returned Robot object does not contain a valid interface"
    assert robot_weak_ptr.lock().valid(), "The Robot object is not valid"

    # Get the pointer to the robot interface
    robot = robot_weak_ptr.lock()

    # Set the joint controller period
    robot.setdt(0.001 if physics_rate < 1000 else physics_rate)

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
                  # controller_rate: float = None,
                  ) -> None:

    # Get the simulator and the robot objects
    gazebo, robot = get_gazebo_and_robot(rtf=rtf,
                                         physics_rate=physics_rate,
                                         agent_rate=agent_rate,
                                         # controller_rate=None,  # Does not matter
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

    logger.debug(f"Simulating {cart_ref.size} steps")
    for i, ref in enumerate(cart_ref):
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

    # Terminate simulation
    gazebo.close()


def create_test_matrix() -> List[Tuple]:
    rtf_list = [0.5, 1, 2, 5, 10]
    agent_rate_list = [100, 500, 1000]
    physics_rate_list = [100, 500, 1000]

    matrix = list()

    for agent_rate in agent_rate_list:
        # Compute all the combinations of simulation rate and physics rate
        combinations = list(itertools.product(rtf_list, physics_rate_list))

        for rtf, physics_rate in combinations:
            # Remove some combination too demanding for a single process
            if physics_rate * rtf >= 2000:
                continue

            # This case is not allowed
            if agent_rate > physics_rate:
                continue

            matrix.append((rtf, agent_rate, physics_rate))

    print(matrix)
    return matrix


# @pytest.mark.xfail
@pytest.mark.parametrize("rtf, agent_rate, physics_rate", create_test_matrix())
def test_rates(rtf, agent_rate, physics_rate):

    print("========")
    print("Testing:")
    print("========")
    print("RTF = {}".format(rtf))
    print("Agent rate = {}".format(agent_rate))
    print("Physics rate = {}".format(physics_rate))

    template_test(rtf, physics_rate, agent_rate)
