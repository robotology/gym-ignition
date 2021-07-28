# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import itertools
import time
from typing import List, Tuple

import numpy as np
import pytest
from gym_ignition.base.robot.robot_joints import JointControlMode
from gym_ignition.utils import logger

from . import utils


def almost_equal(first, second, epsilon=None) -> bool:
    if not epsilon:
        epsilon = first * 0.05

    res = np.abs(first - second) <= epsilon

    if not res:
        print("----------------")
        print("ASSERTION FAILED")
        print("----------------")
        print(f"#1={first}")
        print(f"#2={second}")
        print("Error={}".format(np.abs(first - second)))
        print(f"Tolerance={epsilon}")
        print("----------------")

    return res


def template_test(
    rtf: float,
    physics_rate: float,
    agent_rate: float,
) -> None:

    # Get the simulator
    iterations = int(physics_rate / agent_rate)
    gazebo = utils.Gazebo(physics_rate=physics_rate, iterations=iterations, rtf=rtf)

    # Get the cartpole robot
    robot = utils.get_cartpole(gazebo)
    assert robot.valid(), "The robot object is not valid"

    # Configure the cartpole
    ok_cm = robot.set_joint_control_mode("linear", JointControlMode.POSITION)
    assert ok_cm, "Failed to set the control mode"

    # Set the joint controller period
    robot.set_dt(0.001 if physics_rate < 1000 else physics_rate)

    # Trajectory specifications
    trajectory_dt = 1 / agent_rate
    tot_simulated_seconds = 2

    # Generate the cart trajectory. This is analogous of setting an action containing
    # the cart references, hence it is related to the agent rate.
    cart_ref = np.fromiter(
        (
            0.2 * np.sin(2 * np.pi * 0.5 * t)
            for t in np.arange(0, tot_simulated_seconds, trajectory_dt)
        ),
        dtype=np.float,
    )

    avg_time_per_step = 0.0
    start = time.time()

    logger.debug(f"Simulating {cart_ref.size} steps")
    for i, ref in enumerate(cart_ref):
        # Set the references
        ok_ref = robot.set_joint_position("linear", ref)
        assert ok_ref, "Failed to set joint references"

        # Step the simulator
        now = time.time()
        gazebo.step()
        this_step = time.time() - now
        avg_time_per_step = avg_time_per_step + 0.1 * (this_step - avg_time_per_step)

    stop = time.time()
    elapsed_time = stop - start

    # Compute useful quantities
    number_of_actions = tot_simulated_seconds / trajectory_dt
    physics_iterations_per_run = physics_rate / agent_rate
    simulation_dt = 1 / (physics_rate * rtf)

    assert number_of_actions == cart_ref.size

    print()
    print(f"Elapsed time = {elapsed_time}")
    print(f"Average step time = {avg_time_per_step}")
    print("Number of actions: = {}".format(tot_simulated_seconds / trajectory_dt))
    print(f"Physics iteration per simulator step = {physics_iterations_per_run}")
    print()

    # Check if the average time per step matches what expected
    assert almost_equal(
        first=avg_time_per_step,
        second=simulation_dt * physics_iterations_per_run,
        epsilon=avg_time_per_step * 0.5,
    )

    # Check if the total time of the simulation matched what expected
    assert almost_equal(
        first=elapsed_time,
        second=number_of_actions * simulation_dt * physics_iterations_per_run,
        epsilon=elapsed_time * 0.5,
    )

    # Terminate simulation
    gazebo.close()


def create_test_matrix() -> List[Tuple[float, float, float]]:
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


@pytest.mark.parametrize("rtf, agent_rate, physics_rate", create_test_matrix())
def test_rates(rtf: float, agent_rate: float, physics_rate: float):

    print("========")
    print("Testing:")
    print("========")
    print(f"RTF = {rtf}")
    print(f"Agent rate = {agent_rate}")
    print(f"Physics rate = {physics_rate}")

    template_test(rtf, physics_rate, agent_rate)
