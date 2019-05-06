#!/usr/bin/env python3

# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
import numpy as np
import gym_ignition
from numpy import pi, sin
from gym_ignition import gympp
from gym_ignition.utils import logger


# Set gym verbosity
logger.set_level(gym.logger.WARN)


def test_create_cpp_environment():
    env = gym.make("CartPoleIgnition-v0")
    assert env, "Failed to create CartPoleIgnition-v0 environment"

    action = env.action_space.sample()
    assert isinstance(action, int), "The sampled action is empty"

    observation = env.observation_space.sample()
    assert observation.size > 0, "The sampled observation is empty"

    observation = env.reset()
    assert observation.size > 0, "The observation is empty"

    state, reward, done, _ = env.step(action)
    assert state.size > 0, "The environment didn't return a valid state"


def test_create_python_environment():
    env = gym.make("CartPoleIgnitionPython-v0")
    assert env, "Failed to create CartPoleIgnitionPython-v0 environment"

    action = env.action_space.sample()
    assert isinstance(action, int), "The sampled action is empty"

    observation = env.observation_space.sample()
    assert observation.size > 0, "The sampled observation is empty"

    observation = env.reset()
    assert observation.size > 0, "The observation is empty"

    state, reward, done, _ = env.step(action)
    assert state.size > 0, "The environment didn't return a valid state"


def test_joint_controller():
    lib_name = "RobotController"
    class_name = "gympp::plugins::RobotController"

    model_sdf = "CartPole/CartPole.sdf"
    world_sdf = "CartPole.world"

    agent_rate = 100.0
    physics_rate = 1000.0
    controller_rate = 500.0

    physics_iterations = 10  # TODO: Computed already inside the class

    # Create the gazebo wrapper
    gazebo = gym_ignition.gympp.GazeboWrapper(physics_rate, physics_iterations)
    assert gazebo, "Failed to get the gazebo wrapper"

    # Set verbosity
    logger.set_level(gym.logger.WARN)

    # Initialize the world
    world_ok = gazebo.setupGazeboWorld(world_sdf)
    assert world_ok, "Failed to initialize the gazebo world"

    # Initialize the model
    model_ok = gazebo.setupGazeboModel(model_sdf)
    assert model_ok, "Failed to initialize the gazebo model"

    # Initialize the plugin
    wrapper_ok = gazebo.setupIgnitionPlugin(lib_name, class_name, agent_rate)
    assert wrapper_ok, "Failed to setup the ignition plugin"

    # Initialize the ignition gazebo wrapper
    gazebo_initialized = gazebo.initialize()
    assert gazebo_initialized, "Failed to initialize ignition gazebo"

    # Get the robot name
    model_names = gazebo.getModelNames()
    assert len(model_names) == 1, "The environment has more than one model"
    model_name = model_names[0]

    # Get the robot object
    robot = gympp.RobotSingleton_get().getRobot(model_name)
    assert robot, "Failed to get the Robot object"
    assert robot.valid(), "The Robot object is not valid"

    # Set the default update rate
    robot.setdt(1 / controller_rate)

    # Set the PID of the cart joint
    pid = gympp.PID(10000, 1000, 1000)
    pid_ok = robot.setJointPID("linear", pid)
    assert pid_ok, "Failed to set the PID of the cart joint"

    # Reset the robot state
    robot.resetJoint("pivot", 0, 0)
    robot.resetJoint("linear", 0, 0)

    # Generate the cart trajectory
    cart_ref = np.fromiter(
        (0.2 * sin(2 * pi * 0.5 * t) for t in np.arange(0, 5, 1 / agent_rate)),
        dtype=np.float)

    # Initialize the cart position buffer
    pos_cart_buffer = np.zeros(np.shape(cart_ref))

    for (i, ref) in enumerate(cart_ref):
        # Set the references
        ok1 = robot.setJointPositionTarget("linear", ref)
        assert ok1, "Failed to set joint references"

        # Step the simulator
        gazebo.run()

        # Read the position
        pos_cart_buffer[i] = robot.jointPosition("linear")

    # Check that the trajectory was followed correctly
    assert np.abs(pos_cart_buffer - cart_ref).sum() / cart_ref.size < 5E-3,\
        "The reference trajectory was not tracked correctly"
