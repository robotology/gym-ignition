# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
import numpy as np
from gym_ignition import gympp_bindings as bindings
from gym_ignition.utils import logger, resource_finder


def test_joint_controller():
    # ==========
    # PARAMETERS
    # ==========

    agent_rate = 100.0
    physics_rate = 1000.0
    controller_rate = 500.0

    plugin_data = bindings.PluginData()
    plugin_data.libName = "RobotController"
    plugin_data.className = "scenario::plugins::gazebo::RobotController"

    # Find and load the model SDF file
    model_sdf_file = resource_finder.find_resource("CartPole/CartPole.urdf")
    with open(model_sdf_file, "r") as stream:
        model_sdf_string = stream.read()

    # Initialize the model data
    model_data = bindings.ModelInitData()
    model_data.sdfString = model_sdf_string

    # Get the model name
    model_name = bindings.GazeboSimulator.getModelNameFromSDF(model_sdf_string)
    robot_name = model_name

    # =============
    # CONFIGURATION
    # =============

    # Create the gazebo wrapper
    num_of_iterations = int(physics_rate / agent_rate)
    desired_rtf = float(np.finfo(np.float32).max)
    gazebo = bindings.GazeboSimulator(num_of_iterations, desired_rtf, physics_rate)
    assert gazebo, "Failed to get the gazebo wrapper"

    # Set the verbosity
    logger.set_level(gym.logger.DEBUG)

    # Initialize the world
    world_ok = gazebo.setupGazeboWorld("DefaultEmptyWorld.world")
    assert world_ok, "Failed to initialize the gazebo world"

    # Initialize the ignition gazebo wrapper (creates a paused simulation)
    gazebo_initialized = gazebo.initialize()
    assert gazebo_initialized, "Failed to initialize ignition gazebo"

    # Insert the model
    model_ok = gazebo.insertModel(model_data, plugin_data)
    assert model_ok, "Failed to insert the model in the simulation"

    assert bindings.RobotSingleton_get().exists(
        robot_name
    ), "The robot interface was not registered in the singleton"

    # Extract the robot interface from the singleton
    robot_weak_ptr = bindings.RobotSingleton_get().getRobot(robot_name)
    assert not robot_weak_ptr.expired(), "The Robot object has expired"
    assert (
        robot_weak_ptr.lock()
    ), "The returned Robot object does not contain a valid interface"
    assert robot_weak_ptr.lock().valid(), "The Robot object is not valid"

    # Get the pointer to the robot interface
    robot = robot_weak_ptr.lock()

    # Set the default update rate
    robot.setdt(1 / controller_rate)

    # Control the cart joint in position
    ok_cm = robot.setJointControlMode("linear", bindings.JointControlMode_Position)
    assert ok_cm, "Failed to control the cart joint in position"

    # Set the PID of the cart joint
    pid = bindings.PID(10000, 1000, 1000)
    pid_ok = robot.setJointPID("linear", pid)
    assert pid_ok, "Failed to set the PID of the cart joint"

    # Reset the robot state
    robot.resetJoint("pivot", 0, 0)
    robot.resetJoint("linear", 0, 0)

    # Generate the cart trajectory
    cart_ref = np.fromiter(
        (0.2 * np.sin(2 * np.pi * 0.5 * t) for t in np.arange(0, 5, 1 / agent_rate)),
        dtype=np.float,
    )

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
    assert (
        np.abs(pos_cart_buffer - cart_ref).sum() / cart_ref.size < 5e-3
    ), "The reference trajectory was not tracked correctly"
