# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym_ignition.base.robot.robot_joints import JointControlMode

from . import utils


def test_joint_force():
    # Get the simulator
    gazebo = utils.Gazebo(physics_rate=1000, iterations=1)

    # Insert the robot
    pendulum = utils.get_pendulum(simulator=gazebo)

    # Configure the robot
    ok_mode = pendulum.set_joint_control_mode("pivot", JointControlMode.TORQUE)
    assert ok_mode

    # Step the simulator
    gazebo.step()

    # No references set
    assert pendulum.joint_force("pivot") == 0.0

    # Step the simulator
    gazebo.step()

    # Set a reference. No force reference yet applied.
    torque = 42.42
    pendulum.set_joint_force("pivot", torque)
    assert pendulum.joint_force("pivot") == 0.0

    # Step the simulator. Now the force reference should have been actuated and after
    # the physics step it should be returned by the method.
    gazebo.step()
    assert pendulum.joint_force("pivot") == torque

    # Step again the simulator. No force reference has been specified and the method
    # should return zero.
    gazebo.step()
    assert pendulum.joint_force("pivot") == 0.0

    # Close the simulator
    gazebo.close()


def test_joint_force_multiple_iterations():
    # Get the simulator
    gazebo = utils.Gazebo(physics_rate=1000, iterations=2)

    # Insert the robot
    pendulum = utils.get_pendulum(simulator=gazebo)
    pendulum.set_joint_control_mode("pivot", JointControlMode.TORQUE)

    # Step the simulator
    gazebo.step()

    # No references set
    assert pendulum.joint_force("pivot") == 0.0

    # Step the simulator
    gazebo.step()

    # Set a reference. No force reference yet applied.
    torque = 42.42
    pendulum.set_joint_force("pivot", torque)
    assert pendulum.joint_force("pivot") == 0.0

    # Step the simulator.
    # Note that since gazebo was configured with multiple iterations,
    # the force is applied only in the first one and we still read zero!
    gazebo.step()
    assert pendulum.joint_force("pivot") == 0.0

    # Step again the simulator
    gazebo.step()
    assert pendulum.joint_force("pivot") == 0.0

    # Close the simulator
    gazebo.close()
