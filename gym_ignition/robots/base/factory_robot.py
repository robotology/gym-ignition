# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import string
import random
import numpy as np
from typing import List, Union, Tuple
from gym_ignition.utils import logger, resource_finder
from gym_ignition.base.robot import robot_abc, robot_joints
from gym_ignition import gympp_bindings as bindings


class FactoryRobot(robot_abc.RobotABC,
                   robot_joints.RobotJoints):
    def __init__(self,
                 model_file: str,
                 gazebo: bindings.GazeboWrapper,
                 controller_rate: float = None) -> None:

        # Find the model file
        model_abs_path = resource_finder.find_resource(model_file)

        # Initialize the parent classes
        super().__init__()
        self.model_file = model_abs_path

        # Private attributes
        self._gazebo = gazebo
        self._robot_name = None
        self._gympp_robot = None
        self._controller_rate = controller_rate

        # Create a random prefix that will be used for the robot name
        letters_and_digits = string.ascii_letters + string.digits
        self._prefix = ''.join(random.choice(letters_and_digits) for _ in range(6))

    # ==============================
    # PRIVATE METHODS AND PROPERTIES
    # ==============================

    def delete_simulated_robot(self) -> None:
        # Remove the robot from the simulation
        ok_model = self._gazebo.removeModel(self._robot_name)
        assert ok_model, f"Failed to remove the model '{self._robot_name}' from gazebo"

    @property
    def gympp_robot(self):
        if self._gympp_robot:
            assert not self._gympp_robot.expired(), "The Robot object has expired"
            assert self._gympp_robot.lock(), "The Robot object is empty"
            assert self._gympp_robot.lock().valid(), "The Robot object is not valid"
            return self._gympp_robot.lock()

        # Find and load the model SDF file
        sdf_file = resource_finder.find_resource(self.model_file)
        with open(sdf_file, "r") as stream:
            sdf_string = stream.read()

        # Get the model name
        original_name = bindings.GazeboWrapper.getModelNameFromSDF(sdf_string)
        assert original_name, f"Failed to get model name from file {self.model_file}"

        # Create a unique robot name
        self._robot_name = self._prefix + "::" + original_name

        # Initialize the model data
        model_data = bindings.ModelInitData()
        model_data.setModelName(self._robot_name)
        model_data.setSdfString(sdf_string)
        model_data.setPosition([0., 0, 0])  # TODO: default initial position
        model_data.setOrientation([1., 0, 0, 0])  # TODO: default initial orientation

        # Initialize robot controller plugin
        plugin_data = bindings.PluginData()
        plugin_data.setLibName("RobotController")
        plugin_data.setClassName("gympp::plugins::RobotController")

        # Insert the model
        ok_model = self._gazebo.insertModel(model_data, plugin_data)
        assert ok_model, "Failed to insert the model"

        # Extract the robot from the singleton
        self._gympp_robot = bindings.RobotSingleton_get().getRobot(self._robot_name)

        # The robot is a weak pointer. Check that it is valid.
        assert not self._gympp_robot.expired(), "The Robot object has expired"
        assert self._gympp_robot.lock(), \
            "The returned Robot object does not contain a valid interface"
        assert self._gympp_robot.lock().valid(), "The Robot object is not valid"

        if self._controller_rate:
            logger.debug("Robot controller rate: {} Hz".format(self._controller_rate))
            ok_dt = self._gympp_robot.lock().setdt(1 / self._controller_rate)
            assert ok_dt, "Failed to set the robot controller period"

        logger.debug(f"IgnitionRobot '{self._gympp_robot.lock().name()}' added to the "
                     "simulation")
        return self._gympp_robot.lock()

    # ========
    # RobotABC
    # ========

    def name(self):
        return self.gympp_robot.name()

    def valid(self) -> bool:
        return self.gympp_robot.valid()

    # ===========
    # RobotJoints
    # ===========

    def dofs(self) -> None:
        raise NotImplementedError

    def joint_names(self) -> List[str]:
        return self.gympp_robot.jointNames()

    def joint_type(self, joint_name: str) -> robot_joints.JointType:
        raise NotImplementedError

    def joint_control_mode(self, joint_name: str) -> robot_joints.JointControlMode:
        raise NotImplementedError

    def set_joint_control_mode(self,
                               joint_name: str,
                               mode: robot_joints.JointControlMode) -> bool:
        raise NotImplementedError

    def initial_positions(self) -> np.ndarray:
        raise NotImplementedError

    def set_initial_positions(self, positions: np.ndarray) -> bool:
        raise NotImplementedError

    def joint_position(self, joint_name: str) -> float:
        return self.gympp_robot.jointPosition(joint_name)

    def joint_velocity(self, joint_name: str) -> float:
        return self.gympp_robot.jointVelocity(joint_name)

    def joint_positions(self) -> List[float]:
        return self.gympp_robot.jointPositions()

    def joint_velocities(self) -> List[float]:
        return self.gympp_robot.jointVelocities()

    def joint_pid(self, joint_name: str) -> Union[robot_joints.PID, None]:
        return self.gympp_robot.jointPID()

    def dt(self) -> float:
        return self.gympp_robot.dt()

    def set_dt(self, step_size: float) -> bool:
        return self.gympp_robot.setdt(step_size)

    def set_joint_force(self, joint_name: str, force: float, clip: bool = False) -> bool:
        return self.gympp_robot.setJointForce(joint_name, force)

    def set_joint_position(self, joint_name: str, position: float) -> bool:
        return self.gympp_robot.setJointPositionTarget(joint_name, position)

    def set_joint_velocity(self, joint_name: str, velocity: float) -> bool:
        return self.gympp_robot.setJointVelocityTarget(joint_name, velocity)

    def set_joint_interpolated_position(self, joint_name: str, position: float):
        raise NotImplementedError

    def set_joint_pid(self, joint_name: str, pid: robot_joints.PID) -> bool:
        return self.gympp_robot.setJointPID(joint_name, pid)

    def reset_joint(self,
                    joint_name: str,
                    position: float = None,
                    velocity: float = None) -> bool:
        return self.gympp_robot.resetJoint(joint_name, position, velocity)

    def update(self, dt: float):
        return self.gympp_robot.update(dt)

    def joint_position_limits(self, joint_name: str) -> Tuple[float, float]:
        raise NotImplementedError

    def joint_force_limit(self, joint_name: str) -> float:
        raise NotImplementedError
