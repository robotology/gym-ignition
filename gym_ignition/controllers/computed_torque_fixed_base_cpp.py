# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import numpy as np
from typing import List
import gympp_bindings as bindings
from gym_ignition import base
from gym_ignition.utils import logger, resource_finder
from gym_ignition.base.robot import robot_abc, robot_joints
from gym_ignition.base.controllers import PositionController, PositionControllerReferences


@base.robot.feature_detector
class RobotFeatures(robot_abc.RobotABC,
                    robot_joints.RobotJoints,
                    abc.ABC):
    pass


class ComputedTorqueFixedBaseCpp(PositionController):

    def __init__(self,
                 robot: RobotFeatures,
                 urdf: str,
                 controlled_joints: List[str],
                 kp: np.ndarray,
                 kd: np.ndarray,
                 # dt: float = None,
                 clip_torques: bool = False,
                 **kwargs) -> None:

        # Find the urdf file
        abs_path_urdf = resource_finder.find_resource(urdf)

        # Initialize base class
        super().__init__(dt=0.0,
                         robot=robot,
                         urdf=abs_path_urdf,
                         controlled_joints=controlled_joints)

        # Check that the robot has all the requested features
        assert robot.valid(), "The robot object is not valid"
        RobotFeatures.has_all_features(robot)

        # Make sure that the robot object is a C++ robot
        try:
            robot.gympp_robot
        except AttributeError:
            logger.error("This controller can be used only with C++ robots objects")
            raise

        # Prepare C++ controller arguments
        robot_cpp = robot.gympp_robot

        # Create the C++ controller
        self._controller = bindings.ComputedTorqueFixedBase(
            urdf, robot_cpp, kp, kd, controlled_joints)

        # Initialize other attributes
        self._clip_torques = clip_torques
        self._references = bindings.PositionControllerReferences(len(controlled_joints))

    # ==================
    # PositionController
    # ==================

    def set_control_references(self, references: PositionControllerReferences) -> bool:
        assert references.valid(), "References are not valid"

        # Populate the C++ class with the references
        self._references.joints.position = references.position
        self._references.joints.velocity = references.velocity
        self._references.joints.acceleration = references.acceleration

        assert self._references.valid(), "Stored references are not valid"

        # Insert the references in the controller
        ok_refs = self._controller.setReferences(self._references)
        assert ok_refs, "Failed to set references in the controller"

        return True

    # ==========
    # Controller
    # ==========

    def initialize(self) -> bool:
        ok_initialize = self._controller.initialize()
        assert ok_initialize, "Failed to initialize cpp controller"
        return True

    def step(self) -> np.ndarray:
        torques_optional = self._controller.step()
        assert torques_optional.has_value(), "Failed to step the controller"

        # Get the torques from the optional type
        torques = np.array(torques_optional.value())

        # Clip to the maximum torque allowed
        if self._clip_torques:
            for idx, joint_name in enumerate(self.controlled_joints):
                torques[idx] = \
                    np.min([torques[idx], self.robot.joint_torque_limits(joint_name)])

        return torques

    def terminate(self) -> bool:
        ok_terminate = self._controller.terminate()
        assert ok_terminate, "Failed to terminate the controller"
        return True
