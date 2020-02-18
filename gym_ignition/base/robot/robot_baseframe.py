# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
from typing import Tuple
from abc import ABC, abstractmethod


class RobotBaseFrame(ABC):
    """
    Robot base frame interface.

    This interface provides methods to get and set quantities related to the robot base.
    """

    def __init__(self) -> None:
        self._is_floating_base = True

    @abstractmethod
    def set_as_floating_base(self, floating: bool) -> bool:
        """
        Specify if the robot is floating or fixed base.

        Args:
            floating: True for floating base robots, False for fixed based robots.

        Returns:
            True if successful, False otherwise.
        """

    @abstractmethod
    def is_floating_base(self) -> bool:
        """
        Check the base type of the robot.

        Returns:
            True if the robot is floating base, False is the robot is fixed base.
        """

    @abstractmethod
    def set_base_frame(self, frame_name: str) -> bool:
        """
        Set the base frame of the robot. If this method is not called, the detection of
        the base frame depends on the implementation.

        Args:
            frame_name: The name of the base frame.

        Returns:
            True if successful, False otherwise.
        """

    @abstractmethod
    def base_frame(self) -> str:
        """
        Get the name of the base frame.

        Returns:
            The name of the base frame.
        """

    @abstractmethod
    def base_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return the pose of the robot base.

        The pose is composed of a position and a quaternion associated to the base frame,
        both expressed in the the world inertial frame.

        Returns:
            A Tuple containing the position and orientation of the base:

            - position: 3D array in the [x, y, z] form.
            - orientation: a 4D array containing a quaternion in the [w, x, y, z] form.
        """

    @abstractmethod
    def base_velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return the velocity of the robot base.

        The velocity is composed of linear and angular velocities of the base wrt the
        world inertial frame, both expressed in the world inertial frame.

        Returns:
            A Tuple containing the linear and angular velocity of the base:

            - linear velocity: a 3D array in the [vx, vy, vz] form.
            - angular velocity: a 3D array in the [wx, wy, wz] form.
        """

    @abstractmethod
    def reset_base_pose(self,
                        position: np.ndarray,
                        orientation: np.ndarray) -> bool:
        """
        Reset the pose of the robot base.

        The pose is composed of a position and a quaternion associated to the base frame,
        both expressed in the the world inertial frame.

        This method should be used only on simulated robots, and its effects bypass
        the physics.

        Args:
            position: 3D array in the [x, y, z] form
            orientation: a 4D array containing a quaternion in the [w, x, y, z] form.

        Returns:
            True if successful, False otherwise.
        """

    @abstractmethod
    def reset_base_velocity(self,
                            linear_velocity: np.ndarray,
                            angular_velocity: np.ndarray) -> bool:
        """
        Reset the velocity of the robot base.

        The velocity is composed of linear and angular velocities of the base wrt the
        world inertial frame, both expressed in the world inertial frame.

        This method should be used only on simulated robots, and its effects bypass
        the physics.

        Args:
            linear_velocity: a 3D array in the [vx, vy, vz] form.
            angular_velocity: a 3D array in the [wx, wy, wz] form.

        Returns:
            True if successful, False otherwise.
        """

    @abstractmethod
    def base_wrench(self) -> np.ndarray:
        """
        Get the wrench applied on the robot base (only for fixed-based setup)

        Returns:
            The 6D array of the base wrench expressed in the base frame.
        """
