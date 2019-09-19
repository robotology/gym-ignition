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

    def __init__(self) -> None: ...

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
        Returns the pose of the robot base.

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
            - angular velocity: a 3D array in the [wx, wy, wx] form.
        """

    @abstractmethod
    def reset_base_pose(self,
                        position: np.ndarray,
                        orientation: np.ndarray,
                        floating: bool = False) -> bool:
        """
        Reset the pose of the robot base.

        The pose is composed of a position and a quaternion associated to the base frame,
        both expressed in the the world inertial frame.

        This method should be used only on simulated robots, and its effects bypass
        the physics.

        Args:
            position: 3D array in the [x, y, z] form
            orientation: a 4D array containing a quaternion in the [w, x, y, z] form.
            floating: a boolean flag that, if false, fixes the base in the configured
                position.

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
            angular_velocity: a 3D array in the [wx, wy, wx] form.

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
