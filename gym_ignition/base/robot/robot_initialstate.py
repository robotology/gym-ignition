# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
from typing import Tuple
from abc import ABC, abstractmethod


class RobotInitialState(ABC):
    """
    Robot initial state interface.

    This interface provides methods to get and set the initial state of the joints and
    the base.
    """

    def __init__(self) -> None:
        self._initial_joint_positions = None
        self._initial_joint_velocities = None
        self._initial_base_pose = (np.array([0., 0., 0.]), np.array([1., 0., 0., 0.]))
        self._initial_base_velocity = (np.array([0., 0., 0.]), np.array([0., 0., 0.]))

    @abstractmethod
    def initial_joint_positions(self) -> np.ndarray:
        """
        Return the initial joint positions.

        Returns:
            The initial joint positions.
        """

    @abstractmethod
    def set_initial_joint_positions(self, positions: np.ndarray) -> bool:
        """
        Set the initial joint positions.

        Note that this does not actuate the robot, it only stores the initial joint
        positions values.

        Args:
            positions: The initial joint positions.

        Returns:
             True if successful, False otherwise.
        """

    @abstractmethod
    def initial_joint_velocities(self) -> np.ndarray:
        """
        Return the initial joint velocities.

        Returns:
            The initial joint velocities.
        """

    @abstractmethod
    def set_initial_joint_velocities(self, velocities: np.ndarray) -> bool:
        """
        Set the initial joint velocities.

        Note that this does not actuate the robot, it only stores the initial joint
        velocities values.

        Args:
            velocities: The initial joint velocities.

        Returns:
             True if successful, False otherwise.
        """

    @abstractmethod
    def initial_base_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return the initial base pose.

        Returns:
            A tuple containing the the initial position and orientation of the base:

            - position: 3D array in the [x, y, z] form.
            - orientation: a 4D array containing a quaternion in the [w, x, y, z] form.
        """

    @abstractmethod
    def set_initial_base_pose(self, position: np.ndarray,
                              orientation: np.ndarray) -> bool:
        """
         Set the initial base pose.

         Note that this does not actuate the robot, it only stores the initial base
         pose values.

         Args:
             position: The initial base position expressed as a 3D array.
             orientation: The initial base orientation expressed as a wxyz quaternion.

         Returns:
              True if successful, False otherwise.
         """

    @abstractmethod
    def initial_base_velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return the initial base velocity.

        Returns:
            A tuple containing the the linear and angular initial velocity of the base:

            - linear: 3D array in the [vx, vy, vz] form.
            - angular: a 3D array in the [wx, wy, wz] form.
        """

    @abstractmethod
    def set_initial_base_velocity(self, linear: np.ndarray, angular: np.ndarray) -> bool:
        """
         Set the initial base velocity.

         Note that this does not actuate the robot, it only stores the initial base
         pose values.

         Args:
             linear: The linear initial velocity expressed as a 3D array.
             angular: The angular initial velocity expressed as a 3D array.

         Returns:
              True if successful, False otherwise.
         """
