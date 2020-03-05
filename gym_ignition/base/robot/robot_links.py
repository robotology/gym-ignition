# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
from typing import List, Tuple
from abc import ABC, abstractmethod


class RobotLinks(ABC):
    """
    Robot links interface.

    This interface provides methods to get and set link-related quantities.
    """

    def __init__(self) -> None: ...

    @abstractmethod
    def link_names(self) -> List[str]:
        """
        Return a list of link names. The order matches link indices.

        Returns:
            The link names.
        """

    @abstractmethod
    def link_pose(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return the pose of the specified link.

        The pose is composed of a position and a quaternion associated to the link frame,
        both expressed in the the world frame.

        Args:
            link_name: The name of the link.

        Returns:
            A Tuple containing the position and orientation of the link:

            - position: 3D array in the [x, y, z] form.
            - orientation: a 4D array containing a quaternion in the [w, x, y, z] form.
        """

    @abstractmethod
    def link_velocity(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return the velocity of the specified link.

        The velocity is composed of linear velocity (first three elements) and angular
        velocity (last three elements) of the link frame.
        The linear velocity is the time derivative of the position of the link frame
        origin expressed in the world frame.
        The angular velocity is expressed with the orientation of the world frame.

        Args:
            link_name: The name of the link.

        Returns:
            A Tuple containing the linear and angular velocity of the link:

            - linear velocity: a 3D array in the [vx, vy, vz] form.
            - angular velocity: a 3D array in the [wx, wy, wz] form.
        """

    @abstractmethod
    def link_acceleration(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return the acceleration of the specified link.

        The acceleration is composed of linear acceleration (first three elements) and
        angular acceleration (last three elements) of the link frame.
        The linear acceleration is the second time derivative of the position of the link
        frame origin expressed in the world frame.
        The angular acceleration is expressed with the orientation of the world frame.

        Args:
            link_name: The name of the link.

        Returns:
            A Tuple containing the linear and angular acceleration of the link:

            - linear acceleration: a 3D array in the [ax, ay, az] form.
            - angular acceleration: a 3D array in the [wdotx, wdoty, wdotz] form.
        """

    @abstractmethod
    def apply_external_force(self,
                             link_name: str,
                             force: np.ndarray,
                             torque: np.ndarray) -> bool:
        """
        Apply external force and torque to the link.

        The force and the torque are applied at the link origin.
        The force is expressed in the world frame.
        The torque is expressed with the orientation of the world frame.

        The external force and torque are only applied for the next simulation step,
        and then they are automatically reset to zero. To apply a force for more then one
        step, call `apply_external_force` before each step.

        Args:
            link_name: The name of the link.
            force: A 3D array containing the force vector.
            torque: A 3D array containing the torque vector.

        Returns:
            True if successful, False otherwise.
        """