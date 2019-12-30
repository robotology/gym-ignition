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
            link_name: The name of the link

        Returns:
            A Tuple containing the position and orientation of the link:

            - position: 3D array in the [x, y, z] form.
            - orientation: a 4D array containing a quaternion in the [w, x, y, z] form.
        """

    @abstractmethod
    def link_velocity(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return the velocity of the specified link.

        The velocity is composed of linear and angular velocities of the link frame,
        both expressed in the world frame.

        Args:
            link_name: The name of the link

        Returns:
            A Tuple containing the linear and angular velocity of the link:

            - linear velocity: a 3D array in the [vx, vy, vz] form.
            - angular velocity: a 3D array in the [wx, wy, wz] form.
        """

    @abstractmethod
    def link_acceleration(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return the acceleration of the specified link.

        The acceleration is composed of linear and angular accelerations of the link
        frame, both expressed in the world frame.

        Args:
            link_name: The name of the link

        Returns:
            A Tuple containing the linear and angular acceleration of the link:

            - linear acceleration: a 3D array in the [ax, ay, az] form.
            - angular acceleration: a 3D array in the [wdotx, wdoty, wdotz] form.
        """
