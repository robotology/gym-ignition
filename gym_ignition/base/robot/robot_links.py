# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
from typing import List
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
    def link_pose(self, link_name: str) -> np.ndarray:
        """
        Return the pose of the specified link.

        The pose is composed of a position and a quaternion associated to the link frame,
        both expressed in the the robot base frame.

        Args:
            link_name: The name of the link

        Returns:
            A Tuple containing the position and orientation of the base:

            - position: 3D array in the [x, y, z] form.
            - orientation: a 4D array containing a quaternion in the [w, x, y, z] form.
        """
