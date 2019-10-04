# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc


class RobotABC(abc.ABC):
    """
    The base interface of all robot objects
    """

    def __init__(self, model_file: str = None) -> None:
        # Optional model file associated with this object
        self.model_file = model_file

    @abc.abstractmethod
    def name(self) -> str: ...

    @abc.abstractmethod
    def valid(self) -> bool: ...
