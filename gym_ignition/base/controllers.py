# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import os.path
import numpy as np
from typing import List, NamedTuple
from gym_ignition.utils import logger
from gym_ignition.base.robot import RobotFeatures


class Controller:
    def __init__(self,
                 dt: float,
                 urdf: str,
                 robot: RobotFeatures,
                 controlled_joints: List[str]):

        self.dt = dt
        self.urdf = urdf
        self.robot = robot
        self.controlled_joints = controlled_joints

        if not robot.valid():
            raise Exception("The Robot object is not valid")

        if not self.controlled_joints:
            logger.debug("Controlling all robot joints")
            self.controlled_joints = self.robot.joint_names()

        if not os.path.isfile(urdf):
            raise Exception(f"The urdf file '{urdf}' does not exist")

    @property
    def nr_controlled_dofs(self) -> int:
        return len(self.controlled_joints)

    @abc.abstractmethod
    def initialize(self) -> bool:
        pass

    @abc.abstractmethod
    def step(self) -> np.ndarray:
        pass

    @abc.abstractmethod
    def terminate(self) -> bool:
        pass


class PositionControllerReferences(NamedTuple):
    position: np.ndarray                         # (1 x DoF)
    velocity: np.ndarray = None                  # (1 x DoF)
    acceleration: np.ndarray = None              # (1 x DoF)
    base_position: np.ndarray = np.zeros(3)      # (1 x 3)
    base_orientation: np.ndarray = np.zeros(4)   # (1 x 4)
    base_lin_velocity: np.ndarray = np.zeros(3)  # (1 x 3)
    base_ang_velocity: np.ndarray = np.zeros(3)  # (1 x 3)

    def flatten(self) -> np.ndarray:
        output = np.ndarray([], dtype=np.float)
        for _, value in self._asdict().items():
            if value:
                output = np.append(output, value)
        return output

    def valid(self) -> bool:
        dofs = self.position.size
        return \
            dofs > 0 and \
            self.velocity.size == self.acceleration.size == dofs and \
            self.base_position.size == 3 and \
            self.base_orientation.size == 4 and \
            self.base_lin_velocity.size == 3 and \
            self.base_ang_velocity.size == 3


class PositionController(Controller, abc.ABC):
    def __init__(self,
                 dt: float,
                 urdf: str,
                 robot: RobotFeatures,
                 controlled_joints: List[str],
                 floating_base: bool = False):

        self._is_floating_base = floating_base

        super().__init__(dt=dt,
                         urdf=urdf,
                         robot=robot,
                         controlled_joints=controlled_joints)

    @abc.abstractmethod
    def set_control_references(self, references: PositionControllerReferences) -> bool:
        pass
