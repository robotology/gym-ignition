# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import List
from abc import ABC, abstractmethod


class PID():
    def __init__(self, p : float, i: float, d: float) -> None:
        self.p = p
        self.i = i
        self.d = d


class Robot(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def valid(self) -> bool:
        pass

    @abstractmethod
    # TODO numpy?
    def joint_names(self) -> List[str]:
        pass

    @abstractmethod
    def joint_position(self, joint_name: str) -> float:
        pass

    @abstractmethod
    def joint_velocity(self, joint_name: str) -> float:
        pass

    @abstractmethod
    def joint_positions(self) -> List[float]:
        pass

    @abstractmethod
    def joint_velocities(self) -> List[float]:
        pass

    @abstractmethod
    def dt(self) -> float:
        pass

    @abstractmethod
    def joint_pid(self) -> PID:
        pass

    @abstractmethod
    def set_dt(self, step_size: float) -> bool:
        pass

    @abstractmethod
    def set_joint_force(self, joint_name: str, force: float) -> bool:
        pass

    @abstractmethod
    def set_joint_position_target(self, joint_name: str, position_reference: float) -> \
            bool:
        pass

    @abstractmethod
    def set_joint_velocity_target(self, joint_name: str, velocity_reference: float) -> \
            bool:
        pass

    @abstractmethod
    def set_joint_position(self, joint_name: str, position: float) -> bool:
        pass

    @abstractmethod
    def set_joint_velocity(self, joint_name: str, velocity: float) -> bool:
        pass

    @abstractmethod
    def set_joint_pid(self, joint_name: str, pid: PID) -> bool:
        pass

    @abstractmethod
    def reset_joint(self, joint_name: str, position: float, velocity: float) -> bool:
        pass

    @abstractmethod
    def update(self, dt: float):
        pass
