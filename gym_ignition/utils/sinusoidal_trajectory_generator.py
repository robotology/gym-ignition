# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import numpy as np
from typing import Union
from gym_ignition.base.robot import robot_joints, robot_abc, feature_detector


@feature_detector
class RobotFeatures(robot_abc.RobotABC, robot_joints.RobotJoints, abc.ABC):
    pass


class SinusoidalTrajectoryGenerator:
    def __init__(self,
                 dt: float,
                 robot: RobotFeatures,
                 initial_positions: np.ndarray,
                 f: float,
                 sine_amplitude: Union[np.ndarray, float]):
        self.t = -dt
        self.f = f
        self.dt = dt
        self.dofs = initial_positions.size
        self.positions = initial_positions

        assert robot.valid(), "Robot is not valid"

        # Get the joint position limits
        self.min_pos, self.max_pos = [], []
        for joint_name in robot.joint_names():
            min_lim, max_lim = robot.joint_position_limits(joint_name)
            self.min_pos.append(min_lim)
            self.max_pos.append(max_lim)

        # Different amplitude for each joint
        if isinstance(sine_amplitude, np.ndarray):
            if sine_amplitude.size is not self.dofs:
                raise Exception("Wrong number of joint sine amplitudes")
            self.amplitudes = sine_amplitude
        # Same amplitude for all joints
        elif isinstance(sine_amplitude, float):
            self.amplitudes = [sine_amplitude] * self.dofs
        else:
            raise Exception(f"Wrong data type: {type(sine_amplitude)}")

    def _clip(self, position):
        return np.clip(position, self.min_pos, self.max_pos)

    def get_references(self) -> np.ndarray:
        self.t += self.dt
        pos = self.positions + self.amplitudes * np.sin(2 * np.pi * self.f * self.t)
        return self._clip(pos)