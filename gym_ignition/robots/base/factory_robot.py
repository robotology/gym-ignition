# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import List
from gym_ignition import gympp_bindings as bindings
from gym_ignition.base import robot


class FactoryRobot(robot.Robot):
    def __init__(self, robot_name: str, controller_rate: float = None) -> None:
        super().__init__()

        self._gympp_robot = None
        self._robot_name = robot_name
        self._controller_rate = controller_rate

        robot_ok = self.gympp_robot
        assert robot_ok, "Failed to create robot"

    @property
    def gympp_robot(self):
        if self._gympp_robot:
            assert self._gympp_robot.valid(), "The Robot object is not valid"
            return self._gympp_robot

        assert self._robot_name, "Robot name was not set"
        self._gympp_robot = bindings.RobotSingleton_get().getRobot(self._robot_name)

        assert self._gympp_robot, "Failed to get the Robot object"
        assert self._gympp_robot.valid(), "The Robot object is not valid"

        if self._controller_rate:
            ok_dt = self._gympp_robot.setdt(1 / self._controller_rate)
            assert ok_dt, "Failed to set the robot controller period"

        return self._gympp_robot

    def valid(self) -> bool:
        return self.gympp_robot.valid()

    def joint_names(self) -> List[str]:
        return self.gympp_robot.jointNames()

    def joint_position(self, joint_name: str) -> float:
        return self.gympp_robot.jointPosition(joint_name)

    def joint_velocity(self, joint_name: str) -> float:
        return self.gympp_robot.jointVelocity(joint_name)

    def joint_positions(self) -> List[float]:
        return self.gympp_robot.jointPositions()

    def joint_velocities(self) -> List[float]:
        return self.gympp_robot.jointVelocities()

    def dt(self) -> float:
        return self.gympp_robot.dt()

    def joint_pid(self) -> robot.PID:
        return self.gympp_robot.jointPID()

    def set_dt(self, step_size: float) -> bool:
        return self.gympp_robot.setdt(step_size)

    def set_joint_force(self, joint_name: str, force: float) -> bool:
        return self.gympp_robot.setJointForce(joint_name, force)

    def set_joint_position_target(self, joint_name: str, position_reference: float) -> \
            bool:
        return self.gympp_robot.setJointPositionTarget(joint_name, position_reference)

    def set_joint_velocity_target(self, joint_name: str, velocity_reference: float) -> \
            bool:
        return self.gympp_robot.setJointVelocityTarget(joint_name, velocity_reference)

    def set_joint_position(self, joint_name: str, position: float) -> bool:
        return self.gympp_robot.setJointPosition(joint_name, position)

    def set_joint_velocity(self, joint_name: str, velocity: float) -> bool:
        return self.gympp_robot.setJointVelocity(joint_name, velocity)

    def set_joint_pid(self, joint_name: str, pid: robot.PID) -> bool:
        return self.gympp_robot.setJointPID(joint_name, pid)

    def reset_joint(self, joint_name: str, position: float, velocity: float) -> bool:
        return self.gympp_robot.resetJoint(joint_name, position, velocity)

    def update(self, dt: float):
        return self.gympp_robot.update(dt)
