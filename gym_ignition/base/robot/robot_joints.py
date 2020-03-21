# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
from enum import Enum, auto
from abc import ABC, abstractmethod
from typing import List, Tuple, Union


class JointType(Enum):
    INVALID = auto()
    FIXED = auto()
    REVOLUTE = auto()
    PRISMATIC = auto()

class JointControlMode(Enum):
    POSITION = auto()
    POSITION_INTERPOLATED = auto()
    VELOCITY = auto()
    TORQUE = auto()


class PID:
    def __init__(self, p: float, i: float, d: float) -> None:
        self.p = p
        self.i = i
        self.d = d


class RobotJoints(ABC):
    """
    Robot joints interface.

    This interface provides methods to get and set joint-related quantities.
    """

    @abstractmethod
    def dofs(self) -> int:
        """
        Return the number of degrees of freedom of the robot.

        Returns:
            The number of degrees of freedom.
        """

    @abstractmethod
    def joint_names(self) -> List[str]:
        """
        Return a list of joint names. The order matches joints indices.

        Returns:
            The joint names.
        """

    @abstractmethod
    def joint_type(self, joint_name: str) -> JointType:
        """
        Return the type of the specified joint.

        Args:
            joint_name: The name of the joint.

        Returns:
            The joint type.
        """

    @abstractmethod
    def joint_control_mode(self, joint_name: str) -> JointControlMode:
        """
        Return the control mode of the specified joint.

        Args:
            joint_name: The name of the joint.

        Returns:
             The joint control mode.
        """

    @abstractmethod
    def set_joint_control_mode(self, joint_name: str, mode: JointControlMode) -> bool:
        """
        Set the control mode of the specified joint.

        Args:
            joint_name: The name of the joint.
            mode: The joint control mode.

        Returns:
             True if successful, False otherwise.
        """

    @abstractmethod
    def joint_position(self, joint_name: str) -> float:
        """
        Return the generalized position of the specified joint.

        Args:
            joint_name: The name of the joint.

        Returns:
            The joint generalized position.

        """

    @abstractmethod
    def joint_velocity(self, joint_name: str) -> float:
        """
        Return the generalized velocity of the specified joint.

        Args:
            joint_name: The name of the joint.

        Returns:
            The joint generalized velocity.

        """

    @abstractmethod
    def joint_force(self, joint_name: str) -> float:
        """
        Return the joint force applied to the specified joint in the last physics step.

        The returned value is the effort or torque of the joint, since currently only
        1 DoF joints are supported.

        Note that the returned value depends on the rate of the enabled controllers.
        For example, the JointControlMode.POSITION allows to specify a controller rate
        different than the physics rate. In this case, this method will return only the
        last applied force reference. If the PIDs are running faster than the code
        that calls this `joint_force` method, the PIDs are generating forces that
        are not returned.

        Args:
            joint_name: The name of the joint.

        Returns:
            The generalized joint force reference applied in the last physics step.
        """

    @abstractmethod
    def joint_positions(self) -> np.ndarray:
        """
        Return the generalized positions of the joints.

        The order matches :meth:`joint_names`.

        Returns:
            The joint generalized positions.
        """

    @abstractmethod
    def joint_velocities(self) -> np.ndarray:
        """
        Return the generalized velocities of the joints.

        The order matches :meth:`joint_names`.

        Returns:
            The joint generalized velocities.
        """

    @abstractmethod
    def joint_pid(self, joint_name: str) -> Union[PID, None]:
        """
        Return the PID associated to the specified joint.

        Args:
            joint_name: The joint name.

        Returns:
            The low-level joint PID or None if no PID was configured.
        """

    @abstractmethod
    def dt(self) -> float:
        """
        Returns the step_size of the low-level PID controllers.

        Returns:
            The PIDs step_size.
        """

    @abstractmethod
    def set_dt(self, step_size: float) -> bool:
        """
        Set the step size used by the low-level PIDs.

        Args:
            step_size: The PIDs step size.

        Returns:
            True if successful, False otherwise.
        """

    @abstractmethod
    def set_joint_force(self, joint_name: str, force: float, clip: bool = False) -> bool:
        """
        Set the generalized force applied to the specified joint.

        If the control mode of the joint is not JointControlMode.TORQUE, this method
        returns `False`.

        Args:
            joint_name: The name of the joint.
            force: The joint generalize force.
            clip: Clip joint forces reading effort limits.

        Returns:
            True if successful, False otherwise.
        """

    @abstractmethod
    def set_joint_position(self, joint_name: str, position: float) -> bool:
        """
        Set the desired position of the specified joint.

        This method sends a new position reference to the joint PID controller.

        If the control mode of the joint is not JointControlMode.POSITION, this method
        returns `False`.

        Args:
            joint_name: The name of the joint.
            position: The position reference of the joint.

        Returns:
            True if successful, False otherwise.
        """

    @abstractmethod
    def set_joint_velocity(self, joint_name: str, velocity: float) -> bool:
        """
        Set the desired velocity of the specified joint.

        This method sends a new velocity reference to the joint PID controller.

        If the control mode of the joint is not JointControlMode.VELOCITY, this method
        returns `False`.

        Args:
            joint_name: The name of the joint.
            velocity: The velocity reference of the joint.

        Returns:
            True if successful, False otherwise.
        """

    def set_joint_interpolated_position(self, joint_name: str, position: float):
        """
        Set the position reference of the joint trajectory generator.

        This method sends a new position reference to the trajectory generator,
        which smooths the transient with an interpolation strategy. Refer to the
        documentation of the implementation for details about the trajectory generator.

        If the control mode of the joint is not JointControlMode.POSITION_INTERPOLATED,
        this method returns `False`.

        Args:
            joint_name: The name of the joint.
            position: The position reference of the joint trajectory generator.

        Returns:
            True if successful, False otherwise.
        """

    @abstractmethod
    def set_joint_pid(self, joint_name: str, pid: PID) -> bool:
        """
        Set the low-level PID associated to the specified joint.

        It can be either a position or a velocity PID. This method should be called
        after setting the joint control mode.

        If the control mode of the joint is not either `POSITION` or `VELOCITY` this
        method returns `False`.

        Args:
            joint_name: The name of the joint.
            pid: The joint PID.

        Returns:
            True if successful, False otherwise.
        """

    @abstractmethod
    def reset_joint(self,
                    joint_name: str,
                    position: float = None,
                    velocity: float = None) -> bool:
        """
        Reset the position and velocity of a joint.

        On simulated robots this method bypasses the physics.
        On real robots the usage of this method is undefined and depends on the
        implementation. Refer to the documentation of the implementation for more
        details.

        Args:
            joint_name: The name of the joint.
            position: The position of the joint.
            velocity: The velocity of the joint.

        Returns:
            True if successful, False otherwise.
        """

    @abstractmethod
    def update(self, current_time: float) -> bool:
        """
        Update the low-level joint PIDs.

        This method steps the joint PIDs. It can be used to separate the rates of which
        references are set with the `set_joint_position` or `set_joint_velocity`
        methods, and those associated to the PID controllers. During each `update` call,
        PIDs gather new measurements from the robot and actuate a new generalized force.

        PIDs are updated only if a time greater than the timestep, configured with
        `set_dt`, has passed. This appproach allows calling this `update` method multiple
        times in a faster loop while only updating the low-level PIDs when necessary.

        Args:
            current_time: The current time used to check if the PIDs have to be triggered.

        Returns:
            True if successful, False otherwise.
        """

    @abstractmethod
    def joint_position_limits(self, joint_name: str) -> Tuple[float, float]:
        """
        Return the position limits of the specified joint.

        Args:
            joint_name: The name of the joint.

        Returns:
            A Tuple in the form [min, max] containing the position limit.
        """

    @abstractmethod
    def joint_force_limit(self, joint_name: str) -> float:
        """
        Return the generalized force limit of the specified joint.

        Args:
            joint_name: The name of the joint.

        Returns:
            The maximum generalized force supported by the joint.
        """

    @abstractmethod
    def set_joint_force_limit(self, joint_name: str, limit: float) -> bool:
        """
        Set the maximum effort of the specified joint.

        Args:
            joint_name: The name of the joint.
            limit: The effort limit.

        Returns:
            True if successful, False otherwise.
        """
