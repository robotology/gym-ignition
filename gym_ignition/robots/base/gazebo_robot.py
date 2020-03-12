# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import string
import random
import numpy as np
from typing import List, Union, Tuple
from gym_ignition import gympp_bindings as bindings
from gym_ignition.utils import logger, resource_finder
from gym_ignition.base.robot import robot_baseframe, robot_initialstate
from gym_ignition.base.robot import robot_abc, robot_joints, robot_links, robot_contacts


class GazeboRobot(robot_abc.RobotABC,
                  robot_links.RobotLinks,
                  robot_joints.RobotJoints,
                  robot_contacts.RobotContacts,
                  robot_baseframe.RobotBaseFrame,
                  robot_initialstate.RobotInitialState):

    def __init__(self,
                 model_file: str,
                 gazebo: bindings.GazeboWrapper,
                 controller_rate: float = None) -> None:

        # Find the model file
        model_abs_path = resource_finder.find_resource(model_file)

        # Initialize the parent classes
        robot_abc.RobotABC.__init__(self)
        robot_baseframe.RobotBaseFrame.__init__(self)
        robot_initialstate.RobotInitialState.__init__(self)
        self.model_file = model_abs_path

        # Private attributes
        self._gazebo = gazebo
        self._robot_name = None
        self._gympp_robot = None
        self._base_frame = None
        self._joint_names = None
        self._controller_rate = controller_rate

        # Create a random prefix that will be used for the robot name
        letters_and_digits = string.ascii_letters + string.digits
        self._prefix = ''.join(random.choice(letters_and_digits) for _ in range(6))

    # ==============================
    # PRIVATE METHODS AND PROPERTIES
    # ==============================

    def delete_simulated_robot(self) -> None:
        if not self._gazebo or not self._gympp_robot:
            logger.warn("Failed to delete robot from the simulation. "
                        "Simulator not running.")
            return

        # Remove the robot from the simulation
        ok_model = self._gazebo.removeModel(self._robot_name)
        assert ok_model, f"Failed to remove the model '{self._robot_name}' from gazebo"

    @staticmethod
    def _to_cpp_controlmode(mode: robot_joints.JointControlMode):
        map_cm = {
            robot_joints.JointControlMode.POSITION: bindings.JointControlMode_Position,
            robot_joints.JointControlMode.POSITION_INTERPOLATED:
                bindings.JointControlMode_PositionInterpolated,
            robot_joints.JointControlMode.VELOCITY: bindings.JointControlMode_Velocity,
            robot_joints.JointControlMode.TORQUE: bindings.JointControlMode_Torque,
        }
        assert mode in map_cm, f"Unsupported '{mode}' control mode"
        return map_cm[mode]

    @staticmethod
    def _from_cpp_controlmode(mode) -> robot_joints.JointControlMode:
        map_cm = {
            bindings.JointControlMode_Position: robot_joints.JointControlMode.POSITION,
            bindings.JointControlMode_PositionInterpolated:
                robot_joints.JointControlMode.POSITION_INTERPOLATED,
            bindings.JointControlMode_Velocity: robot_joints.JointControlMode.VELOCITY,
            bindings.JointControlMode_Torque: robot_joints.JointControlMode.TORQUE,
        }
        assert mode in map_cm, f"Unsupported '{mode}' control mode"
        return map_cm[mode]

    @property
    def gympp_robot(self) -> bindings.Robot:
        if self._gympp_robot:
            return self._gympp_robot

        # Find and load the model SDF file
        sdf_file = resource_finder.find_resource(self.model_file)
        with open(sdf_file, "r") as stream:
            sdf_string = stream.read()

        # Get the model name
        original_name = bindings.GazeboWrapper.getModelNameFromSDF(sdf_string)
        assert original_name, f"Failed to get model name from file {self.model_file}"

        # Create a unique robot name
        self._robot_name = self._prefix + "::" + original_name

        initial_base_pose, initial_base_orientation = self.initial_base_pose()

        # Initialize the model data
        model_data = bindings.ModelInitData()
        model_data.modelName = self._robot_name
        model_data.sdfString = sdf_string
        model_data.fixedPose = not self.is_floating_base()
        model_data.position =initial_base_pose.tolist()
        model_data.orientation = initial_base_orientation.tolist()

        if self._base_frame is not None:
            model_data.baseLink = self._base_frame

        # Initialize robot controller plugin
        plugin_data = bindings.PluginData()
        plugin_data.libName = "RobotController"
        plugin_data.className = "gympp::plugins::RobotController"

        # Insert the model
        ok_model = self._gazebo.insertModel(model_data, plugin_data)
        assert ok_model, "Failed to insert the model"

        # Extract the robot from the singleton
        gympp_robot_weak = bindings.RobotSingleton_get().getRobot(self._robot_name)

        # The robot is a weak pointer. Check that it is valid.
        assert not gympp_robot_weak.expired(), "The Robot object has expired"
        assert gympp_robot_weak.lock(), \
            "The returned Robot object does not contain a valid interface"

        self._gympp_robot = gympp_robot_weak.lock()
        assert self._gympp_robot.valid(), "The Robot object is not valid"

        if self._controller_rate is not None:
            logger.debug(f"Robot controller rate: {self._controller_rate} Hz")
            ok_dt = self._gympp_robot.setdt(1 / self._controller_rate)
            assert ok_dt, "Failed to set the robot controller period"

        s0 = self.initial_joint_positions()
        sdot0 = self.initial_joint_velocities()
        joint_names = list(self._gympp_robot.jointNames())

        assert s0.size == len(joint_names)
        assert sdot0.size == len(joint_names)

        for idx, name in enumerate(joint_names):
            ok_reset = self._gympp_robot.resetJoint(name, s0[idx], sdot0[idx])
            assert ok_reset, f"Failed to initialize the state of joint '{name}'"

        logger.debug(
            f"GazeboRobot '{self._gympp_robot.name()}' added to the simulation")
        return self._gympp_robot

    # ========
    # RobotABC
    # ========

    def name(self):
        return self.gympp_robot.name()

    def valid(self) -> bool:
        return self.gympp_robot.valid()

    # ===========
    # RobotJoints
    # ===========

    def dofs(self) -> int:
        return len(self.joint_names())

    def joint_names(self) -> List[str]:
        if self._joint_names is None:
            self._joint_names = list(self.gympp_robot.jointNames())

        return self._joint_names

    def joint_type(self, joint_name: str) -> robot_joints.JointType:
        joint_type = self.gympp_robot.jointType(joint_name)
        assert joint_type != bindings.JointType_Invalid, \
            f"Type of joint '{joint_name}' not valid"

        if joint_type == bindings.JointType_Revolute:
            return robot_joints.JointType.REVOLUTE
        elif joint_type == bindings.JointType_Prismatic:
            return robot_joints.JointType.PRISMATIC
        elif joint_type == bindings.JointType_Fixed:
            return robot_joints.JointType.FIXED
        elif joint_type == bindings.JointType_Invalid:
            return robot_joints.JointType.INVALID
        else:
            raise Exception(f"Failed to recognize type of joint '{joint_name}'")

    def joint_control_mode(self, joint_name: str) -> robot_joints.JointControlMode:
        return self._from_cpp_controlmode(self.gympp_robot.jointControlMode(joint_name))

    def set_joint_control_mode(self,
                               joint_name: str,
                               mode: robot_joints.JointControlMode) -> bool:
        return self.gympp_robot.setJointControlMode(joint_name,
                                                    self._to_cpp_controlmode(mode))

    def joint_position(self, joint_name: str) -> float:
        return self.gympp_robot.jointPosition(joint_name)

    def joint_velocity(self, joint_name: str) -> float:
        return self.gympp_robot.jointVelocity(joint_name)

    def joint_force(self, joint_name: str) -> float:
        return self.gympp_robot.jointForce(joint_name)

    def joint_positions(self) -> np.ndarray:
        return np.array(self.gympp_robot.jointPositions())

    def joint_velocities(self) -> np.ndarray:
        return np.array(self.gympp_robot.jointVelocities())

    def joint_pid(self, joint_name: str) -> Union[robot_joints.PID, None]:
        gazebo_pid = self.gympp_robot.jointPID(joint_name)
        return robot_joints.PID(p=gazebo_pid.p, i=gazebo_pid.i, d=gazebo_pid.d)

    def dt(self) -> float:
        return self.gympp_robot.dt()

    def set_dt(self, step_size: float) -> bool:
        return self.gympp_robot.setdt(step_size)

    def set_joint_force(self, joint_name: str, force: float, clip: bool = False) -> bool:
        return self.gympp_robot.setJointForce(joint_name, force)

    def set_joint_position(self, joint_name: str, position: float) -> bool:
        return self.gympp_robot.setJointPositionTarget(joint_name, position)

    def set_joint_velocity(self, joint_name: str, velocity: float) -> bool:
        return self.gympp_robot.setJointVelocityTarget(joint_name, velocity)

    def set_joint_interpolated_position(self, joint_name: str, position: float):
        raise NotImplementedError

    def set_joint_pid(self, joint_name: str, pid: robot_joints.PID) -> bool:
        gazebo_pid = bindings.PID(pid.p, pid.i, pid.d)
        return self.gympp_robot.setJointPID(joint_name, gazebo_pid)

    def reset_joint(self,
                    joint_name: str,
                    position: float = None,
                    velocity: float = None) -> bool:
        return self.gympp_robot.resetJoint(joint_name, position, velocity)

    def update(self, dt: float):
        return self.gympp_robot.update(dt)

    def joint_position_limits(self, joint_name: str) -> Tuple[float, float]:
        limit = self.gympp_robot.jointPositionLimits(joint_name)
        return float(limit.min), float(limit.max)

    def joint_force_limit(self, joint_name: str) -> float:
        limit = self.gympp_robot.jointEffortLimit(joint_name)
        return float(limit)

    def set_joint_force_limit(self, joint_name: str, limit: float) -> bool:
        if limit < 0:
            raise ValueError(limit)

        ok_limit = self.gympp_robot.setJointEffortLimit(joint_name, limit)
        return ok_limit

    # ==============
    # RobotBaseFrame
    # ==============

    def set_as_floating_base(self, floating: bool) -> bool:
        if self._gympp_robot is not None and self._is_floating_base != floating:
            raise Exception(
                "Changing the base type after the creation of the robot is not supported")

        self._is_floating_base = floating
        return True

    def is_floating_base(self) -> bool:
        return self._is_floating_base

    def set_base_frame(self, frame_name: str) -> bool:
        self._base_frame = frame_name

        if self._gympp_robot:
            ok_base_frame = self.gympp_robot.setBaseFrame(frame_name)
            assert ok_base_frame, "Failed to set base frame"

        return True

    def base_frame(self) -> str:
        if self._base_frame is None:
            self._base_frame = self.gympp_robot.baseFrame()

        return self._base_frame

    def base_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        base_pose = self.gympp_robot.basePose()
        position = np.array(base_pose.position)
        orientation = np.array(base_pose.orientation)
        return position, orientation

    def base_velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        base_velocity_gympp = self.gympp_robot.baseVelocity()
        return np.array(base_velocity_gympp.linear), np.array(base_velocity_gympp.angular)

    def reset_base_pose(self,
                        position: np.ndarray,
                        orientation: np.ndarray) -> bool:

        assert position.size == 3, "Position should be a list with 3 elements"
        assert orientation.size == 4, "Orientation should be a list with 4 elements"

        if not self._is_floating_base:
            logger.error("Changing the pose of a fixed-base robot is not yet supported")
            logger.error("Remove and insert again the robot in the new pose")
            return False

        ok_pose = self.gympp_robot.resetBasePose(position.tolist(),
                                                 orientation.tolist())
        assert ok_pose, "Failed to set base pose"

        return True

    def reset_base_velocity(self,
                            linear_velocity: np.ndarray,
                            angular_velocity: np.ndarray) -> bool:
        assert linear_velocity.size == 3, \
            "Linear velocity should be a list with 3 elements"
        assert angular_velocity.size == 3, \
            "Angular velocity should be a list with 3 elements"

        if not self._is_floating_base:
            logger.error("Changing the velocity of a fixed-base robot is not supported")
            return False

        ok_velocity = self.gympp_robot.resetBaseVelocity(linear_velocity.tolist(),
                                                         angular_velocity.tolist())
        assert ok_velocity, "Failed to reset the base velocity"

        return True

    def base_wrench(self) -> np.ndarray:
        raise NotImplementedError

    # =================
    # RobotInitialState
    # =================

    def initial_joint_positions(self) -> np.ndarray:
        if self._initial_joint_positions is None:
            self._initial_joint_positions = \
                np.array(self.gympp_robot.initialJointPositions())

        return self._initial_joint_positions

    def set_initial_joint_positions(self, positions: np.ndarray) -> bool:
        if self._gympp_robot is not None:
            raise Exception("The robot object has been already created")

        self._initial_joint_positions = positions
        return True

    def initial_joint_velocities(self) -> np.ndarray:
        if self._initial_joint_velocities is None:
            self._initial_joint_velocities = np.zeros(self.dofs())

        return self._initial_joint_velocities

    def set_initial_joint_velocities(self, velocities: np.ndarray) -> bool:
        if self._gympp_robot is not None:
            raise Exception("The robot object has been already created")

        self._initial_joint_velocities = velocities
        return True

    def initial_base_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._initial_base_pose

    def set_initial_base_pose(self,
                              position: np.ndarray,
                              orientation: np.ndarray) -> bool:
        assert position.size == 3, "'position' should be an array with 3 elements"
        assert orientation.size == 4, "'orientation' should be an array with 4 elements"
        self._initial_base_pose = (position, orientation)
        return True

    def initial_base_velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._initial_base_velocity

    def set_initial_base_velocity(self, linear: np.ndarray, angular: np.ndarray) -> bool:
        assert linear.size == 3, "'linear' should be an array with 3 elements"
        assert angular.size == 4, "'angular' should be an array with 4 elements"
        self._initial_base_velocity = (linear, angular)
        return True

    # =============
    # RobotContacts
    # =============

    def links_in_contact(self) -> List[str]:
        return list(self._gympp_robot.linksInContact())

    def contact_data(self, contact_link_name: str) -> List[robot_contacts.ContactData]:
        contacts = []
        gazebo_contacts = self._gympp_robot.contactData(contact_link_name)

        for contact in gazebo_contacts:
            contact_data = robot_contacts.ContactData(bodyA=contact.bodyA,
                                                      bodyB=contact.bodyB,
                                                      position=contact.position)
            contacts.append(contact_data)

        return contacts

    def total_contact_wrench_on_link(self, contact_link_name: str) -> np.ndarray:
        raise NotImplementedError

    # ==========
    # RobotLinks
    # ==========

    def link_names(self) -> List[str]:
        return self.gympp_robot.linkNames()

    def link_pose(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        link_pose_gympp = self.gympp_robot.linkPose(link_name)
        return np.array(link_pose_gympp.position), np.array(link_pose_gympp.orientation)

    def link_velocity(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        link_velocity_gympp = self.gympp_robot.linkVelocity(link_name)
        return np.array(link_velocity_gympp.linear), np.array(link_velocity_gympp.angular)

    def link_acceleration(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        link_acc_gympp = self.gympp_robot.linkAcceleration(link_name)
        return np.array(link_acc_gympp.linear), np.array(link_acc_gympp.angular)

    def apply_external_force(self,
                             link_name: str,
                             force: np.ndarray,
                             torque: np.ndarray) -> bool:
        assert link_name in self.link_names(), "The link is not part of the model"

        ok_wrench = self.gympp_robot.addExternalWrench(
            link_name, list(force), list(torque))
        assert ok_wrench, f"Failed to add external wrench to link '{link_name}'"

        return True
