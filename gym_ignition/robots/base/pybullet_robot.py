# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import numpy as np
from gym_ignition.base import robot
from gym_ignition.utils import logger, resource_finder
from typing import List, Tuple, Dict, Optional, Union, NamedTuple
from gym_ignition.base.robot.robot_joints import JointControlMode


class JointControlInfo(NamedTuple):
    mode: JointControlMode
    PID: robot.PID = None


def controlmode_to_pybullet(control_mode: JointControlMode):
    import pybullet
    JointControlMode2PyBullet = {
        JointControlMode.POSITION: pybullet.PD_CONTROL,
        JointControlMode.POSITION_INTERPOLATED: None,
        JointControlMode.VELOCITY: pybullet.PD_CONTROL,
        JointControlMode.TORQUE: pybullet.TORQUE_CONTROL,
    }

    return JointControlMode2PyBullet[control_mode]


class ContactPyBullet(NamedTuple):
    contactFlag: int
    bodyUniqueIdA: int
    bodyUniqueIdB: int
    linkIndexA: int
    linkIndexB: int
    positionOnA: List[float]
    positionOnB: List[float]
    contactNormalOnB: List[float]
    contactDistance: float
    normalForce: float
    lateralFriction1: float
    lateralFrictionDir1: List[float]
    lateralFriction2: float
    lateralFrictionDir2: List[float]


class JointInfoPyBullet(NamedTuple):
    jointIndex: int
    jointName: bytes
    jointType: int
    qIndex: int
    uIndex: int
    flags: int
    jointDamping: float
    jointFriction: float
    jointLowerLimit: float
    jointUpperLimit: float
    jointMaxForce: float
    jointMaxVelocity: float
    linkName: bytes
    jointAxis: List
    parentFramePos: List
    parentFrameOrn: List
    parentIndex: int


class JointStatePyBullet(NamedTuple):
    jointPosition: float
    jointVelocity: float
    jointReactionForces: List[float]
    appliedJointMotorTorque: float


class LinkInfoPyBullet(NamedTuple):
    linkWorldPosition: List[float]
    linkWorldOrientation: List[float]
    localInertialFramePosition: List[float]
    localInertialFrameOrientation: List[float]
    worldLinkFramePosition: List[float]
    worldLinkFrameOrientation: List[float]
    worldLinkLinearVelocity: List[float]
    worldLinkAngularVelocity: List[float]


class ConstraintInfoPyBullet(NamedTuple):
    parentBodyUniqueId: int
    parentJointIndex: int
    childBodyUniqueId: int
    childLinkIndex: int
    constraintType: int
    jointAxis: List[float]
    jointPivotInParent: List[float]
    jointPivotInChild: List[float]
    jointFrameOrientationParent: List[float]
    jointFrameOrientationChild: List[float]
    maxAppliedForce: float
    gearRatio: float
    gearAuxLink: float
    relativePositionTarget: float
    erp: float


class PyBulletRobot(robot.robot_abc.RobotABC,
                    robot.robot_links.RobotLinks,
                    robot.robot_joints.RobotJoints,
                    robot.robot_contacts.RobotContacts,
                    robot.robot_baseframe.RobotBaseFrame,
                    robot.robot_initialstate.RobotInitialState):
    """
    A "class"`Robot` implementation for the PyBullet simulator.

    Args:
        p: An instance of the pybullet simulator.
        model_file: The file (URDF, SDF) containing the robot model.
        plane_id: the pybullet ID associated with the plane model. It is used to
            compute contacts between the robot and the plane.
    """

    def __init__(self,
                 p,
                 model_file: str,
                 plane_id: int = None,
                 keep_fixed_joints: bool = False) -> None:

        self._pybullet = p
        self._plane_id = plane_id
        self._keep_fixed_joints = keep_fixed_joints

        # Find the model file
        model_abs_path = resource_finder.find_resource(model_file)

        # Initialize the parent classes
        robot.robot_abc.RobotABC.__init__(self)
        robot.robot_baseframe.RobotBaseFrame.__init__(self)
        robot.robot_initialstate.RobotInitialState.__init__(self)
        self.model_file = model_abs_path

        # Private attributes
        self._links_name2index_dict = None
        self._joints_name2index_dict = None
        self._robot_id: Optional[int] = None

        # TODO
        self._base_frame = None
        self._base_constraint = None

        # Create a map from the joint name to its control info
        self._jointname2jointcontrolinfo = dict()

    # ==============================
    # PRIVATE METHODS AND PROPERTIES
    # ==============================

    @property
    def robot_id(self) -> int:

        if self._robot_id is not None:
            return self._robot_id

        raise ValueError("The robot has not yet been added in the simulation")

    @robot_id.setter
    def robot_id(self, robot_id: int) -> None:

        if self._robot_id is not None:
            raise RuntimeError("This object already has an associated robot id")

        self._robot_id = robot_id

    def delete_simulated_robot(self) -> None:
        if not self._pybullet or self.robot_id is None:
            logger.warn("Failed to delete robot from the simulation. "
                        "Simulator not running.")
            return

        # Remove the robot from the simulation
        self._pybullet.removeBody(self.robot_id)
        self._robot_id = None

    @property
    def _joints_name2index(self) -> Dict[str, int]:
        if self._joints_name2index_dict is not None:
            return self._joints_name2index_dict

        self._joints_name2index_dict = dict()
        joints_info = self._get_joints_info()

        for _, info in joints_info.items():
            joint_idx = info.jointIndex
            joint_name = info.jointName.decode()
            self._joints_name2index_dict[joint_name] = joint_idx

        return self._joints_name2index_dict

    @property
    def _links_name2index(self) -> Dict[str, int]:
        if self._links_name2index_dict is not None:
            return self._links_name2index_dict

        self._links_name2index_dict = dict()

        # Add the base link
        self._links_name2index_dict[self.base_frame()] = -1

        # Add the other links
        for _, info in self._get_joints_info().items():
            self._links_name2index_dict[info.linkName.decode()] = info.jointIndex

        return self._links_name2index_dict

    def _load_model(self, filename: str, **kwargs) -> int:
        # Get the file extension
        extension = os.path.splitext(filename)[1][1:]

        if extension == "sdf":
            model_id = self._pybullet.loadSDF(filename, **kwargs)[0]
        else:
            import pybullet
            flags = pybullet.URDF_USE_INERTIA_FROM_FILE
            flags = flags | pybullet.URDF_MERGE_FIXED_LINKS
            model_id = self._pybullet.loadURDF(
                fileName=filename,
                flags=flags,
                **kwargs)

        return model_id

    def initialize_model(self) -> None:
        # Load the model
        self.robot_id = self._load_model(self.model_file)
        assert self.robot_id is not None, "Failed to load the robot model"

        # Initialize all the joints in POSITION mode
        for name in self.joint_names():
            self._jointname2jointcontrolinfo[name] = JointControlInfo(
                mode=JointControlMode.POSITION)
            ok_mode = self.set_joint_control_mode(name, JointControlMode.POSITION)
            assert ok_mode, \
                f"Failed to initialize the control mode of joint '{name}'"

        assert self.initial_joint_positions().size == self.dofs()
        assert self.initial_joint_velocities().size == self.dofs()

        # Initialize the joints state
        for idx, name in enumerate(self.joint_names()):
            self.reset_joint(joint_name=name,
                             position=self.initial_joint_positions()[idx],
                             velocity=self.initial_joint_velocities()[idx])

        # Initialize the base pose
        initial_base_position, initial_base_orientation = self.initial_base_pose()
        self.reset_base_pose(initial_base_position, initial_base_orientation)

    def _get_joint_info(self, joint_name: str) -> JointInfoPyBullet:
        # Get the joint index
        joint_idx = self._joints_name2index[joint_name]

        # Get the joint info from pybullet
        joint_info_pybullet = self._pybullet.getJointInfo(self.robot_id, joint_idx)

        # Store it in a namedtuple
        joint_info = JointInfoPyBullet._make(joint_info_pybullet)

        return joint_info

    def _get_joints_info(self) -> Dict[str, JointInfoPyBullet]:
        joints_info = {}

        for j in range(self._pybullet.getNumJoints(self.robot_id)):
            # Get the joint info from pybullet
            joint_info_pybullet = self._pybullet.getJointInfo(self.robot_id, j)

            # Store it in a namedtuple
            joint_info = JointInfoPyBullet._make(joint_info_pybullet)

            # Add the dict entry
            joints_info[joint_info.jointName.decode()] = joint_info

        return joints_info

    def _get_base_link_info(self) -> LinkInfoPyBullet:
        # Get the base link info
        base_position, base_orientation = self.base_pose()
        base_lin_vel, base_ang_vel = self.base_velocity()
        base_link_idx = self._links_name2index[self.base_frame()]
        assert base_link_idx == -1

        base_link_info = LinkInfoPyBullet(linkWorldPosition=base_position.tolist(),
                                          linkWorldOrientation=base_orientation.tolist(),
                                          localInertialFramePosition=None,
                                          localInertialFrameOrientation=None,
                                          worldLinkFramePosition=None,
                                          worldLinkFrameOrientation=None,
                                          worldLinkLinearVelocity=base_lin_vel.tolist(),
                                          worldLinkAngularVelocity=base_ang_vel.tolist())

        return base_link_info

    def _get_link_info(self, link_name: str) -> LinkInfoPyBullet:
        # Handle the base link
        if link_name == self.base_frame():
            return self._get_base_link_info()

        # Get the link index
        link_idx = self._links_name2index[link_name]

        # Get the link info from pybullet
        link_info_pybullet = self._pybullet.getLinkState(bodyUniqueId=self.robot_id,
                                                         linkIndex=link_idx,
                                                         computeLinkVelocity=1,
                                                         computeForwardKinematics=1)

        # Store it in a namedtuple
        link_info = LinkInfoPyBullet._make(link_info_pybullet)

        return link_info

    def _get_links_info(self) -> Dict[str, LinkInfoPyBullet]:
        links_info = {}

        # Add the information of all other links
        for name in set(self.link_names()):
            links_info[name] = self._get_link_info(link_name=name)

        return links_info

    def _get_contact_info(self) -> List[ContactPyBullet]:
        # Get the all contact points in the simulation
        pybullet_contacts = self._pybullet.getContactPoints()

        # List containing only the contacts that involve the robot model
        contacts = []

        # Extract only the robot contacts
        for pybullet_contact in pybullet_contacts:
            contact = ContactPyBullet._make(pybullet_contact)
            contacts.append(contact)

        return contacts

    # =====================
    # robot.Robot INTERFACE
    # =====================

    def name(self) -> str:
        # TODO
        pass

    def valid(self) -> bool:
        try:
            # TODO: improve the check
            self._pybullet.getBasePositionAndOrientation(self.robot_id)
            return True
        except:
            return False

    # ==================================
    # robot_joints.RobotJoints INTERFACE
    # ==================================

    def dofs(self) -> int:
        dofs = len(self.joint_names())

        if self._keep_fixed_joints:
            assert dofs == self._pybullet.getNumJoints(self.robot_id), \
                "Number of DoFs does not match with the simulated model"

        return dofs

    def joint_names(self) -> List[str]:
        import pybullet
        joints_names = []

        # Get Joint names
        for _, info in self._get_joints_info().items():
            if not self._keep_fixed_joints and info.jointType == pybullet.JOINT_FIXED:
                continue

            # Strings are byte-encoded. They have to be decoded with UTF-8 (default).
            joints_names.append(info.jointName.decode())

        return joints_names

    def joint_type(self, joint_name: str) -> robot.robot_joints.JointType:
        import pybullet
        joint_info = self._get_joint_info(joint_name)
        joint_type_pybullet = joint_info.jointType

        if joint_type_pybullet == pybullet.JOINT_FIXED:
            return robot.robot_joints.JointType.FIXED
        elif joint_type_pybullet == pybullet.JOINT_REVOLUTE:
            return robot.robot_joints.JointType.REVOLUTE
        elif joint_type_pybullet == pybullet.JOINT_PRISMATIC:
            return robot.robot_joints.JointType.PRISMATIC
        else:
            raise Exception(f"Joint type '{joint_type_pybullet}' not yet supported")

    def joint_control_mode(self, joint_name: str) -> JointControlMode:
        return self._jointname2jointcontrolinfo[joint_name].mode

    def set_joint_control_mode(self, joint_name: str, mode: JointControlMode) -> bool:

        if joint_name not in self._jointname2jointcontrolinfo:
            raise ValueError(joint_name)

        current_pid = self._jointname2jointcontrolinfo[joint_name].PID

        # Create a default PID if gains were never set
        if mode in {JointControlMode.POSITION, JointControlMode.VELOCITY}:

            if current_pid is None:
                current_pid = robot.PID(p=1, i=0, d=0)

        joint_control_info = JointControlInfo(mode=mode, PID=current_pid)
        self._jointname2jointcontrolinfo[joint_name] = joint_control_info

        # Get intermediate data
        mode_pybullet = controlmode_to_pybullet(mode)
        pid = self._jointname2jointcontrolinfo[joint_name].PID
        joint_idx_pybullet = self._joints_name2index[joint_name]

        if mode == JointControlMode.TORQUE:
            # Disable the default joint motorization setting a 0 maximum force
            import pybullet
            # Disable the PID if was configured
            self._pybullet.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_idx_pybullet,
                controlMode=pybullet.PD_CONTROL,
                positionGain=0,
                velocityGain=0)
            # Put the motor in IDLE for torque control
            self._pybullet.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_idx_pybullet,
                controlMode=pybullet.VELOCITY_CONTROL,
                force=0)

        elif mode == JointControlMode.POSITION:
            if pid.i != 0.0:
                logger.warn("Integral PID gain not supported in POSITION mode")

            self._pybullet.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_idx_pybullet,
                controlMode=mode_pybullet,
                positionGain=pid.p,
                velocityGain=pid.d)

        elif mode == JointControlMode.VELOCITY:
            if pid.d != 0.0:
                logger.warn("Derivative PID gain not supported in VELOCITY mode")

            # TODO: verify that setting the gains in this way processes the reference
            #  correctly.
            self._pybullet.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_idx_pybullet,
                controlMode=mode_pybullet,
                positionGain=pid.i,
                velocityGain=pid.p)

        else:
            raise RuntimeError(f"Control mode '{mode}' is not supported")

        return True

    def joint_position(self, joint_name: str) -> float:
        joint_idx = self._joints_name2index[joint_name]
        joint_state_pybullet = self._pybullet.getJointState(self.robot_id, joint_idx)
        joint_state = JointStatePyBullet._make(joint_state_pybullet)

        return joint_state.jointPosition

    def joint_velocity(self, joint_name: str) -> float:
        joint_idx = self._joints_name2index[joint_name]
        joint_state_pybullet = self._pybullet.getJointState(self.robot_id, joint_idx)
        joint_state = JointStatePyBullet._make(joint_state_pybullet)

        return joint_state.jointVelocity

    def joint_force(self, joint_name: str) -> float:
        joint_idx = self._joints_name2index[joint_name]
        joint_state_pybullet = self._pybullet.getJointState(self.robot_id, joint_idx)
        joint_state = JointStatePyBullet._make(joint_state_pybullet)

        return joint_state.appliedJointMotorTorque

    def joint_positions(self) -> np.ndarray:
        joint_states = self._pybullet.getJointStates(self.robot_id, range(self.dofs()))
        joint_positions = \
            [JointStatePyBullet._make(state).jointPosition for state in joint_states]

        return np.array(joint_positions)

    def joint_velocities(self) -> np.ndarray:
        joint_states = self._pybullet.getJointStates(self.robot_id, range(self.dofs()))
        joint_velocities = \
            [JointStatePyBullet._make(state).jointVelocity for state in joint_states]

        return np.array(joint_velocities)

    def joint_pid(self, joint_name: str) -> Union[robot.PID, None]:
        return self._jointname2jointcontrolinfo[joint_name].PID

    def dt(self) -> float:
        logger.warn("This method is no-op")
        return True

    def set_dt(self, step_size: float) -> bool:
        logger.warn("This method is no-op")
        return True

    def set_joint_force(self, joint_name: str, force: float, clip: bool = False) -> bool:

        if self._jointname2jointcontrolinfo[joint_name].mode != JointControlMode.TORQUE:
            raise Exception("Joint '{}' is not controlled in TORQUE".format(joint_name))

        joint_idx_pybullet = self._joints_name2index[joint_name]
        mode_pybullet = controlmode_to_pybullet(JointControlMode.TORQUE)

        # Clip the force if specified
        if clip:
            fmin, fmax = self.joint_force_limit(joint_name)
            force = fmin if force < fmin else fmax if force > fmax else force

        # Set the joint force
        self._pybullet.setJointMotorControl2(bodyUniqueId=self.robot_id,
                                             jointIndex=joint_idx_pybullet,
                                             controlMode=mode_pybullet,
                                             force=force)

        return True

    def set_joint_position(self, joint_name: str, position: float) -> bool:

        if self._jointname2jointcontrolinfo[joint_name].mode != JointControlMode.POSITION:
            raise RuntimeError(f"Joint '{joint_name}' is not controlled in POSITION")

        pid = self._jointname2jointcontrolinfo[joint_name].PID

        joint_idx_pybullet = self._joints_name2index[joint_name]
        mode_pybullet = controlmode_to_pybullet(JointControlMode.POSITION)

        # Change the control mode of the joint
        self._pybullet.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=joint_idx_pybullet,
            controlMode=mode_pybullet,
            targetPosition=position,
            positionGain=pid.p,
            velocityGain=pid.d)

        return True

    def set_joint_velocity(self, joint_name: str, velocity: float) -> bool:

        if self._jointname2jointcontrolinfo[joint_name].mode not in \
                {JointControlMode.POSITION, JointControlMode.VELOCITY}:
            raise Exception("Joint '{}' is not controlled in VELOCITY".format(joint_name))

        pid = self._jointname2jointcontrolinfo[joint_name].PID

        joint_idx_pybullet = self._joints_name2index[joint_name]
        mode_pybullet = controlmode_to_pybullet(JointControlMode.POSITION)

        # Change the control mode of the joint
        self._pybullet.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=joint_idx_pybullet,
            controlMode=mode_pybullet,
            targetVelocity=velocity,
            positionGain=pid.p,
            velocityGain=pid.d)

        return True

    def set_joint_interpolated_position(self, joint_name: str, position: float):
        return NotImplementedError

    def set_joint_pid(self, joint_name: str, pid: robot.PID) -> bool:

        if joint_name not in self._jointname2jointcontrolinfo:
            raise ValueError(joint_name)

        mode = self._jointname2jointcontrolinfo[joint_name].mode
        self._jointname2jointcontrolinfo[joint_name] = \
            JointControlInfo(mode=mode, PID=pid)

        # Update the PIDs by setting again the control mode
        ok_mode = self.set_joint_control_mode(joint_name, mode)
        assert ok_mode, "Failed to set the control mode"

        return True

    def reset_joint(self,
                    joint_name: str,
                    position: float = None,
                    velocity: float = None) -> bool:

        joint_idx_pybullet = self._joints_name2index[joint_name]
        self._pybullet.resetJointState(bodyUniqueId=self.robot_id,
                                       jointIndex=joint_idx_pybullet,
                                       targetValue=position,
                                       targetVelocity=velocity)

        return True

    def update(self, current_time: float) -> bool:
        raise NotImplementedError

    def joint_position_limits(self, joint_name: str) -> Tuple[float, float]:
        # Get Joint info
        joint_info = self._get_joint_info(joint_name)

        return joint_info.jointLowerLimit, joint_info.jointUpperLimit

    def joint_force_limit(self, joint_name: str) -> float:
        # Get Joint info
        joint_info = self._get_joint_info(joint_name)

        return joint_info.jointMaxForce

    def set_joint_force_limit(self, joint_name: str, limit: float) -> bool:
        raise NotImplementedError

    # =====================================
    # robot_joints.RobotBaseFrame INTERFACE
    # =====================================

    def set_as_floating_base(self, floating: bool) -> bool:
        # Handle fixed to floating base conversion
        if self._robot_id is not None and not self._is_floating_base and floating:
            assert self._base_constraint, "No base constraint found"
            self._pybullet.removeConstraint(self._base_constraint)
            self._base_constraint = None

        # The next call of reset_base_pose will create the constraint
        self._is_floating_base = floating
        return True

    def is_floating_base(self) -> bool:
        if not self._is_floating_base:
            assert self._base_constraint is not None

        return self._is_floating_base

    def set_base_frame(self, frame_name: str) -> bool:
        # TODO: check that it exists
        self._base_frame = frame_name
        return True

    def base_frame(self) -> str:
        if not self._base_frame:
            logger.warn("Base name not set. Returning dummy name.")
            return "root"

        return self._base_frame

    def base_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        # Get the values
        pos, quat = self._pybullet.getBasePositionAndOrientation(self.robot_id)

        # Convert tuples into lists
        pos = np.array(pos)
        quat = np.array(quat)

        # PyBullet returns xyzw, the interface instead returns wxyz
        quat = np.roll(quat, 1)

        return pos, quat

    def base_velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        # TODO: double check
        # Get the values
        lin, ang = self._pybullet.getBaseVelocity(self.robot_id)

        # Convert tuples into lists
        lin = np.array(lin)
        ang = np.array(ang)

        return lin, ang

    def reset_base_pose(self, position: np.ndarray, orientation: np.ndarray) -> bool:

        assert position.size == 3, "Position should be a list with 3 elements"
        assert orientation.size == 4, "Orientation should be a list with 4 elements"

        # PyBullet wants xyzw, but the function argument quaternion is wxyz
        orientation = np.roll(orientation, -1)

        # Fixed base robot and constraint already created
        if not self._is_floating_base and self._base_constraint:
            cur_position, cur_orientation = self.base_pose()
            if not np.allclose(position, cur_position) or \
                np.allclose(orientation, cur_orientation):
                assert self._base_constraint, "No base constraint found"
                self._pybullet.removeConstraint(self._base_constraint)
                self._base_constraint = None

        # Fixed base robot and constraint not yet created
        if not self._is_floating_base and not self._base_constraint:
            # Set a constraint between base_link and world
            # TODO: check the floating base link
            self._base_constraint = self._pybullet.createConstraint(
                self.robot_id, -1, -1, -1, self._pybullet.JOINT_FIXED,
                [0, 0, 0], [0, 0, 0], position, orientation)

        self._pybullet.resetBasePositionAndOrientation(
            self.robot_id, position, orientation)

        return True

    def reset_base_velocity(self,
                            linear_velocity: np.ndarray,
                            angular_velocity: np.ndarray) -> bool:
        raise NotImplementedError

    def base_wrench(self) -> np.ndarray:
        assert self._base_constraint is not None, "The robot is not fixed base"
        constraint_wrench = self._pybullet.getConstraintState(self._base_constraint)
        # print("Constraint State =", constraint_wrench)
        return np.array(constraint_wrench)

    # =============
    # RobotContacts
    # =============

    def links_in_contact(self) -> List[str]:
        # Get the all contact points in the simulation
        all_contacts = self._get_contact_info()

        # List containing only the contacts that involve the robot model
        contacts_robot = []

        # Extract only the robot contacts
        for contact in all_contacts:
            if contact.bodyUniqueIdA == self.robot_id or \
                    contact.bodyUniqueIdB == self.robot_id:
                contacts_robot.append(contact)

        # List containing the names of the link with active contacts
        links_in_contact = []

        for contact in contacts_robot:
            # Link of the link in contact
            link_idx_in_contact = None

            # Get the index of the link in contact.
            # The robot body could be either body A or body B.
            if contact.bodyUniqueIdA == self.robot_id:
                link_idx_in_contact = contact.linkIndexA
            elif contact.bodyUniqueIdB == self.robot_id:
                link_idx_in_contact = contact.linkIndexB

            # Get the link name from the index and add it to the returned list
            for link_name, link_idx in self._links_name2index.items():
                if link_idx == link_idx_in_contact:
                    if link_name not in links_in_contact:

                        # Do not consider the contact if the wrench is (almost) zero
                        wrench = self.total_contact_wrench_on_link(link_name)

                        if np.linalg.norm(wrench) < 1e-6:
                            continue

                        # Append the link name to the list of link in contact
                        links_in_contact.append(link_name)

        return links_in_contact

    def contact_data(self, contact_link_name: str):  # TODO
        raise NotImplementedError

    def total_contact_wrench_on_link(self, contact_link_name: str) -> np.ndarray:
        # TODO: finish the implementation of this method

        # Get the all contact points in the simulation
        all_contacts = self._get_contact_info()

        # List containing only the contacts that involve the robot model
        contacts_robot = []

        # Extract only the robot contacts
        for contact in all_contacts:
            if contact.bodyUniqueIdA == self.robot_id or contact.bodyUniqueIdB == \
                    self.robot_id:
                contacts_robot.append(contact)

        contact_link_idx = self._links_name2index[contact_link_name]

        # List containing only the contacts that involve the link
        contacts_link = []

        for contact in contacts_robot:
            if contact.bodyUniqueIdA == self.robot_id and \
                    contact.linkIndexA == contact_link_idx or \
               contact.bodyUniqueIdB == self.robot_id and \
                    contact.linkIndexB == contact_link_idx:
                contacts_link.append(contact)

        total_force = np.zeros(3)

        for contact in contacts_link:
            # TODO: handle that id could be either body a or b
            force_1 = contact.normalForce * np.array(contact.contactNormalOnB)
            force_2 = contact.lateralFriction1 * np.array(contact.lateralFrictionDir1)
            force_3 = contact.lateralFriction2 * np.array(contact.lateralFrictionDir2)

            # TODO: compose the force transforming it in wrench applied to the link frame
            total_force += force_1 + force_2 + force_3

        return total_force

    # =================
    # RobotInitialState
    # =================

    def initial_joint_positions(self) -> np.ndarray:
        if self._initial_joint_positions is None:
            self._initial_joint_positions = np.zeros(self.dofs())

        return self._initial_joint_positions

    def set_initial_joint_positions(self, positions: np.ndarray) -> bool:
        if self._robot_id is not None:
            raise Exception("The robot object has been already created")

        self._initial_joint_positions = positions
        return True

    def initial_joint_velocities(self) -> np.ndarray:
        if self._initial_joint_velocities is None:
            self._initial_joint_velocities = np.zeros(self.dofs())

        return self._initial_joint_velocities

    def set_initial_joint_velocities(self, velocities: np.ndarray) -> bool:
        if self._robot_id is not None:
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

    # ==========
    # RobotLinks
    # ==========

    def link_names(self) -> List[str]:
        link_names = list()

        # Append the base link that pybullet ignores
        link_names.append(self.base_frame())

        for _, info in self._get_joints_info().items():
            link_names.append(info.linkName.decode())

        return link_names

    def link_pose(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        link_info = self._get_link_info(link_name)

        link_position = link_info.linkWorldPosition
        link_quaternion = link_info.linkWorldOrientation

        return np.array(link_position), np.array(link_quaternion)

    def link_velocity(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        link_info = self._get_link_info(link_name)

        # TODO: CoM or frame?
        link_lin_vel = link_info.worldLinkLinearVelocity
        link_ang_vel = link_info.worldLinkAngularVelocity

        return np.array(link_lin_vel), np.array(link_ang_vel)

    def link_acceleration(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        raise NotImplementedError

    def apply_external_force(self,
                             link_name: str,
                             force: np.ndarray,
                             torque: np.ndarray) -> bool:
        import pybullet
        link_idx = self._links_name2index[link_name]

        self._pybullet.applyExternalForce(self.robot_id,
                                          link_idx,
                                          force.tolist(),
                                          [0, 0, 0],
                                          flags=pybullet.LINK_FRAME)

        self._pybullet.applyExternalTorque(self.robot_id,
                                           link_idx,
                                           torque.tolist(),
                                           flags=pybullet.LINK_FRAME)

        return True
