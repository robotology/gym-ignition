# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import numpy as np
from typing import List
from gym_ignition import base
from gym_ignition.utils import logger, resource_finder
from gym_ignition.base.robot import robot_abc, robot_joints
from gym_ignition.base.controllers import PositionController, PositionControllerReferences


@base.robot.feature_detector
class RobotFeatures(robot_abc.RobotABC,
                    robot_joints.RobotJoints,
                    abc.ABC):
    pass


class ComputedTorqueFixedBase(PositionController):
    """
    """

    def __init__(self,
                 robot: RobotFeatures,
                 urdf: str,
                 controlled_joints: List[str],
                 kp: np.ndarray,
                 kd: np.ndarray,
                 dt: float = None,
                 clip_torques: bool = False,
                 **kwargs) -> None:
        import iDynTree as idyn

        # Find the urdf file
        abs_path_urdf = resource_finder.find_resource(urdf)

        # Initialize base class
        super().__init__(dt=dt,
                         robot=robot,
                         urdf=abs_path_urdf,
                         controlled_joints=controlled_joints)

        # Check that the robot has all the requested features
        RobotFeatures.has_all_features(robot)

        # Object to compute rigid-body dynamics quantities
        self.kindyn = idyn.KinDynComputations()

        # Set the velocity representation
        self.kindyn.setFrameVelocityRepresentation(idyn.MIXED_REPRESENTATION)

        # Gains
        self.kp = kp
        self.kd = kd

        self._references = None
        self._clip_torques = clip_torques
        self._initial_control_mode = dict()

    def __del__(self):
        self.terminate()

        # ===============
    # PRIVATE METHODS
    # ===============

    def _load_model(self) -> bool:
        logger.debug(f"Loading model '{self.urdf}' in the fixed-base controller")
        if len(self.controlled_joints) == self.robot.dofs():
            logger.debug("Controlling all joints")
        else:
            logger.debug(f"Controlling joints: {self.controlled_joints}")

        # Load the urdf model
        import iDynTree as idyn
        mdl_loader = idyn.ModelLoader()
        ok_load = mdl_loader.loadReducedModelFromFile(self.urdf,
                                                      self.controlled_joints)
        assert ok_load, "Failed to load urdf model from file"

        # Insert the model in KinDynComputations
        ok_model = self.kindyn.loadRobotModel(mdl_loader.model())
        assert ok_model, "Failed to load model in KinDynComputations"

        return ok_model

    def _controlled_joint_positions(self) -> np.ndarray:
        joint_positions = np.zeros(self.nr_controlled_dofs)

        for idx, name in enumerate(self.controlled_joints):
            joint_positions[idx] = self.robot.joint_position(name)

        return joint_positions

    def _controlled_joint_velocities(self) -> np.ndarray:
        joint_velocities = np.zeros(self.nr_controlled_dofs)

        for idx, name in enumerate(self.controlled_joints):
            joint_velocities[idx] = self.robot.joint_velocity(name)

        return joint_velocities

    # ==================
    # PositionController
    # ==================

    def set_control_references(self, references: PositionControllerReferences) -> bool:
        dofs = references.position.size
        assert dofs == self.nr_controlled_dofs

        self._references = references

        if references.velocity is None:
            self._references = \
                self._references._replace(velocity=np.zeros_like(references.position))

        if references.acceleration is None:
            self._references = \
                self._references._replace(acceleration=np.zeros_like(references.position))

        assert self._references.valid()
        return True

    # ==========
    # Controller
    # ==========

    def initialize(self) -> bool:
        # Load the model
        ok_model = self._load_model()
        assert ok_model, "Failed to load the model"

        for name in self.controlled_joints:
            # Store the initial control mode
            self._initial_control_mode[name] = self.robot.joint_control_mode(name)

            if self._initial_control_mode[name] is None:
                self._initial_control_mode[name] = robot_joints.JointControlMode.POSITION

            # Control the joint in TORQUE
            ok_mode = self.robot.set_joint_control_mode(
                name, base.robot.robot_joints.JointControlMode.TORQUE)
            assert ok_mode, f"Failed to control joint '{name}' in TORQUE"

        return True

    def step(self) -> np.ndarray:
        # ==============
        # Get joint data
        # ==============

        joint_positions = self._controlled_joint_positions()
        joint_velocities = self._controlled_joint_velocities()

        dofs = self.nr_controlled_dofs

        # =============================
        # Compute rigid-body quantities
        # =============================
        import iDynTree as idyn

        s = idyn.VectorDynSize(dofs)
        s_dot = idyn.VectorDynSize(dofs)

        s = s.FromPython(joint_positions)
        s_dot = s_dot.FromPython(joint_velocities)

        world_gravity = idyn.Vector3()
        world_gravity.zero()
        world_gravity.setVal(2, -9.8182)

        # Not relevant for fixed-base robots
        pos_base = np.array([0., 0, 0])
        orient_base = np.array([1., 0, 0, 0])

        world_T_base = idyn.Transform()
        world_T_base.setPosition(idyn.Position(pos_base[0], pos_base[1], pos_base[2]))

        rot_base = idyn.Rotation()
        quaternion_base = idyn.Vector4()
        quaternion_base = quaternion_base.FromPython(orient_base)

        rot_base.fromQuaternion(quaternion_base)
        world_T_base.setRotation(rot_base)

        base_velocity = idyn.Twist()
        base_velocity.zero()

        # Set the robot state
        ok_state = self.kindyn.setRobotState(
            world_T_base, s, base_velocity, s_dot, world_gravity)
        assert ok_state, "Failed to set the robot state"

        M = idyn.MatrixDynSize(dofs + 6, dofs + 6)
        ok_M = self.kindyn.getFreeFloatingMassMatrix(M)
        assert ok_M, "Failed to get the Mass Matrix"

        h = idyn.FreeFloatingGeneralizedTorques(self.kindyn.getRobotModel())
        ok_h = self.kindyn.generalizedBiasForces(h)
        assert ok_h, "Failed to get the Bias forces"

        # ========================
        # Convert to numpy objects
        # ========================

        # Extract only the joint-related components and discard the base elements
        h_j_np = h.jointTorques().toNumPy()
        M_j_np = M.toNumPy()[6:, 6:]

        # ===============
        # Get the torques
        # ===============

        # Compute the errors
        s_tilde = joint_positions - self._references.position
        s_dot_tilde = joint_velocities - self._references.velocity

        assert self.kp.shape == s_tilde.shape, "Wrong shape of kp gains"
        assert self.kd.shape == s_dot_tilde.shape, "Wrong shape of kd gains"

        # Apply the gains
        s_ddot_desired = self._references.acceleration
        s_ddot_star = s_ddot_desired - (self.kp * s_tilde + self.kd * s_dot_tilde)

        # Compute the torques
        tau_np = M_j_np @ s_ddot_star + h_j_np

        # Clip to the maximum torque allowed
        if self._clip_torques:
            for idx, joint_name in enumerate(self.controlled_joints):
                print("Clipped torques")
                tau_np[idx] = \
                    np.min([tau_np[idx], self.robot.joint_torque_limits(joint_name)])

        return tau_np

    def terminate(self) -> bool:
        for joint_name, initial_mode in self._initial_control_mode.items():
            ok_mode = self.robot.set_joint_control_mode(joint_name, initial_mode)
            assert ok_mode, \
                f"Failed to restore initial control mode of joint '{joint_name}'"

        return True
