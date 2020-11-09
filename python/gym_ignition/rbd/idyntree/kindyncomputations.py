# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
from typing import List, Tuple
import idyntree.bindings as idt
from scenario import core as scenario_core
from gym_ignition.rbd import conversions
from gym_ignition.rbd.idyntree import numpy
from .helpers import FrameVelocityRepresentation, iDynTreeHelpers


class KinDynComputations:

    def __init__(self,
                 model_file: str,
                 considered_joints: List[str] = None,
                 world_gravity: np.ndarray = np.array([0, 0, -9.806]),
                 velocity_representation: FrameVelocityRepresentation = None) -> None:

        self.kindyn = iDynTreeHelpers.get_kindyncomputations(model_file,
                                                             considered_joints,
                                                             velocity_representation)

        self.world_gravity = np.array(world_gravity)
        self.dofs = self.kindyn.getNrOfDegreesOfFreedom()

        if considered_joints is None:

            model: idt.Model = self.kindyn.model()

            all_joints = [model.getJointName(i) for i in range(model.getNrOfJoints())]
            self._considered_joints = all_joints
        else:
            self._considered_joints = considered_joints

    def joint_serialization(self) -> List[str]:

        return self._considered_joints

    def set_robot_state(self,
                        s: np.ndarray,
                        ds: np.ndarray,
                        world_H_base: np.ndarray = np.eye(4),
                        base_velocity: np.ndarray = np.zeros(6),
                        world_gravity: np.ndarray = None) -> None:

        gravity = world_gravity if world_gravity is not None else self.world_gravity

        if s.size != self.dofs:
            raise ValueError(s)

        if ds.size != self.dofs:
            raise ValueError(ds)

        if gravity.size != 3:
            raise ValueError(gravity)

        if world_H_base.shape != (4, 4):
            raise ValueError(world_H_base)

        if base_velocity.size != 6:
            raise ValueError(base_velocity)

        s_idyntree = numpy.FromNumPy.to_idyntree_dyn_vector(array=s)
        ds_idyntree = numpy.FromNumPy.to_idyntree_dyn_vector(array=ds)

        world_gravity_idyntree = \
            numpy.FromNumPy.to_idyntree_fixed_vector(array=gravity)

        world_H_base_idyntree = numpy.FromNumPy.to_idyntree_transform(
            position=world_H_base[0:3, 3], rotation=world_H_base[0:3, 0:3])

        base_velocity_idyntree = numpy.FromNumPy.to_idyntree_twist(
            linear_velocity=base_velocity[0:3], angular_velocity=base_velocity[3:6])

        ok_state = self.kindyn.setRobotState(world_H_base_idyntree,
                                             s_idyntree,
                                             base_velocity_idyntree,
                                             ds_idyntree,
                                             world_gravity_idyntree)

        if not ok_state:
            raise RuntimeError("Failed to set the robot state")

    def set_robot_state_from_model(self,
                                   model: scenario_core.Model,
                                   world_gravity: np.ndarray = None) \
            -> None:

        s = np.array(model.joint_positions(self.joint_serialization()))
        ds = np.array(model.joint_velocities(self.joint_serialization()))

        world_o_base = np.array(model.base_position())
        world_quat_base = np.array(model.base_orientation())

        # Velocity representations
        body = FrameVelocityRepresentation.BODY_FIXED_REPRESENTATION.to_idyntree()
        mixed = FrameVelocityRepresentation.MIXED_REPRESENTATION.to_idyntree()

        if self.kindyn.getFrameVelocityRepresentation() is mixed:

            base_linear_velocity = np.array(model.base_world_linear_velocity())
            base_angular_velocity = np.array(model.base_world_angular_velocity())

        elif self.kindyn.getFrameVelocityRepresentation() is body:

            base_linear_velocity = np.array(model.base_body_linear_velocity())
            base_angular_velocity = np.array(model.base_body_angular_velocity())

        else:
            raise RuntimeError("INERTIAL_FIXED_REPRESENTATION not yet supported")

        # Pack the data structures
        world_H_base = conversions.Transform.from_position_and_quaternion(
            position=world_o_base, quaternion=world_quat_base)
        base_velocity_6d = np.concatenate((base_linear_velocity, base_angular_velocity))

        self.set_robot_state(s=s,
                             ds=ds,
                             world_H_base=world_H_base,
                             base_velocity=base_velocity_6d,
                             world_gravity=world_gravity)

    def get_floating_base(self) -> str:

        return self.kindyn.getFloatingBase()

    def get_joint_positions(self) -> np.ndarray:

        vector = idt.VectorDynSize()

        if not self.kindyn.getJointPos(vector):
            raise RuntimeError("Failed to extract joint positions")

        return vector.toNumPy()

    def get_joint_velocities(self) -> np.ndarray:

        vector = idt.VectorDynSize()

        if not self.kindyn.getJointVel(vector):
            raise RuntimeError("Failed to extract joint velocities")

        return vector.toNumPy()

    def get_model_velocity(self) -> np.ndarray:

        nu = idt.VectorDynSize()

        if not self.kindyn.getModelVel(nu):
            raise RuntimeError("Failed to get the model velocity")

        return nu.toNumPy()

    def get_model_position(self) -> np.ndarray:

        W_H_B: idt.Transform = self.kindyn.getWorldBaseTransform()
        rpy: idt.Vector3 = self.kindyn.getWorldBaseTransform().getRotation().asRPY()

        q_base = np.concatenate([W_H_B.getPosition().toNumPy(), rpy.toNumPy()])

        return np.concatenate([q_base, self.get_joint_positions()])

    def get_world_transform(self, frame_name: str) -> np.ndarray:

        if self.kindyn.getFrameIndex(frame_name) < 0:
            raise ValueError(f"Frame '{frame_name}' does not exist")

        H = self.kindyn.getWorldTransform(frame_name)

        return numpy.ToNumPy.from_idyntree_transform(transform=H)

    def get_relative_transform(self,
                               ref_frame_name: str,
                               frame_name: str) -> np.ndarray:

        if self.kindyn.getFrameIndex(ref_frame_name) < 0:
            raise ValueError(f"Frame '{ref_frame_name}' does not exist")

        if self.kindyn.getFrameIndex(frame_name) < 0:
            raise ValueError(f"Frame '{frame_name}' does not exist")

        ref_H_other: idt.Transform = self.kindyn.getRelativeTransform(ref_frame_name,
                                                                      frame_name)

        return ref_H_other.asHomogeneousTransform().toNumPy()

    def get_world_base_transform(self) -> np.ndarray:

        W_H_B: idt.Transform = self.kindyn.getWorldBaseTransform()
        return W_H_B.asHomogeneousTransform().toNumPy()

    def get_relative_transform_explicit(self,
                                        ref_frame_origin: str,
                                        ref_frame_orientation: str,
                                        frame_origin: str,
                                        frame_orientation: str) -> np.ndarray:

        for frame in {ref_frame_origin,
                      ref_frame_orientation,
                      frame_origin,
                      frame_orientation}:

            if frame != "world" and self.kindyn.getFrameIndex(frame) < 0:
                raise ValueError(f"Frame '{frame}' does not exist")

        # Compute the transform AB_H_CD
        frameA = ref_frame_origin
        frameB = ref_frame_orientation
        frameC = frame_origin
        frameD = frame_orientation

        if frame_orientation == "world":
            frameD = frameC

        if ref_frame_orientation == "world":
            frameB = frameA

        frameA_index = self.kindyn.getFrameIndex(frameName=frameA)
        frameB_index = self.kindyn.getFrameIndex(frameName=frameB)
        frameC_index = self.kindyn.getFrameIndex(frameName=frameC)
        frameD_index = self.kindyn.getFrameIndex(frameName=frameD)

        ref_H_other: idt.Transform = self.kindyn.getRelativeTransformExplicit(
            frameA_index,
            frameB_index,
            frameC_index,
            frameD_index)

        AB_H_CD = ref_H_other

        if frame_orientation == "world":
            AB_H_C = AB_H_CD
            C_H_CW: idt.Transform = self.kindyn.getWorldTransform(frameC).inverse()
            C_H_CW.setPosition(idt.Position(0, 0, 0))
            AB_H_CW = AB_H_C * C_H_CW
            AB_H_CD = AB_H_CW

        if ref_frame_orientation == "world":
            A_H_CD = AB_H_CD
            AW_H_A: idt.Transform = self.kindyn.getWorldTransform(frameA)
            AW_H_A.setPosition(idt.Position(0, 0, 0))
            AW_H_CD = AW_H_A * A_H_CD
            AB_H_CD = AW_H_CD

        return AB_H_CD.asHomogeneousTransform().toNumPy()

    def get_relative_adjoint_wrench_transform_explicit(self,
                                                       ref_frame_origin: str,
                                                       ref_frame_orientation: str,
                                                       frame_origin: str,
                                                       frame_orientation: str) \
            -> np.ndarray:

        AB_H_CD = self.get_relative_transform_explicit(
            ref_frame_origin=ref_frame_origin,
            ref_frame_orientation=ref_frame_orientation,
            frame_origin=frame_origin,
            frame_orientation=frame_orientation)

        return numpy.FromNumPy.to_idyntree_transform(
            position=AB_H_CD[0:3, 3], rotation=AB_H_CD[0:3, 0:3]) \
            .asAdjointTransformWrench().toNumPy()

    def get_mass_matrix(self) -> np.ndarray:

        M = idt.MatrixDynSize()

        if not self.kindyn.getFreeFloatingMassMatrix(M):
            raise RuntimeError("Failed to get the free floating mass matrix")

        return M.toNumPy()

    def get_bias_forces(self) -> np.ndarray:

        h = idt.FreeFloatingGeneralizedTorques(self.kindyn.model())
        ok_h = self.kindyn.generalizedBiasForces(h)

        if not ok_h:
            raise RuntimeError("Failed to get the generalized bias forces")

        base_wrench: idt.Wrench = h.baseWrench()
        joint_torques: idt.JointDOFsDoubleArray = h.jointTorques()

        return np.concatenate([base_wrench.toNumPy().flatten(),
                               joint_torques.toNumPy().flatten()])

    def get_momentum(self) -> Tuple[np.ndarray, np.ndarray]:

        spatial_momentum = self.kindyn.getLinearAngularMomentum()
        momentum_6d = spatial_momentum.asVector().toNumPy()

        linear, angular = np.split(momentum_6d, 2)
        return linear, angular

    def get_centroidal_momentum(self) -> Tuple[np.ndarray, np.ndarray]:

        spatial_momentum = self.kindyn.getCentroidalTotalMomentum()
        momentum_6d = spatial_momentum.asVector().toNumPy()

        linear, angular = np.split(momentum_6d, 2)
        return linear, angular

    def get_com_position(self) -> np.ndarray:

        W_p_com = self.kindyn.getCenterOfMassPosition()
        return W_p_com.toNumPy()

    def get_com_velocity(self) -> np.ndarray:

        # Velocity representations
        body = FrameVelocityRepresentation.BODY_FIXED_REPRESENTATION.to_idyntree()
        mixed = FrameVelocityRepresentation.MIXED_REPRESENTATION.to_idyntree()

        if self.kindyn.getFrameVelocityRepresentation() is mixed:

            # The method always returns the MIXED velocity of the CoM, regardless of
            # how KinDynComputations was configured.
            v_com = self.kindyn.getCenterOfMassVelocity()
            return v_com.toNumPy()

        elif self.kindyn.getFrameVelocityRepresentation() is body:

            # Get the transform of the base frame
            W_H_B = self.kindyn.getWorldBaseTransform()
            _, W_R_B = numpy.ToNumPy.from_idyntree_transform(transform=W_H_B, split=True)

            # Get the rotation between world and base frame
            B_R_W = np.linalg.inv(W_R_B)

            # Convert linear velocity from MIXED to BODY representation
            BW_v_com = self.kindyn.getCenterOfMassVelocity().toNumPy()
            B_v_com = B_R_W @ BW_v_com

            return B_v_com

        else:
            raise RuntimeError("INERTIAL_FIXED_REPRESENTATION not yet supported")

    def get_frame_jacobian(self, frame_name: str) -> np.ndarray:

        if self.kindyn.getFrameIndex(frame_name) < 0:
            raise ValueError(f"Frame '{frame_name}' does not exist")

        J = idt.MatrixDynSize(6, self.dofs + 6)

        if not self.kindyn.getFrameFreeFloatingJacobian(frame_name, J):
            raise RuntimeError("Failed to get the frame free floating jacobian")

        return J.toNumPy()

    def get_linear_angular_momentum_jacobian(self) -> np.ndarray:

        J_mom = idt.MatrixDynSize()

        if not self.kindyn.getLinearAngularMomentumJacobian(J_mom):
            raise RuntimeError("Failed to get the momentum jacobian")

        return J_mom.toNumPy()

    def get_centroidal_total_momentum_jacobian(self) -> np.ndarray:

        J_cmm = idt.MatrixDynSize()

        if not self.kindyn.getCentroidalTotalMomentumJacobian(J_cmm):
            raise RuntimeError("Failed to get the centroidal total momentum jacobian")

        return J_cmm.toNumPy()

    def get_frame_bias_acc(self, frame_name: str) -> np.ndarray:

        if self.kindyn.getFrameIndex(frame_name) < 0:
            raise ValueError(f"Frame '{frame_name}' does not exist")

        dJ_nu = self.kindyn.getFrameBiasAcc(frame_name)

        return dJ_nu.toNumPy()

    def get_com_bias_acc(self) -> np.ndarray:

        dJ_nu = self.kindyn.getCenterOfMassBiasAcc()
        return dJ_nu.toNumPy()
