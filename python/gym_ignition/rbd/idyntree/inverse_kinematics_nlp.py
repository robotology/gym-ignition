# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import numpy as np
from enum import Enum, auto
from typing import List, Dict, Optional, Union, NamedTuple


class TargetType(Enum):
    POSITION = auto()
    ROTATION = auto()
    POSE = auto()


class TransformTargetData(NamedTuple):
    position: np.ndarray
    quaternion: np.ndarray


class TargetData(NamedTuple):
    type: TargetType
    weight: float
    data: Union[np.ndarray, TransformTargetData]


class IKSolution(NamedTuple):
    joint_configuration: np.ndarray
    base_position: np.ndarray = np.array([0.0, 0.0, 0.0])
    base_quaternion: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])


class RotationParametrization(Enum):
    QUATERNION = auto()
    ROLL_PITCH_YAW = auto()

    def to_idyntree(self):
        import iDynTree

        if self.value == RotationParametrization.QUATERNION.value:
            return iDynTree.InverseKinematicsRotationParametrizationQuaternion
        elif self.value == RotationParametrization.ROLL_PITCH_YAW.value:
            return iDynTree.InverseKinematicsRotationParametrizationRollPitchYaw
        else:
            raise ValueError(self.value)


class TargetResolutionMode(Enum):
    TARGET_AS_CONSTRAINT_FULL = auto()
    TARGET_AS_CONSTRAINT_NONE = auto()
    TARGET_AS_CONSTRAINT_POSITION = auto()
    TARGET_AS_CONSTRAINT_ROTATION = auto()

    def to_idyntree(self):
        import iDynTree

        if self.value == TargetResolutionMode.TARGET_AS_CONSTRAINT_FULL.value:
            return iDynTree.InverseKinematicsTreatTargetAsConstraintFull
        elif self.value == TargetResolutionMode.TARGET_AS_CONSTRAINT_NONE.value:
            return iDynTree.InverseKinematicsTreatTargetAsConstraintNone
        elif self.value == TargetResolutionMode.TARGET_AS_CONSTRAINT_POSITION.value:
            return iDynTree.InverseKinematicsTreatTargetAsConstraintPositionOnly
        elif self.value == TargetResolutionMode.TARGET_AS_CONSTRAINT_ROTATION.value:
            return iDynTree.InverseKinematicsTreatTargetAsConstraintRotationOnly
        else:
            raise ValueError(self.value)


class InverseKinematicsNLP:

    def __init__(self, urdf_filename: str, considered_joints: List[str] = None) -> None:

        import iDynTree

        self._base_frame: Optional[str] = None
        self._urdf_filename: str = urdf_filename
        self._targets_data: Dict[str, TargetData] = dict()
        self._ik: Optional[iDynTree.InverseKinematics] = None
        self._considered_joints: List[str] = considered_joints

    # ======================
    # INITIALIZATION METHODS
    # ======================

    def initialize(self,
                   rotation_parametrization: RotationParametrization =
                   RotationParametrization.ROLL_PITCH_YAW,
                   target_mode: TargetResolutionMode =
                   TargetResolutionMode.TARGET_AS_CONSTRAINT_NONE,
                   cost_tolerance: float = 1E-8,
                   max_iterations: int = 1000,
                   base_frame: str = None,
                   floating_base: bool = False,
                   verbosity: int = 1) -> None:

        # Lazy import iDynTree
        import iDynTree

        # Create the IK object
        self._ik = iDynTree.InverseKinematics()

        # Load the iDynTree model and get the loader
        model_loader: iDynTree.ModelLoader = self._get_model_loader(
            urdf=self._urdf_filename, considered_joints=self._considered_joints)

        # Get the model
        model = model_loader.model()

        # If all joints are enabled, get the list of joint names in order to know the
        # serialization of the IK solution
        if self._considered_joints is None:
            self._considered_joints = []

            for joint_idx in range(model.getNrOfJoints()):
                self._considered_joints.append(model.getJointName(joint_idx))

        # Set the model
        ok_model = self._ik.setModel(model)

        if not ok_model:
            raise RuntimeError("Failed to set the model in the IK object")

        # Configure IK
        self._ik.setVerbosity(verbosity)
        self._ik.setMaxIterations(max_iterations)
        self._ik.setCostTolerance(cost_tolerance)
        self._ik.setDefaultTargetResolutionMode(target_mode.to_idyntree())
        self._ik.setRotationParametrization(rotation_parametrization.to_idyntree())

        # Optionally change the base frame
        if base_frame is not None:
            # Store the frame of the base link
            self._base_frame = base_frame

            # Change the base frame
            ok_base = self._ik.setFloatingBaseOnFrameNamed(base_frame)

            if not ok_base:
                raise RuntimeError("Failed to change floating base frame")
        else:
            self._base_frame = model.getLinkName(model.getDefaultBaseLink())

        if not floating_base:
            # Add a frame constraint for the base
            self.add_frame_transform_constraint(frame_name=self._base_frame,
                                                position=np.array([0.0, 0, 0]),
                                                quaternion=np.array([1.0, 0, 0, 0]))

    # ==============
    # TARGET METHODS
    # ==============

    def add_target(self,
                   frame_name: str,
                   target_type: TargetType,
                   weight: float = 1.0,
                   as_constraint: bool = False) -> None:
        # Lazy import iDynTree
        import iDynTree as iDyn

        if target_type == TargetType.ROTATION:
            # Add the target
            ok_target = self._ik.addRotationTarget(frame_name,
                                                   iDyn.Rotation.Identity(),
                                                   weight)

            # Initialize the target data buffers
            self._targets_data[frame_name] = TargetData(type=TargetType.ROTATION,
                                                        weight=weight,
                                                        data=np.array([1.0, 0, 0, 0]))

        elif target_type == TargetType.POSITION:
            # Add the target
            ok_target = self._ik.addPositionTarget(frame_name,
                                                   iDyn.Position_Zero(),
                                                   weight)

            # Initialize the target data buffers
            self._targets_data[frame_name] = TargetData(type=TargetType.POSITION,
                                                        weight=weight,
                                                        data=np.array([0.0, 0, 0]))

        elif target_type == TargetType.POSE:
            # Add the target
            ok_target = self._ik.addTarget(frame_name,
                                           iDyn.Transform.Identity(),
                                           weight)

            # Create the transform target data
            target_data = TransformTargetData(position=np.array([0.0, 0, 0]),
                                              quaternion=np.array([1., 0, 0, 0]))

            # Initialize the target data buffers
            self._targets_data[frame_name] = TargetData(type=TargetType.POSE,
                                                        weight=weight,
                                                        data=target_data)

        else:
            raise ValueError(target_type)

        if not ok_target:
            raise RuntimeError(f"Failed to add target for frame '{frame_name}'")

        if as_constraint:
            if target_type == TargetType.ROTATION:
                constraint = iDyn.InverseKinematicsTreatTargetAsConstraintRotationOnly
            elif target_type == TargetType.POSITION:
                constraint = iDyn.InverseKinematicsTreatTargetAsConstraintPositionOnly
            else:
                assert target_type == TargetType.POSE
                constraint = iDyn.InverseKinematicsTreatTargetAsConstraintFull

            ok_constraint = self._ik.setTargetResolutionMode(frame_name, constraint)

            if not ok_constraint:
                raise RuntimeError(f"Failed to set target '{frame_name}' as constraint")

    def update_position_target(self, target_name: str, position: np.ndarray) -> None:
        if target_name not in self.get_active_target_names():
            raise ValueError(f"Target '{target_name}' was never added")

        # Create the iDynTree position
        p = self._to_idyntree_position(position=position)

        # Update the target inside IK
        ok_target = self._ik.updateTarget(target_name, p)

        if not ok_target:
            raise RuntimeError(f"Failed to update position of target '{target_name}'")

        # Get the configured weight
        weight = self._targets_data[target_name].weight

        # Update the target data
        self._targets_data[target_name] = TargetData(type=TargetType.ROTATION,
                                                     weight=weight,
                                                     data=position)

    def update_rotation_target(self, target_name: str, quaternion: np.ndarray) -> None:
        if target_name not in self.get_active_target_names():
            raise ValueError(f"Target '{target_name}' was never added")

        # Create the iDynTree rotation matrix
        R = self._to_idyntree_rotation(quaternion=quaternion)

        ok_target = self._ik.updateRotationTarget(target_name, R)

        if not ok_target:
            raise RuntimeError(f"Failed to update rotation of target '{target_name}'")

        # Get the configured weight
        weight = self._targets_data[target_name].weight

        # Update the target data
        self._targets_data[target_name] = TargetData(type=TargetType.ROTATION,
                                                     weight=weight,
                                                     data=quaternion)

    def update_transform_target(self,
                                target_name: str,
                                position: np.ndarray,
                                quaternion: np.ndarray) -> None:
        if target_name not in self.get_active_target_names():
            raise ValueError(f"Target '{target_name}' was never added")

        # Create the iDynTree transform
        H = self._to_idyntree_transform(position=position, quaternion=quaternion)

        # Update the target inside IK
        ok_target = self._ik.updateTarget(target_name, H)

        if not ok_target:
            raise RuntimeError(f"Failed to update transform of target '{target_name}'")

        # Get the configured weight
        weight = self._targets_data[target_name].weight

        # Create the transform target data
        transform_data = TransformTargetData(position=position, quaternion=quaternion)

        # Update the target data
        self._targets_data[target_name] = TargetData(type=TargetType.POSE,
                                                     weight=weight,
                                                     data=transform_data)

    # =============
    # FRAME METHODS
    # =============

    def add_frame_transform_constraint(self,
                                       frame_name: str,
                                       position: np.ndarray,
                                       quaternion: np.ndarray) -> None:
        # Create the transform
        transform = self._to_idyntree_transform(position=position, quaternion=quaternion)

        # Add the target
        ok_constraint = self._ik.addFrameConstraint(frame_name, transform)

        if not ok_constraint:
            raise RuntimeError(f"Failed to add constraint on frame '{frame_name}'")

    def add_frame_position_constraint(self,
                                      frame_name: str,
                                      position: np.ndarray) -> None:
        # Create the position
        position = self._to_idyntree_position(position=position)

        # Add the target
        ok_constraint = self._ik.addFramePositionConstraint(frame_name, position)

        if not ok_constraint:
            raise RuntimeError(f"Failed to add constraint on frame '{frame_name}'")

    def add_frame_rotation_constraint(self,
                                      frame_name: str,
                                      quaternion: np.ndarray) -> None:
        # Create the position
        rotation = self._to_idyntree_rotation(quaternion=quaternion)

        # Add the target
        ok_constraint = self._ik.addFrameRotationConstraint(frame_name, rotation)

        if not ok_constraint:
            raise RuntimeError(f"Failed to add constraint on frame '{frame_name}'")

    def deactivate_frame_constraint(self, frame_name: str) -> None:
        if not self._ik.isFrameConstraintActive(frame_name):
            raise RuntimeError(f"Constraint on frame '{frame_name}' not active")

        ok_removed = self._ik.deactivateFrameConstraint(frame_name)

        if not ok_removed:
            raise RuntimeError(f"Failed to deactivate constraint on frame '{frame_name}")

    # ===================
    # IK SOLUTION METHODS
    # ===================

    def solve(self) -> None:

        ok_solved = self._ik.solve()

        if not ok_solved:
            raise RuntimeError("Failed to solve IK")

        # Initialize next solver call
        self._warm_start_with_last_solution()

    def warm_start_from(self, warm_start_solution: IKSolution) -> None:

        if warm_start_solution.joint_configuration.size != len(self._considered_joints):
            raise RuntimeError(
                "The joint configuration does not match the number of considered joints")

        H = self._to_idyntree_transform(position=warm_start_solution.base_position,
                                        quaternion=warm_start_solution.base_quaternion)
        q = self._to_idyntree_dyn_vector(array=warm_start_solution.joint_configuration)

        # Warm start the solver
        ok_init = self._ik.setFullJointsInitialCondition(H, q)

        if not ok_init:
            raise RuntimeError("Failed to warm start the IK solver")

    # =======
    # GETTERS
    # =======

    def get_base_frame(self) -> str:
        return self._base_frame

    def get_available_target_names(self) -> List[str]:

        # Get the model
        model = self._ik.model()

        # Note that also frames (modeled as fake links) are available targets
        link_names = []

        for link_index in range(model.getNrOfLinks()):
            link_names.append(model.getLinkName(link_index))

        return link_names

    def get_active_target_names(self) -> List[str]:
        return list(self._targets_data.keys())

    def get_target_data(self, target_name: str) -> TargetData:
        return self._targets_data[target_name]

    def get_solution(self) -> IKSolution:

        # Lazy import iDynTree
        import iDynTree

        # Initialize buffers
        base_transform = iDynTree.Transform.Identity()
        joint_positions = iDynTree.VectorDynSize(self._ik.model().getNrOfJoints())

        # Get the solution
        self._ik.getFullJointsSolution(base_transform, joint_positions)

        # Convert to numpy objects
        joint_positions = joint_positions.toNumPy()
        base_position = base_transform.getPosition().toNumPy()
        base_quaternion = base_transform.getRotation().asQuaternion().toNumPy()

        return IKSolution(base_position=base_position,
                          base_quaternion=base_quaternion,
                          joint_configuration=joint_positions)

    # ===============
    # PRIVATE METHODS
    # ===============

    @staticmethod
    def _to_idyntree_dyn_vector(array: np.ndarray):
        import iDynTree
        dyn_vector = iDynTree.VectorDynSize()
        dyn_vector = dyn_vector.FromPython(array)

        return dyn_vector

    @staticmethod
    def _to_idyntree_position(position: np.ndarray):
        if position.size != 3:
            raise ValueError("The position must have 3 elements")

        import iDynTree
        p = iDynTree.Position()
        for i in range(3):
            p.setVal(i, position[i])

        return p

    @staticmethod
    def _to_idyntree_rotation(quaternion: np.ndarray):
        if quaternion.size != 4:
            raise ValueError("The quaternion must have 4 elements")

        import iDynTree
        quat = iDynTree.Vector4()
        quat = quat.FromPython(quaternion)

        R = iDynTree.Rotation()
        R.fromQuaternion(quat)

        return R

    @staticmethod
    def _to_idyntree_transform(position: np.ndarray, quaternion: np.ndarray):
        p = InverseKinematicsNLP._to_idyntree_position(position=position)
        R = InverseKinematicsNLP._to_idyntree_rotation(quaternion=quaternion)

        import iDynTree
        H = iDynTree.Transform()
        H.setPosition(p)
        H.setRotation(R)

        return H

    @staticmethod
    def _get_model_loader(urdf: str, considered_joints: List[str] = None):
        # Lazy import iDynTree
        import iDynTree

        # Get the model loader
        model_loader = iDynTree.ModelLoader()

        if not os.path.exists(urdf):
            raise FileNotFoundError(urdf)

        # Load the model
        if considered_joints:
            ok_load = model_loader.loadReducedModelFromFile(urdf, considered_joints)
        else:
            ok_load = model_loader.loadModelFromFile(urdf)

        if not ok_load:
            raise RuntimeError("Failed to load model")

        # Due to some SWIG internal, returning the model contained by the loader
        # does not work as expected
        return model_loader

    def _warm_start_with_last_solution(self) -> None:
        last_solution = self.get_solution()
        self.warm_start_from(last_solution)
