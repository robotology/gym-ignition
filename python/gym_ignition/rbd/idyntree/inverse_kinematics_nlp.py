# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import numpy as np
from enum import Enum, auto
from gym_ignition import rbd
import idyntree.bindings as idt
from dataclasses import dataclass
from typing import List, Dict, Optional, Union


class TargetType(Enum):

    POSITION = auto()
    ROTATION = auto()
    POSE = auto()


@dataclass
class TransformTargetData:

    position: np.ndarray
    quaternion: np.ndarray


@dataclass
class TargetData:

    type: TargetType
    weight: float
    data: Union[np.ndarray, TransformTargetData]


@dataclass
class IKSolution:

    joint_configuration: np.ndarray
    base_position: np.ndarray = np.array([0.0, 0.0, 0.0])
    base_quaternion: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])


class RotationParametrization(Enum):

    QUATERNION = auto()
    ROLL_PITCH_YAW = auto()

    def to_idyntree(self):

        if self.value == RotationParametrization.QUATERNION.value:
            return idt.InverseKinematicsRotationParametrizationQuaternion

        elif self.value == RotationParametrization.ROLL_PITCH_YAW.value:
            return idt.InverseKinematicsRotationParametrizationRollPitchYaw

        else:
            raise ValueError(self.value)


class TargetResolutionMode(Enum):

    TARGET_AS_CONSTRAINT_FULL = auto()
    TARGET_AS_CONSTRAINT_NONE = auto()
    TARGET_AS_CONSTRAINT_POSITION = auto()
    TARGET_AS_CONSTRAINT_ROTATION = auto()

    def to_idyntree(self):

        if self.value == TargetResolutionMode.TARGET_AS_CONSTRAINT_FULL.value:
            return idt.InverseKinematicsTreatTargetAsConstraintFull

        elif self.value == TargetResolutionMode.TARGET_AS_CONSTRAINT_NONE.value:
            return idt.InverseKinematicsTreatTargetAsConstraintNone

        elif self.value == TargetResolutionMode.TARGET_AS_CONSTRAINT_POSITION.value:
            return idt.InverseKinematicsTreatTargetAsConstraintPositionOnly

        elif self.value == TargetResolutionMode.TARGET_AS_CONSTRAINT_ROTATION.value:
            return idt.InverseKinematicsTreatTargetAsConstraintRotationOnly

        else:
            raise ValueError(self.value)


class InverseKinematicsNLP:

    def __init__(self,
                 urdf_filename: str,
                 considered_joints: List[str] = None,
                 joint_serialization: List[str] = None) -> None:

        self._base_frame: Optional[str] = None
        self._urdf_filename: str = urdf_filename
        self._targets_data: Dict[str, TargetData] = dict()
        self._ik: Optional[idt.InverseKinematics] = None

        self._considered_joints: List[str] = considered_joints
        self._joint_serialization: List[str] = joint_serialization

    # ======================
    # INITIALIZATION METHODS
    # ======================

    def initialize(self,
                   rotation_parametrization: RotationParametrization =
                   RotationParametrization.ROLL_PITCH_YAW,
                   target_mode: TargetResolutionMode =
                   TargetResolutionMode.TARGET_AS_CONSTRAINT_NONE,
                   cost_tolerance: float = 1E-08,
                   constraints_tolerance: float = 1E-4,
                   max_iterations: int = 1000,
                   base_frame: str = None,
                   floating_base: bool = False,
                   verbosity: int = 1) -> None:

        # Create the IK object
        self._ik = idt.InverseKinematics()

        # Load the URDF model and get the model loader object.
        # We create the full model with all the joints specified in joint_serialization.
        model_loader: idt.ModelLoader = self._get_model_loader(
            urdf=self._urdf_filename,
            joint_serialization=self._joint_serialization)

        # Get the model
        model: idt.Model = model_loader.model()

        # If all joints are enabled, get the list of joint names in order to know the
        # serialization of the full IK solution
        if self._joint_serialization is None:
            self._joint_serialization = []

            for joint_idx in range(model.getNrOfJoints()):
                self._joint_serialization.append(model.getJointName(joint_idx))

        # Configure the considered joints for the optimization.
        # If not specified, use the serialization of the full solution.
        if self._considered_joints is None:
            self._considered_joints = self._joint_serialization

        # Set the model in the IK specifying the considered joints
        if not self._ik.setModel(model, self._considered_joints):
            raise RuntimeError("Failed to set the model in the IK object")

        # Configure IK
        self._ik.setVerbosity(verbosity)
        self._ik.setMaxIterations(max_iterations)
        self._ik.setCostTolerance(cost_tolerance)
        self._ik.setConstraintsTolerance(constraints_tolerance)
        self._ik.setDefaultTargetResolutionMode(target_mode.to_idyntree())
        self._ik.setRotationParametrization(rotation_parametrization.to_idyntree())

        # Optionally change the base frame
        if base_frame is not None:
            # Store the frame of the base link
            self._base_frame = base_frame

            # Change the base frame
            if not self._ik.setFloatingBaseOnFrameNamed(base_frame):
                raise RuntimeError("Failed to change floating base frame")
        else:
            self._base_frame = model.getLinkName(model.getDefaultBaseLink())

        if not floating_base:
            # Add a frame constraint for the base
            self.add_frame_transform_constraint(frame_name=self._base_frame,
                                                position=np.array([0.0, 0, 0]),
                                                quaternion=np.array([1.0, 0, 0, 0]))

    def set_current_robot_configuration(self,
                                        base_position: np.ndarray,
                                        base_quaternion: np.ndarray,
                                        joint_configuration: np.ndarray) -> None:

        if joint_configuration.size != len(self._joint_serialization):
            raise ValueError(joint_configuration)

        H = rbd.idyntree.numpy.FromNumPy.to_idyntree_transform(
            position=base_position,
            quaternion=base_quaternion)

        q = rbd.idyntree.numpy.FromNumPy.to_idyntree_dyn_vector(array=joint_configuration)

        if not self._ik.setCurrentRobotConfiguration(baseConfiguration=H,
                                                     jointConfiguration=q):
            raise RuntimeError("Failed to set the current robot configuration")

    def set_current_joint_configuration(self,
                                        joint_name: str,
                                        configuration: float) -> None:

        if joint_name not in self._joint_serialization:
            raise ValueError(joint_name)

        if not self._ik.setJointConfiguration(jointName=joint_name,
                                              jointConfiguration=configuration):
            raise RuntimeError(f"Failed to set the configuration of joint '{joint_name}'")

    # ==============
    # TARGET METHODS
    # ==============

    def add_target(self,
                   frame_name: str,
                   target_type: TargetType,
                   weight: float = 1.0,
                   as_constraint: bool = False) -> None:

        if target_type == TargetType.ROTATION:
            # Add the target
            ok_target = self._ik.addRotationTarget(frame_name,
                                                   idt.Rotation.Identity(),
                                                   weight)

            # Initialize the target data buffers
            self._targets_data[frame_name] = TargetData(type=TargetType.ROTATION,
                                                        weight=weight,
                                                        data=np.array([1.0, 0, 0, 0]))

        elif target_type == TargetType.POSITION:
            # Add the target
            ok_target = self._ik.addPositionTarget(frame_name,
                                                   idt.Position_Zero(),
                                                   weight)

            # Initialize the target data buffers
            self._targets_data[frame_name] = TargetData(type=TargetType.POSITION,
                                                        weight=weight,
                                                        data=np.array([0.0, 0, 0]))

        elif target_type == TargetType.POSE:
            # Add the target
            ok_target = self._ik.addTarget(frame_name,
                                           idt.Transform.Identity(),
                                           weight,
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
                constraint = idt.InverseKinematicsTreatTargetAsConstraintRotationOnly
            elif target_type == TargetType.POSITION:
                constraint = idt.InverseKinematicsTreatTargetAsConstraintPositionOnly
            else:
                assert target_type == TargetType.POSE
                constraint = idt.InverseKinematicsTreatTargetAsConstraintFull

            if not self._ik.setTargetResolutionMode(frame_name, constraint):
                raise RuntimeError(f"Failed to set target '{frame_name}' as constraint")

    def add_com_target(self,
                       weight: float = 1.0,
                       as_constraint: bool = False,
                       constraint_tolerance: float = 1E-8) -> None:

        # Add the target
        self._ik.setCOMTarget(desiredPosition=idt.Position_Zero(), weight=weight)

        # Configure it either as target or constraint
        self._ik.setCOMAsConstraint(asConstraint=as_constraint)
        self._ik.setCOMAsConstraintTolerance(tolerance=constraint_tolerance)

        # Initialize the target data buffers
        assert "com" not in self._targets_data.keys()
        self._targets_data["com"] = TargetData(type=TargetType.POSITION,
                                               weight=weight,
                                               data=np.array([0.0, 0, 0]))

    def update_position_target(self, target_name: str, position: np.ndarray) -> None:

        if target_name not in self.get_active_target_names(
                target_type=TargetType.POSITION):

            raise ValueError(f"Failed to find a position target '{target_name}'")

        # Create the iDynTree position
        p = rbd.idyntree.numpy.FromNumPy.to_idyntree_position(position=position)

        # Update the target inside IK
        if not self._ik.updateTarget(target_name, p):
            raise RuntimeError(f"Failed to update position of target '{target_name}'")

        # Get the configured weight
        weight = self._targets_data[target_name].weight

        # Update the target data
        self._targets_data[target_name] = TargetData(type=TargetType.POSITION,
                                                     weight=weight,
                                                     data=position)

    def update_rotation_target(self, target_name: str, quaternion: np.ndarray) -> None:

        if target_name not in self.get_active_target_names(
                target_type=TargetType.ROTATION):

            raise ValueError(f"Failed to find a rotation target '{target_name}'")

        # Create the iDynTree rotation matrix
        R = rbd.idyntree.numpy.FromNumPy.to_idyntree_rotation(quaternion=quaternion)

        # Update the target inside IK
        if not self._ik.updateRotationTarget(target_name, R):
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

        if target_name not in self.get_active_target_names(
                target_type=TargetType.POSE):

            raise ValueError(f"Failed to find a transform target '{target_name}'")

        # Create the iDynTree transform
        H = rbd.idyntree.numpy.FromNumPy.to_idyntree_transform(position=position,
                                                               quaternion=quaternion)

        # Update the target inside IK
        if not self._ik.updateTarget(target_name, H):
            raise RuntimeError(f"Failed to update transform of target '{target_name}'")

        # Get the configured weight
        weight = self._targets_data[target_name].weight

        # Create the transform target data
        transform_data = TransformTargetData(position=position, quaternion=quaternion)

        # Update the target data
        self._targets_data[target_name] = TargetData(type=TargetType.POSE,
                                                     weight=weight,
                                                     data=transform_data)

    def update_com_target(self,
                          position: np.ndarray) -> None:

        if not self._ik.isCOMTargetActive():
            raise RuntimeError("Constraint on CoM not active")

        # Create the iDynTree position
        p = rbd.idyntree.numpy.FromNumPy.to_idyntree_position(position=position)

        # Update the target inside IK
        self._ik.setCOMTarget(desiredPosition=p,
                              weight=self._targets_data["com"].weight)

        # Update the target data
        self._targets_data["com"] = TargetData(type=TargetType.POSITION,
                                               weight=self._targets_data["com"].weight,
                                               data=position)

    def deactivate_com_target(self) -> None:

        if not self._ik.isCOMTargetActive():
            raise RuntimeError("Constraint on CoM not active")

        self._ik.deactivateCOMTarget()

    # =============
    # FRAME METHODS
    # =============

    def add_frame_transform_constraint(self,
                                       frame_name: str,
                                       position: np.ndarray,
                                       quaternion: np.ndarray) -> None:

        # Create the transform
        H = rbd.idyntree.numpy.FromNumPy.to_idyntree_transform(position=position,
                                                               quaternion=quaternion)

        # Add the target
        if not self._ik.addFrameConstraint(frame_name, H):
            raise RuntimeError(f"Failed to add constraint on frame '{frame_name}'")

    def add_frame_position_constraint(self,
                                      frame_name: str,
                                      position: np.ndarray) -> None:

        # Create the position
        p = rbd.idyntree.numpy.FromNumPy.to_idyntree_position(position=position)

        # Add the constraint
        if not self._ik.addFramePositionConstraint(frame_name, p):
            raise RuntimeError(f"Failed to add constraint on frame '{frame_name}'")

    def add_frame_rotation_constraint(self,
                                      frame_name: str,
                                      quaternion: np.ndarray) -> None:

        # Create the position
        R = rbd.idyntree.numpy.FromNumPy.to_idyntree_rotation(quaternion=quaternion)

        # Add the constraint
        if not self._ik.addFrameRotationConstraint(frame_name, R):
            raise RuntimeError(f"Failed to add constraint on frame '{frame_name}'")

    def deactivate_frame_constraint(self, frame_name: str) -> None:

        if not self._ik.isFrameConstraintActive(frame_name):
            raise RuntimeError(f"Constraint on frame '{frame_name}' not active")

        if not self._ik.deactivateFrameConstraint(frame_name):
            raise RuntimeError(f"Failed to deactivate constraint on frame '{frame_name}'")

    # ===================
    # IK SOLUTION METHODS
    # ===================

    def solve(self) -> None:

        if not self._ik.solve():
            raise RuntimeError("Failed to solve IK")

        # Initialize next solver call
        self._warm_start_with_last_solution()

    def warm_start_from(self,
                        full_solution: IKSolution = None,
                        reduced_solution: IKSolution = None) -> None:

        if full_solution is None and reduced_solution is None or \
                full_solution is not None and reduced_solution is not None:
            raise ValueError("You have to specify either a full or a reduced solution")

        if reduced_solution is not None and\
                reduced_solution.joint_configuration.size != len(self._considered_joints):
            raise RuntimeError(
                "The joint configuration does not match the number of considered joints")

        if full_solution is not None and \
                full_solution.joint_configuration.size != len(self._joint_serialization):
            raise RuntimeError(
                "The joint configuration does not match the number of model joints")

        solution = reduced_solution if reduced_solution is not None else full_solution

        H = rbd.idyntree.numpy.FromNumPy.to_idyntree_transform(
            position=solution.base_position,
            quaternion=solution.base_quaternion)

        q = rbd.idyntree.numpy.FromNumPy.to_idyntree_dyn_vector(
            array=solution.joint_configuration)

        # Warm start the solver
        if not self._ik.setFullJointsInitialCondition(H, q):
            raise RuntimeError("Failed to warm start the IK solver")

    # =======
    # GETTERS
    # =======

    def get_base_frame(self) -> str:

        return self._base_frame

    def get_available_target_names(self) -> List[str]:

        # Get the reduced model
        model = self._ik.reducedModel()

        # Note that also frames (modeled as fake links) are available targets
        link_names = []

        for link_index in range(model.getNrOfLinks()):
            link_names.append(model.getLinkName(link_index))

        return link_names

    def get_active_target_names(self, target_type: TargetType = None) -> List[str]:

        if target_type is None:
            return list(self._targets_data.keys())

        else:
            return [name for name, value in self._targets_data.items()
                    if value.type == target_type ]

    def get_target_data(self, target_name: str) -> TargetData:

        return self._targets_data[target_name]

    def get_full_solution(self) -> IKSolution:

        # Initialize buffers
        base_transform = idt.Transform.Identity()
        joint_positions = idt.VectorDynSize(self._ik.fullModel().getNrOfJoints())
        assert len(joint_positions) == len(self._joint_serialization)

        # Get the solution
        self._ik.getFullJointsSolution(base_transform, joint_positions)

        # Convert to numpy objects
        joint_positions = joint_positions.toNumPy()
        base_position = base_transform.getPosition().toNumPy()
        base_quaternion = base_transform.getRotation().asQuaternion().toNumPy()

        return IKSolution(base_position=base_position,
                          base_quaternion=base_quaternion,
                          joint_configuration=joint_positions)

    def get_reduced_solution(self) -> IKSolution:

        # Initialize buffers
        base_transform = idt.Transform.Identity()
        joint_positions = idt.VectorDynSize(self._ik.reducedModel().getNrOfJoints())
        assert len(joint_positions) == len(self._considered_joints)

        # Get the solution
        self._ik.getReducedSolution(base_transform, joint_positions)

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
    def _get_model_loader(urdf: str,
                          joint_serialization: List[str] = None) \
            -> idt.ModelLoader:

        # Get the model loader
        model_loader = idt.ModelLoader()

        if not os.path.exists(urdf):
            raise FileNotFoundError(urdf)

        # Load the model
        if joint_serialization:
            ok_load = model_loader.loadReducedModelFromFile(urdf, joint_serialization)
        else:
            ok_load = model_loader.loadModelFromFile(urdf)

        if not ok_load:
            raise RuntimeError("Failed to load model")

        # Due to some SWIG internal, returning the model contained by the loader
        # does not work as expected
        return model_loader

    def _warm_start_with_last_solution(self) -> None:

        last_solution = self.get_full_solution()
        self.warm_start_from(full_solution=last_solution)
