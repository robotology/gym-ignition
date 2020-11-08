# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import abc
from typing import List
from enum import Enum, auto
import idyntree.bindings as idt
from gym_ignition.utils import resource_finder


class FrameVelocityRepresentation(Enum):

    MIXED_REPRESENTATION = auto()
    BODY_FIXED_REPRESENTATION = auto()
    INERTIAL_FIXED_REPRESENTATION = auto()

    def to_idyntree(self):

        if self.value == FrameVelocityRepresentation.MIXED_REPRESENTATION.value:
            return idt.MIXED_REPRESENTATION
        elif self.value == FrameVelocityRepresentation.BODY_FIXED_REPRESENTATION.value:
            return idt.BODY_FIXED_REPRESENTATION
        elif self.value == FrameVelocityRepresentation.INERTIAL_FIXED_REPRESENTATION.value:
            return idt.INERTIAL_FIXED_REPRESENTATION
        else:
            raise ValueError(self.value)


class iDynTreeHelpers(abc.ABC):

    @staticmethod
    def get_model_loader(model_file: str, considered_joints: List[str] = None):

        # Find the urdf file
        urdf_file = resource_finder.find_resource(file_name=model_file)

        # Get the file extension
        folder, model_file = os.path.split(urdf_file)
        model_name, extension = os.path.splitext(model_file)

        if extension == ".sdf":
            raise RuntimeError("SDF models are not currently supported by iDynTree")

        # Create the model loader
        mdl_loader = idt.ModelLoader()

        # Load the urdf model
        if considered_joints is None:
            ok_load = mdl_loader.loadModelFromFile(urdf_file)
        else:
            ok_load = mdl_loader.loadReducedModelFromFile(urdf_file, considered_joints)

        if not ok_load:
            raise RuntimeError(f"Failed to load model from file '{urdf_file}'")

        return mdl_loader

    @staticmethod
    def get_kindyncomputations(
            model_file: str,
            considered_joints: List[str] = None,
            velocity_representation: FrameVelocityRepresentation = None):

        # Get the model loader
        model_loader = iDynTreeHelpers.get_model_loader(model_file, considered_joints)

        # Create KinDynComputations and insert the model
        kindyn = idt.KinDynComputations()
        ok_load = kindyn.loadRobotModel(model_loader.model())

        if not ok_load:
            raise RuntimeError("Failed to load model")

        if velocity_representation is None:
            velocity_representation = FrameVelocityRepresentation.MIXED_REPRESENTATION

        # Configure the velocity representation
        velocity_representation_idyntree = velocity_representation.to_idyntree()
        ok_repr = kindyn.setFrameVelocityRepresentation(velocity_representation_idyntree)

        if not ok_repr:
            raise RuntimeError("Failed to set the velocity representation")

        return kindyn
