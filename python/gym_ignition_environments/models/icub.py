# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
from typing import List
from gym_ignition import scenario
from scenario import core as scenario_core
from gym_ignition.utils.scenario import get_unique_model_name


class ICubGazeboABC(scenario.model_wrapper.ModelWrapper,
                    abc.ABC):

    DOFS = 32
    NUM_LINKS = 39
    NUM_JOINTS = 32

    initial_positions = {
        # Left leg
        'l_knee': -1.05,
        'l_ankle_pitch': -0.57, 'l_ankle_roll': -0.024,
        'l_hip_pitch': 0.48, 'l_hip_roll': 0.023, 'l_hip_yaw': -0.005,
        # Left arm
        'l_elbow': 0.54,
        'l_wrist_pitch': 0.0, 'l_wrist_prosup': 0.0, 'l_wrist_yaw': 0.0,
        'l_shoulder_pitch': -0.159, 'l_shoulder_roll': 0.435, 'l_shoulder_yaw': 0.183,
        # Head
        'neck_pitch': 0.0, 'neck_roll': 0.0, 'neck_yaw': 0.0,
        # Right leg
        'r_knee': -1.05,
        'r_ankle_pitch': -0.57, 'r_ankle_roll': -0.024,
        'r_hip_pitch': 0.48, 'r_hip_roll': 0.023, 'r_hip_yaw': -0.005,
        # Right arm
        'r_elbow': 0.54,
        'r_wrist_pitch': 0.0, 'r_wrist_prosup': 0.0, 'r_wrist_yaw': 0.0,
        'r_shoulder_pitch': -0.159, 'r_shoulder_roll': 0.435, 'r_shoulder_yaw': 0.183,
        # Torso
        'torso_pitch': 0.1, 'torso_roll': 0.0, 'torso_yaw': 0.0,
    }

    def __init__(self,
                 world: scenario_core.World,
                 position: List[float],
                 orientation: List[float],
                 model_file: str = None):

        # Get a unique model name
        model_name = get_unique_model_name(world, "icub")

        # Initial pose
        initial_pose = scenario_core.Pose(position, orientation)

        # Get the model file (URDF or SDF) and allow to override it
        if model_file is None:
            model_file = self.get_model_file()

        # Insert the model
        ok_model = world.to_gazebo().insert_model(model_file,
                                                  initial_pose,
                                                  model_name)

        if not ok_model:
            raise RuntimeError("Failed to insert model")

        # Get the model
        model = world.get_model(model_name)

        # Initialize base class
        super().__init__(model=model)

        # Store the initial positions
        q0 = list(self.initial_positions.values())
        joint_names = list(self.initial_positions.keys())
        assert self.dofs() == len(q0) == len(joint_names)

        ok_q0 = self.to_gazebo().reset_joint_positions(q0, joint_names)
        assert ok_q0, "Failed to set initial position"


class ICubGazebo(ICubGazeboABC,
                 scenario.model_with_file.ModelWithFile):

    def __init__(self,
                 world: scenario_core.World,
                 position: List[float] = (0., 0., 0.572),
                 orientation: List[float] = (0, 0, 0, 1.0),
                 model_file: str = None):

        super().__init__(world=world,
                         position=position,
                         orientation=orientation,
                         model_file=model_file)

    @classmethod
    def get_model_file(cls) -> str:

        import gym_ignition_models
        return gym_ignition_models.get_model_file("iCubGazeboV2_5")


class ICubGazeboSimpleCollisions(ICubGazeboABC):

    def __init__(self,
                 world: scenario_core.World,
                 position: List[float] = (0., 0., 0.572),
                 orientation: List[float] = (0, 0, 0, 1.0),
                 model_file: str = None):

        super().__init__(world=world,
                         position=position,
                         orientation=orientation,
                         model_file=model_file)

    @classmethod
    def get_model_file(cls) -> str:

        import gym_ignition_models
        return gym_ignition_models.get_model_file("iCubGazeboSimpleCollisionsV2_5")
