# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import List
from scenario import core as scenario
from gym_ignition.utils.scenario import get_unique_model_name
from gym_ignition.scenario import model_wrapper, model_with_file


class Panda(model_wrapper.ModelWrapper,
            model_with_file.ModelWithFile):

    def __init__(self,
                 world: scenario.World,
                 position: List[float] = (0.0, 0.0, 0.0),
                 orientation: List[float] = (1.0, 0, 0, 0),
                 model_file: str = None):

        # Get a unique model name
        model_name = get_unique_model_name(world, "panda")

        # Initial pose
        initial_pose = scenario.Pose(position, orientation)

        # Get the default model description (URDF or SDF) allowing to pass a custom model
        if model_file is None:
            model_file = Panda.get_model_file()

        # Insert the model
        ok_model = world.to_gazebo().insert_model(model_file,
                                                  initial_pose,
                                                  model_name)

        if not ok_model:
            raise RuntimeError("Failed to insert model")

        # Get the model
        model = world.get_model(model_name)

        # Initial joint configuration
        model.to_gazebo().reset_joint_positions(
            [0, -0.785,0, -2.356, 0, 1.571, 0.785],
            [name for name in model.joint_names() if "panda_joint" in name])

        # From:
        # https://github.com/mkrizmancic/franka_gazebo/blob/master/config/default.yaml
        pid_gains_1000hz = {
            'panda_joint1': scenario.PID(50,    0,  20),
            'panda_joint2': scenario.PID(10000, 0, 500),
            'panda_joint3': scenario.PID(100,   0,  10),
            'panda_joint4': scenario.PID(1000,  0,  50),
            'panda_joint5': scenario.PID(100,   0,  10),
            'panda_joint6': scenario.PID(100,   0,  10),
            'panda_joint7': scenario.PID(10,  0.5, 0.1),
            'panda_finger_joint1': scenario.PID(100, 0, 50),
            'panda_finger_joint2': scenario.PID(100, 0, 50),
        }

        # Check that all joints have gains
        if not set(model.joint_names()) == set(pid_gains_1000hz.keys()):
            raise ValueError("The number of PIDs does not match the number of joints")

        # Set the PID gains
        for joint_name, pid in pid_gains_1000hz.items():

            if not model.get_joint(joint_name).set_pid(pid=pid):
                raise RuntimeError(f"Failed to set the PID of joint '{joint_name}'")

        # Set the default PID update period
        assert model.set_controller_period(1000.0)

        # Initialize base class
        super().__init__(model=model)

    @classmethod
    def get_model_file(cls) -> str:

        import gym_ignition_models
        return gym_ignition_models.get_model_file("panda")
