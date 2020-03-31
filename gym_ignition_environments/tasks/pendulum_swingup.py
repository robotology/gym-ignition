# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym
import numpy as np
from typing import Tuple
from gym_ignition.base import task
from gym_ignition.utils import logger
from gym_ignition.utils.typing import Action, Observation, Reward
from gym_ignition.utils.typing import ActionSpace, ObservationSpace
from gym_ignition.base.robot import robot_abc, feature_detector, robot_joints


@feature_detector
class RobotFeatures(robot_abc.RobotABC,
                    robot_joints.RobotJoints,
                    abc.ABC):
    pass


class PendulumSwingUp(task.Task, abc.ABC):

    def __init__(self, agent_rate: float, **kwargs) -> None:

        # Initialize the Task base class
        super().__init__(agent_rate=agent_rate)

        # Store the requested robot features for this task
        self.robot_features = RobotFeatures

        # Limits
        self._max_speed = 8.0
        self._max_torque = 2.0

        # Private attributes
        self._last_a = None

    def create_spaces(self) -> Tuple[ActionSpace, ObservationSpace]:
        action_space = gym.spaces.Box(low=-self._max_torque, high=self._max_torque,
                                      shape=(1,), dtype=np.float32)

        high = np.array(
            [1.,  # cos(theta)
             1.,  # sin(theta)
             self._max_speed])
        observation_space = gym.spaces.Box(low=-high, high=high, dtype=np.float32)

        return action_space, observation_space

    def set_action(self, action: Action) -> bool:
        # Validate the action
        assert self.action_space.contains(action), \
            "%r (%s) invalid" % (action, type(action))

        # Store the last action. It is used to calculate the reward.
        self._last_a = action

        # Read the action and send the force to the cart
        force = action.tolist()[0]
        ok = self.robot.set_joint_force("pivot", force)

        if not ok:
            raise Exception("Failed to set the force to the pendulum")

        return True

    def get_observation(self) -> Observation:
        # Get the robot object
        robot = self.robot

        # Get the new pendulum position and velocity
        theta = robot.joint_position("pivot")
        theta_dot = robot.joint_velocity("pivot")

        # Create the observation object
        observation = Observation(np.array([np.cos(theta), np.sin(theta), theta_dot]))

        # Return the observation
        return observation

    def get_reward(self) -> Reward:
        # This environments is done only if the observation goes outside its limits.
        # Since it can happen only when velocity is too high, penalize this happening.
        if self.is_done():
            return Reward(-10000)

        # Get the data from the robot object
        theta = self.robot.joint_position("pivot")
        theta_dot = self.robot.joint_velocity("pivot")

        cost = \
            theta * theta + \
            0.1 * theta_dot * theta_dot +\
            0.001 * self._last_a

        return Reward(-cost)

    def is_done(self) -> bool:
        if not self.observation_space.contains(self.get_observation()):
            logger.warn("Observation is outside its space. Marking the episode as done.")
            return True

        # This environment is episodic and always reach the max_episode_steps
        return False

    def reset_task(self) -> bool:
        # Sample the angular velocity from the observation space
        _, _, theta_dot = self.observation_space.sample().tolist()

        # Sample the angular position from an uniform rng
        theta = self.np_random.uniform(0, 2 * np.pi)

        try:
            desired_control_mode = robot_joints.JointControlMode.TORQUE
            if self.robot.joint_control_mode("pivot") != desired_control_mode:
                ok_mode = self.robot.set_joint_control_mode("pivot", desired_control_mode)
                assert ok_mode, "Failed to set pendulum control mode"
        except Exception:
            logger.warn("Failed to set control mode. Is it supported by the runtime?")
            pass

        # Reset the robot state
        ok_reset = self.robot.reset_joint("pivot", theta, theta_dot)
        assert ok_reset, "Failed to reset the pendulum"

        # Clean the last applied force
        self._last_a = None

        return True
