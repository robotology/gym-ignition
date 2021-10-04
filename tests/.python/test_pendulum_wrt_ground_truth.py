# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
import numpy as np
import pytest
from gym.envs import registry
from gym.envs.registration import register
from gym_ignition.robots.sim import gazebo, pybullet
from gym_ignition.tasks.pendulum_swingup import PendulumSwingUp
from gym_ignition.utils import logger
from gym_ignition.utils.typing import Observation, Reward, State

# Set verbosity
logger.set_level(gym.logger.DEBUG)


class PendulumEnv(gym.Env):
    """
    Environment that implements the pendulum dynamics from equations.
    It integrates the system with Euler.
    """

    metadata = {"render.modes": []}

    def __init__(self):
        super().__init__()

        # Check the xacro pendulum model
        r = 0.01
        self.L = 0.5
        self.m = 1
        self.g = 9.8182
        self.I = self.m * (4 * self.L * self.L + 3 * r * r) / 12

        self.dt = None
        # self.force = None
        self.theta = None
        self.theta_dot = None

    def set_state_from_obs(self, observation: np.ndarray) -> None:
        """
        Set the state of the environment from an observation sampled from another
        environment.
        """

        # Unpack the observation
        cos_theta, sin_theta, self.theta_dot = observation.tolist()

        # Calculate the initial angle
        self.theta = np.math.atan2(sin_theta, cos_theta)

    def step(self, action: np.ndarray) -> State:
        tau_m = action[0]
        theta_ddot = (
            self.m * self.g * self.L / 2.0 * np.sin(self.theta) + tau_m
        ) / self.I

        # Integrate with euler.
        # Note that in this way the ground truth used as comparison implements a very
        # basic integration method, and the errors might be reduced implementing a
        # better integrator.
        self.theta_dot = self.theta_dot + theta_ddot * self.dt
        self.theta = self.theta + self.theta_dot * self.dt

        observation = np.array([np.cos(self.theta), np.sin(self.theta), self.theta_dot])

        return State((Observation(observation), Reward(0.0), False, {}))

    def reset(self):
        # Use set_state_from_obs
        pass

    def render(self, mode="human", **kwargs):
        raise Exception("This runtime does not support rendering")

    def seed(self, seed=None):
        raise Exception("This runtime should not be seeded")


def theta_from_obs(observation: np.ndarray) -> float:
    cos_theta, sin_theta, _ = observation
    return np.math.atan2(sin_theta, cos_theta)


if "Pendulum-Ignition-PyTest-v0" not in [spec.id for spec in list(registry.all())]:
    register(
        id="Pendulum-Ignition-PyTest-v0",
        entry_point="gym_ignition.runtimes.gazebo_runtime:GazeboRuntime",
        max_episode_steps=1000,
        kwargs={
            "task_cls": PendulumSwingUp,
            "robot_cls": gazebo.pendulum.PendulumGazeboRobot,
            "model": "Pendulum/Pendulum.urdf",
            "world": "DefaultEmptyWorld.world",
            "rtf": 100,
            "agent_rate": 4000,
            "physics_rate": 4000,
            "hard_reset": False,
        },
    )

if "Pendulum-PyBullet-PyTest-v0" not in [spec.id for spec in list(registry.all())]:
    register(
        id="Pendulum-PyBullet-PyTest-v0",
        entry_point="gym_ignition.runtimes.pybullet_runtime:PyBulletRuntime",
        max_episode_steps=1000,
        kwargs={
            "task_cls": PendulumSwingUp,
            "robot_cls": pybullet.pendulum.PendulumPyBulletRobot,
            "model": "Pendulum/Pendulum.urdf",
            "world": "plane_implicit.urdf",
            "rtf": 100,
            "agent_rate": 4000,
            "physics_rate": 4000,  # To keep errors small, pybullet needs higher rate
            "hard_reset": False,
        },
    )


def template_pendulum_wrt_ground_truth(env_name: str, max_error_in_deg: float):
    # Create the pendulum
    env = gym.make(env_name)

    # Create the environment with the equations and initialize its time step
    env_equation = PendulumEnv()
    env_equation.dt = 1.0 / env.unwrapped.spec._kwargs["agent_rate"]

    # Render the environment
    # env.render('human')
    # time.sleep(5)

    # Seed the environment
    env.seed(42)

    for epoch in range(10):
        # Reset the environment
        logger.info("Resetting the environment")
        observation = env.reset()
        env_equation.set_state_from_obs(observation)

        # Initialize intermediate variables
        iteration = 0
        done = False

        while not done:
            iteration += 1

            # Sample a random action from the environment
            action = env.action_space.sample()

            # Step the environments
            observation, _, done, _ = env.step(action)
            observation_equation, _, _, _ = env_equation.step(action)

            theta = np.rad2deg(theta_from_obs(observation))
            theta_equation = np.rad2deg(theta_from_obs(observation_equation))

            # Compute the error taking care of the change of sign
            if np.sign(theta_equation) * np.sign(theta) == -1:
                error = (180 % abs(theta)) + (180 % abs(theta_equation))
            else:
                error = abs(theta - theta_equation)

            print(iteration, error)

            if error > max_error_in_deg:
                print("===================")
                print(f"Environment name: {env_name}")
                print(f"Iteration: #{iteration}")
                print(f"Error: {error}")
                print(f"Theta Equation    (deg): {theta_equation}")
                print(f"Theta Environment (deg): {theta}")
                print("===================")
                assert False, "Error in pendulum angle is bigger then the threshold"

    env.close()


@pytest.mark.parametrize(
    "env_name, max_error_in_deg",
    [
        ("Pendulum-Ignition-PyTest-v0", 3.0),
        ("Pendulum-PyBullet-PyTest-v0", 3.0),
    ],
)
def test_pendulum_ignition(env_name: str, max_error_in_deg: float):
    template_pendulum_wrt_ground_truth(env_name, max_error_in_deg)
