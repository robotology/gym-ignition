# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import time
import numpy as np
from gym_ignition import base, robots
from gym_ignition.base.robot import robot_abc
from gym_ignition.utils import logger, resource_finder
from gym_ignition.utils.typing import State, Action, Observation, SeedList


class PyBulletRuntime(base.runtime.Runtime):
    metadata = {'render.modes': ['human']}

    def __init__(self,
                 task_cls: type,
                 robot_cls: type,
                 rtf: float,
                 agent_rate: float,
                 physics_rate: float,
                 model: str = None,
                 world: str = "plane_implicit.urdf",
                 **kwargs):

        # Save the keyworded arguments.
        # We use them to build the task and the robot objects, and allow custom class
        # to accept user-defined parameters.
        self._kwargs = kwargs

        # Store the type of the class that provides Robot interface
        self._robot_cls = robot_cls

        # Marks the first execution
        self._first_run = True

        # URDF or SDF model files
        self._world = world
        self._model = model

        # Simulation attributes
        self._rtf = rtf
        self._now = None
        self._bias = 0.0
        self._timestamp = 0.0

        self._physics_rate = physics_rate

        self._plane_id = None
        self._pybullet = None
        self._render_enabled = False
        self._render_called = False

        # Calculate the rate between agent and physics rate
        physics_steps_per_agent_update = physics_rate / agent_rate
        self._num_of_physics_steps = int(physics_steps_per_agent_update)

        assert physics_rate >= agent_rate, "Agent cannot run faster than physics"

        # If there is an incompatibility between the agent and physics rates, round the
        # iterations and notify the user
        if physics_steps_per_agent_update != round(physics_steps_per_agent_update):
            self._num_of_physics_steps = round(physics_steps_per_agent_update)
            logger.warn("The real physics rate will be {} Hz".format(
                agent_rate * self._num_of_physics_steps))

        logger.debug(f"RTF = {rtf}")
        logger.debug(f"Agent rate = {agent_rate} Hz")
        logger.debug(f"Physics rate = {agent_rate * self._num_of_physics_steps} Hz")

        logger.debug("Initializing the Task")
        task = task_cls(agent_rate=agent_rate, **kwargs)

        assert isinstance(task, base.task.Task), \
            "'task_cls' object must inherit from Task"

        # Wrap the task with this runtime
        super().__init__(task=task, agent_rate=agent_rate)

        # Initialize the simulator and the robot
        self.task.robot = self._get_robot()

        # Initialize the spaces
        self.task.action_space, self.task.observation_space = self.task.create_spaces()

        # Seed the environment
        self.seed()

    # =======================
    # PyBulletRuntime METHODS
    # =======================

    @property
    def pybullet(self):
        if self._pybullet is not None:
            return self._pybullet

        logger.debug("Creating PyBullet simulator")

        if self._render_enabled:
            import pybullet
            from pybullet_utils import bullet_client
            self._pybullet = bullet_client.BulletClient(pybullet.GUI)
        else:
            # Connects to an existing instance or, if it fails, creates an headless
            # simulation (DIRECT)
            from pybullet_utils import bullet_client
            self._pybullet = bullet_client.BulletClient()

        assert self._pybullet, "Failed to create the bullet client"

        # Find the ground plane
        import pybullet_data
        resource_finder.add_path(pybullet_data.getDataPath())
        world_abs_path = resource_finder.find_resource(self._world)

        # Load the ground plane
        self._plane_id = self._load_model(world_abs_path)

        # Configure the physics engine
        self._pybullet.setGravity(0, 0, -9.81)
        self._pybullet.setPhysicsEngineParameter(numSolverIterations=10)

        # Configure the physics engine with a single big time step divided in multiple
        # substeps. As an alternative, we could use a single substep and step the
        # simulation multiple times.
        self._pybullet.setTimeStep(1.0 / self._physics_rate / self._num_of_physics_steps)
        self._pybullet.setPhysicsEngineParameter(numSubSteps=self._num_of_physics_steps)

        # Disable real-time update. We step the simulation when needed.
        self._pybullet.setRealTimeSimulation(0)

        logger.info("PyBullet Physic Engine Parameters:")
        logger.info(str(self._pybullet.getPhysicsEngineParameters()))

        step_time = 1.0 / self._physics_rate / self._rtf
        logger.info(f"Nominal step time: {step_time} seconds")

        logger.debug("PyBullet simulator created")
        return self._pybullet

    def _get_robot(self) -> robot_abc.RobotABC:
        if not self.pybullet:
            raise Exception("Failed to instantiate the pybullet simulator")

        if not issubclass(self._robot_cls, robots.pybullet_robot.PyBulletRobot):
            raise Exception("The 'robot_cls' must inherit from PyBulletRobot")

        # Build the robot object
        logger.debug("Creating the robot object")
        robot = self._robot_cls(plane_id=self._plane_id,
                                model_file=self._model,
                                p=self._pybullet,
                                **self._kwargs)

        if self.task.robot_features:
            self.task.robot_features.has_all_features(robot)

        if not robot.valid():
            raise Exception("The robot is not valid")

        logger.debug("Robot object created")
        return robot

    def _load_model(self, filename: str, **kwargs) -> int:
        logger.debug(f"Loading model {filename}")

        # Get the file extension
        extension = os.path.splitext(filename)[1][1:]

        if extension == "sdf":
            model_id = self._pybullet.loadSDF(filename, **kwargs)[0]
        else:
            import pybullet
            model_id = self._pybullet.loadURDF(
                filename,
                flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
                **kwargs)

        return model_id

    def _enforce_rtf(self):
        if self._now is not None:
            # Compute the actual elapsed time from last cycle
            elapsed = time.time() - self._now

            # Nominal timestep with the desired RTF
            expected_dt = 1.0 / self.agent_rate / self._rtf

            # Compute the amount of sleep time to match the desired RTF
            sleep_amount = expected_dt - elapsed - self._bias

            # Sleep if needed
            real_sleep = 0
            if sleep_amount > 0:
                now = time.time()
                time.sleep(sleep_amount)
                real_sleep = time.time() - now

            # We use a low-filtered bias to compensate delays due to code running outside
            # the step method
            self._bias = \
                0.01 * (real_sleep - np.max([sleep_amount, 0.0])) + \
                0.99 * self._bias

        # Update the time for the next cycle
        self._now = time.time()

    # =================
    # Runtime interface
    # =================

    def timestamp(self) -> float:
        return self._timestamp

    # ===============
    # gym.Env METHODS
    # ===============

    def step(self, action: Action) -> State:
        if not self.action_space.contains(action):
            logger.warn("The action does not belong to the action space")

        # Update the timestamp
        self._timestamp += 1.0 / self.agent_rate

        # Set the action
        ok_action = self.task.set_action(action)
        assert ok_action, "Failed to set the action"

        # Step the simulator
        self.pybullet.stepSimulation()

        # Get the observation
        observation = self.task.get_observation()

        if not self.observation_space.contains(observation):
            logger.warn("The observation does not belong to the observation space")

        # Get the reward
        reward = self.task.get_reward()
        assert reward is not None, "Failed to get the reward"

        # Check termination
        done = self.task.is_done()

        # Enforce the real-time factor
        self._enforce_rtf()

        return State((observation, reward, done, {}))

    def reset(self) -> Observation:
        # Initialize pybullet if not yet done
        p = self.pybullet
        assert p, "PyBullet object not valid"

        # Remove the robot and insert a new one
        if not self._first_run:
            logger.debug("Hard reset: deleting the robot")
            self.task.robot.delete_simulated_robot()

            # Gazebo needs a dummy step to process model removal.
            # This line unifies the behaviour of the simulators.
            p.stepSimulation()

            logger.debug("Hard reset: creating new robot")
            self.task.robot = self._get_robot()
        else:
            self._first_run = False

        # Reset the environment
        ok_reset = self.task.reset_task()
        assert ok_reset, "Failed to reset the task"

        # Get the observation
        observation = self.task.get_observation()

        if not self.observation_space.contains(observation):
            logger.warn("The observation does not belong to the observation space")

        # Reset the timestamp
        self._timestamp = 0.0

        return Observation(observation)

    def render(self, mode: str = 'human', **kwargs) -> None:
        if mode != "human":
            raise Exception(f"Render mode '{mode}' not yet supported")

        # If render has been already called once, and the simulator is ok, return
        if self._render_enabled and self._pybullet:
            return

        # Enable rendering
        self._render_enabled = True

        # If the simulator is already allocated (in DIRECT mode, headless), we have to
        # disconnect it, starting it in GUI mode, and recreate the robot object.
        if self._pybullet is not None:
            logger.warn(
                "The simulator was allocated in DIRECT mode before calling render. "
                "The simulation has been reset. All changes to the simulation state "
                "will be lost.")

            # Disconnect the simulator and deletes the robot object
            self.close()

            # Create a new simulator and robot object
            self.task._robot = self._get_robot()

    def close(self) -> None:
        # Disconnect the simulator
        self._pybullet = None

        # Delete the outdated robot object.
        # It was connected to the deleted simulator instance.
        if self.task._robot:
            self.task._robot = None

    def seed(self, seed: int = None) -> SeedList:
        # This method also seeds the spaces. To create them, the robot object is often
        # needed. Here we check that is has been created.
        assert self.task.has_robot(), "The robot has not yet been created"

        # Seed the wrapped environment (task)
        seed = self.env.seed(seed)

        # Update the spaces of the wrapper
        self.action_space = self.task.action_space
        self.observation_space = self.task.observation_space

        return seed
