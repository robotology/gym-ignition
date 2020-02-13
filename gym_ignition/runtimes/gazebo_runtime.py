# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
from gym_ignition import base
from gym_ignition.utils import logger
from gym_ignition.base import runtime
from gym_ignition.base.robot import robot_abc
from gym_ignition import gympp_bindings as bindings
from gym_ignition.utils.typing import State, Action, Observation, SeedList


class GazeboRuntime(runtime.Runtime):
    metadata = {'render.modes': ['human']}

    def __init__(self,
                 task_cls: type,
                 robot_cls: type,
                 rtf: float,
                 agent_rate: float,
                 physics_rate: float,
                 model: str = None,
                 world: str = "DefaultEmptyWorld.world",
                 **kwargs):

        # Save the keyworded arguments.
        # We use them to build the task and the robot objects, and allow custom class
        # to accept user-defined parameters.
        self._kwargs = kwargs

        # Store the type of the class that provides RobotABC interface
        self._robot_cls = robot_cls

        # Delete and create a new robot every environment reset
        self._first_run = True

        # SDF files
        self._model = model
        self._world = world

        # Gazebo simulation attributes
        self._rtf = rtf
        self._physics_rate = physics_rate
        self._gazebo_wrapper = None

        # Create the Task object
        task = task_cls(agent_rate=agent_rate, **kwargs)

        assert isinstance(task, base.task.Task), \
            "'task_cls' object must inherit from Task"

        # Wrap the environment with this class
        super().__init__(task=task, agent_rate=agent_rate)

        # Initialize the simulator and the robot
        self.task.robot = self._get_robot()

        # Initialize the spaces
        self.task.action_space, self.task.observation_space = self.task.create_spaces()

        # Seed the environment
        self.seed()

    # ==========
    # PROPERTIES
    # ==========

    @property
    def gazebo(self) -> bindings.GazeboWrapper:
        if self._gazebo_wrapper:
            assert self._gazebo_wrapper.getPhysicsData().rtf == self._rtf, \
                "The RTF of gazebo does not match the configuration"
            assert 1 / self._gazebo_wrapper.getPhysicsData().maxStepSize == \
                self._physics_rate, \
                "The physics rate of gazebo does not match the configuration"

            return self._gazebo_wrapper

        # =================
        # INITIALIZE GAZEBO
        # =================

        assert self._rtf, "Real Time Factor was not set"
        assert self.agent_rate, "Agent rate was not set"
        assert self._physics_rate, "Physics rate was not set"

        logger.debug("Starting gazebo")
        logger.debug(f"Agent rate: {self.agent_rate} Hz")
        logger.debug(f"Physics rate: {self._physics_rate} Hz")
        logger.debug(f"Simulation RTF: {self._rtf}")

        # Compute the number of physics iteration to execute at every environment step
        num_of_iterations_per_gazebo_step = self._physics_rate / self.agent_rate

        if num_of_iterations_per_gazebo_step != int(num_of_iterations_per_gazebo_step):
            logger.warn("Rounding the number of iterations to {} from the nominal {}"
                        .format(int(num_of_iterations_per_gazebo_step),
                                num_of_iterations_per_gazebo_step))
        else:
            logger.debug("Setting {} iteration per simulator step"
                         .format(int(num_of_iterations_per_gazebo_step)))

        # Create the GazeboWrapper object
        self._gazebo_wrapper = bindings.GazeboWrapper(
            int(num_of_iterations_per_gazebo_step),
            self._rtf,
            self._physics_rate)

        # Set the verbosity
        logger.set_level(gym.logger.MIN_LEVEL)

        # Initialize the world
        world_ok = self._gazebo_wrapper.setupGazeboWorld(self._world)
        assert world_ok, "Failed to initialize the gazebo world"

        # Initialize the ignition gazebo wrapper
        gazebo_initialized = self._gazebo_wrapper.initialize()
        assert gazebo_initialized, "Failed to initialize ignition gazebo"

        return self._gazebo_wrapper

    def _get_robot(self):
        if not self.gazebo:
            raise Exception("Failed to instantiate the gazebo simulator")

        # Build the robot object
        logger.debug("Creating the robot object")
        robot = self._robot_cls(model_file=self._model,
                                gazebo=self.gazebo,
                                **self._kwargs)
        assert isinstance(robot, robot_abc.RobotABC), \
            "'robot' object must inherit from RobotABC"

        # Check the requested robot features
        if self.task.robot_features:
            self.task.robot_features.has_all_features(robot)

        if not robot.valid():
            raise Exception("The robot is not valid")

        return robot

    # =================
    # Runtime interface
    # =================

    def timestamp(self) -> float:
        return self.gazebo.getSimulatedTime()

    # ===============
    # gym.Env METHODS
    # ===============

    def step(self, action: Action) -> State:
        if not self.action_space.contains(action):
            logger.warn("The action does not belong to the action space")

        # Set the action
        ok_action = self.task.set_action(action)
        assert ok_action, "Failed to set the action"

        # Step the simulator
        ok_gazebo = self.gazebo.run()
        assert ok_gazebo, "Failed to step gazebo"

        # Get the observation
        observation = self.task.get_observation()

        if not self.observation_space.contains(observation):
            logger.warn("The observation does not belong to the observation space")

        # Get the reward
        # TODO: use the wrapper method?
        reward = self.task.get_reward()
        assert reward is not None, "Failed to get the reward"

        # Check termination
        done = self.task.is_done()

        return State((observation, reward, done, {}))

    def reset(self) -> Observation:
        # Initialize gazebo if not yet done
        gazebo = self.gazebo
        assert gazebo, "Gazebo object not valid"

        # Remove the robot and insert a new one
        if not self._first_run:
            logger.debug("Hard reset: deleting the robot")
            self.task.robot.delete_simulated_robot()

            # Execute a dummy step to process model removal
            self.gazebo.run()

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

        return Observation(observation)

    def render(self, mode: str = 'human', **kwargs) -> None:
        if mode == 'human':
            # Initialize gazebo if not yet done
            gazebo = self.gazebo
            assert gazebo, "Gazebo object not valid"

            gui_ok = gazebo.gui()
            assert gui_ok, "Failed to render the environment"
            return

        raise Exception("Render mode '{}' not supported".format(mode))

    def close(self) -> None:
        self.gazebo.close()

    def seed(self, seed: int = None) -> SeedList:
        # This method also seeds the spaces. To create them, the robot object is often
        # needed. Here we check that is has been created.
        assert self.task.has_robot(), "The robot has not yet been created"

        # Seed the wrapped environment (task)
        seed = self.env.seed(seed)

        # Update the spaces of the wrapper
        self.action_space = self.env.action_space
        self.observation_space = self.env.observation_space

        return seed
