# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
from gym_ignition import base
from gym_ignition.utils import logger
from gym_ignition.utils.typing import *
from gym_ignition import gympp_bindings as bindings


class GazeboEnvironment(gym.Wrapper):
    def __init__(self,
                 task: type,
                 robot : type,
                 sdf: str,
                 world: str,
                 physics_rate: float,
                 agent_rate: float,
                 **kwargs):

        # Save the keyworded arguments.
        # We use them to build the task and the robot objects, and allow custom class
        # to accept user-defined parameters.
        self._kwargs = kwargs

        # Store the type of the class that provides gymppy.Robot interface
        self._robot_cls = robot

        # Gazebo private attributes
        self._sdf = sdf
        self._world = world
        self._agent_rate = agent_rate
        self._physics_rate = physics_rate
        self._gazebo_wrapper = None

        # Build the environment
        env = task(kwargs)
        assert isinstance(env, base.task.Task), "'task' object must inherit from " \
                                                "gymppy.Task"
        assert isinstance(env, gym.Env), "'task' object must inherit from gym.Env"

        # Wrap the environment with this class
        super().__init__(env=env)

        # Seed the environment
        self.seed()

    @property
    def unwrapped(self):
        # The task is not a complete gym.Env environment since task objects implement
        # the Task interface.
        # This wrapper implements the step method using Ignition Gazebo. For this reason,
        #  the unwrapped environment is the gym.Env interface provided by this wrapper.
        return self

    # ==========
    # PROPERTIES
    # ==========

    @property
    def physics_rate(self) -> float:
        return self._physics_rate

    @property
    def agent_rate(self) -> float:
        return self._agent_rate

    @physics_rate.setter
    def physics_rate(self, physics_rate: float) -> None:
        if self._gazebo_wrapper:
            raise Exception("Gazebo server has been already created. You should set "
                            "the update rate before its initialization.")

        self._physics_rate = float(physics_rate)

    @agent_rate.setter
    def agent_rate(self, agent_rate: float) -> None:
        if self._gazebo_wrapper:
            raise Exception("Gazebo server has been already created. You should set "
                            "the update rate before its initialization.")

        self._agent_rate = float(agent_rate)

    @property
    def gazebo(self) -> bindings.GazeboWrapper:
        if self._gazebo_wrapper:
            assert self._gazebo_wrapper.getUpdateRate() == self.physics_rate, \
                "Update rate modified after gazebo started"

            return self._gazebo_wrapper

        # =================
        # INITIALIZE GAZEBO
        # =================

        assert self.agent_rate, "Agent rate was not set"
        assert self.physics_rate, "Physics rate was not set"

        logger.debug("Starting gazebo with physics at {} Hz and agent at {} Hz ".format(
            self.physics_rate, self.agent_rate))

        # Create the GazeboWrapper object
        self._gazebo_wrapper = bindings.GazeboWrapper(self.physics_rate)

        # Set the verbosity
        logger.set_level(gym.logger.MIN_LEVEL)

        # Initialize the world
        world_ok = self._gazebo_wrapper.setupGazeboWorld(self._world)
        assert world_ok, "Failed to initialize the gazebo world"

        # Initialize the model
        model_ok = self._gazebo_wrapper.setupGazeboModel(self._sdf)
        assert model_ok, "Failed to initialize the gazebo model"

        # Initialize the robot controller plugin
        lib_name = "RobotController"
        class_name = "gympp::plugins::RobotController"
        wrapper_ok = self._gazebo_wrapper.setupIgnitionPlugin(lib_name, class_name,
                                                              self.agent_rate)
        assert wrapper_ok, "Failed to setup the ignition plugin"

        # Initialize the ignition gazebo wrapper
        gazebo_initialized = self._gazebo_wrapper.initialize()
        assert gazebo_initialized, "Failed to initialize ignition gazebo"

        # ==============================
        # INITIALIZE THE ROBOT INTERFACE
        # ==============================

        # Get the robot name
        model_names = self._gazebo_wrapper.getModelNames()
        assert len(model_names) == 1, "The environment has more than one model"
        model_name = model_names[0]

        # Build the robot object
        # TODO: robot_name arg is used only for the SingletonRobot implementation
        self.env.robot = self._robot_cls(robot_name=model_name, **self._kwargs)
        assert isinstance(self.env.robot, base.robot.Robot), \
            "'robot' object must inherit from gymppy.Robot"

        return self._gazebo_wrapper

    # ===============
    # gym.Env METHODS
    # ===============

    def step(self, action: Action) -> State:
        # Validate action and robot
        assert self.action_space.contains(action), \
            "%r (%s) invalid" % (action, type(action))

        # Set the action
        ok_action = self.env._set_action(action)
        assert ok_action, "Failed to set the action"

        # Step the simulator
        ok_gazebo = self.gazebo.run()
        assert ok_gazebo, "Failed to step gazebo"

        # Get the observation
        observation = self.env._get_observation()
        assert self.observation_space.contains(observation), \
            "%r (%s) invalid" % (observation, type(observation))

        # Get the reward
        # TODO: use the wrapper method?
        reward = self.env._get_reward()
        assert reward, "Failed to get the reward"

        # Check termination
        done = self.env._is_done()

        return State((observation, reward, done, {}))

    def reset(self) -> Observation:
        # Initialize gazebo if not yet done
        gazebo = self.gazebo
        assert gazebo, "Gazebo object not valid"

        # Reset the environment
        ok_reset = self.env._reset()
        assert ok_reset, "Failed to reset the task"

        # Get the observation
        observation = self.env._get_observation()
        assert self.observation_space.contains(observation), \
            "%r (%s) invalid" % (observation, type(observation))

        return Observation(observation)

    def render(self, mode: str = 'human', **kwargs) -> None:
        if mode == 'human':
            gui_ok = self.gazebo.gui()
            assert gui_ok, "Failed to render the environment"
            return

        raise Exception("Render mode '{}' not supported".format(mode))

    def close(self) -> None:
        self.gazebo.close()

    def seed(self, seed: int = None) -> SeedList:
        # Seed the wrapped environment (task)
        seed = self.env.seed(seed)

        # Update the spaces of the wrapper
        self.action_space = self.env.action_space
        self.observation_space = self.env.observation_space

        return seed
