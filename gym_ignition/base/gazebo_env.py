# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
from gym_ignition import base
from gym_ignition.utils import logger
from gym_ignition.utils.typing import *
from gym_ignition import gympp_bindings as bindings


class GazeboEnv(gym.Wrapper):

    def __init__(self,
                 task_cls: type,
                 robot_cls : type,
                 sdf: str,
                 world: str,
                 rtf: float,
                 agent_rate: float,
                 physics_rate: float,
                 **kwargs):

        # Save the keyworded arguments.
        # We use them to build the task and the robot objects, and allow custom class
        # to accept user-defined parameters.
        self._kwargs = kwargs

        # Store the type of the class that provides RobotABC interface
        self._robot_cls = robot_cls

        # SDF files
        self._sdf = sdf
        self._world = world

        # Gazebo simulation attributes
        self._rtf = rtf
        self._agent_rate = agent_rate
        self._physics_rate = physics_rate
        self._gazebo_wrapper = None

        # Build the environment
        env = task_cls(**kwargs)
        assert isinstance(env, base.task.Task), \
            "'task_cls' object must inherit from Task"
        assert isinstance(env, gym.Env), "'task_cls' object must inherit from gym.Env"

        # Wrap the environment with this class
        super().__init__(env=env)

        # Seed the environment
        self.seed()

    def __getattr__(self, name):
        # We need to override this method because gym.Wrapper has a custom implementation
        # that forwards all the asked attributes to the wrapped class.
        # Due to this reason, regular wrappers cannot have new public methods and
        # attributes. This is a workaround that requires specifying all the new ones in
        # the list below.
        #
        # This fix is needed after https://github.com/openai/gym/issues/1554

        exposed_public_attributes = ["gazebo"]

        if name in exposed_public_attributes:
            return self.__getattribute__(name)
        else:
            return getattr(self.env, name)

    # ==========
    # PROPERTIES
    # ==========

    @property
    def gazebo(self) -> bindings.GazeboWrapper:
        if self._gazebo_wrapper:
            assert self._gazebo_wrapper.getPhysicsData().rtf == self._rtf, \
                "The RTF of gazebo does not match the configuration"
            assert 1 / self._gazebo_wrapper.getPhysicsData().maxStepSize == self._physics_rate, \
                "The physics rate of gazebo does not match the configuration"

            return self._gazebo_wrapper

        # =================
        # INITIALIZE GAZEBO
        # =================

        assert self._rtf, "Real Time Factor was not set"
        assert self._agent_rate, "Agent rate was not set"
        assert self._physics_rate, "Physics rate was not set"

        logger.debug("Starting gazebo")
        logger.debug("Agent rate: {} Hz".format(self._agent_rate))
        logger.debug("Physics rate: {} Hz".format(self._physics_rate))
        logger.debug("Simulation RTF: {}".format(self._rtf))

        # Compute the number of physics iteration to execute at every environment step
        num_of_iterations_per_gazebo_step = self._physics_rate / self._agent_rate

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

        # Initialize the model
        model_ok = self._gazebo_wrapper.setupGazeboModel(self._sdf)
        assert model_ok, "Failed to initialize the gazebo model"

        # Initialize the robot controller plugin
        lib_name = "RobotController"
        class_name = "gympp::plugins::RobotController"
        wrapper_ok = self._gazebo_wrapper.setupIgnitionPlugin(lib_name, class_name)
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
        # TODO: robot_name arg is used only for the FactoryRobot implementation
        self.env.robot = self._robot_cls(robot_name=model_name, **self._kwargs)
        assert isinstance(self.env.robot, base.robot.robot_abc.RobotABC), \
            "'robot' object must inherit from RobotABC"

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
        assert reward is not None, "Failed to get the reward"

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
        # Seed the wrapped environment (task)
        seed = self.env.seed(seed)

        # Update the spaces of the wrapper
        self.action_space = self.env.action_space
        self.observation_space = self.env.observation_space

        return seed
