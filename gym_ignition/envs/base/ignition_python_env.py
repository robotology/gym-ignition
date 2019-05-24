# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
from gym_ignition.utils import logger
from gym_ignition.utils.typing import *
from gympp import RobotSingleton_get, GazeboWrapper, Robot


class IgnitionPythonEnv(gym.Env):
    """The main class to create Ignition environments in python

    This class implements the gym.Env class and provides interfacing with the Ignition
    Gazebo simulator. The users of this class need to implement the following methods:

    _get_model_sdf
    _get_world_sdf

    Furthermore, the remaining methods of the gym.Env class have to implemented with
    the logic of the environment.

    Attributes:
        agent_rate (float): The rate associated to a single env.step call in Hertz
        physics_rate (float): The rate of the gazebo simulation in Hertz
        robot_controller_rate: The rate of the joint controller in Hertz
        gazebo: The object that wraps the simulator
        robot: The object that allows interacting with the simulated robot
    """

    metadata = {'render.modes': ['human']}

    def __init__(self):
        # Private attributes
        self._robot = None
        self._gazebo_wrapper = None
        self._model_sdf = None
        self._world_sdf = None
        self._np_random = None
        self._iterations = None
        self._physics_rate = None
        self._robot_controller_rate = None

    @property
    def physics_rate(self) -> float:
        return self._physics_rate

    @property
    def agent_rate(self) -> float:
        return self._agent_rate

    @property
    def robot_controller_rate(self) -> float:
        return self._robot_controller_rate

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

    @robot_controller_rate.setter
    def robot_controller_rate(self, robot_controller_rate: float) -> None:
        self._robot_controller_rate = robot_controller_rate

        if self._robot:
            logger.debug("Updating the robot rate after initializing. Consider doing "
                         "this before.")
            self.robot.setdt(robot_controller_rate)

    @property
    def gazebo(self) -> GazeboWrapper:
        if self._gazebo_wrapper:
            assert self._gazebo_wrapper.getUpdateRate() == self.physics_rate, \
                "Update rate modified after gazebo started"

            return self._gazebo_wrapper

        assert self.agent_rate, "Agent rate was not set"
        assert self.physics_rate, "Physics rate was not set"

        logger.debug("Starting gazebo with physics at {} Hz and agent at {} Hz ".format(
            self.physics_rate, self.agent_rate))

        # Create the GazeboWrapper object
        self._gazebo_wrapper = GazeboWrapper(self.physics_rate)

        # Set the verbosity
        logger.set_level(gym.logger.MIN_LEVEL)

        # Configure the robot ignition plugin
        lib_name = "RobotController"
        class_name = "gympp::plugins::RobotController"

        # Get the model and the world from the implementation of this class
        self._model_sdf = self._get_model_sdf()
        self._world_sdf = self._get_world_sdf()

        # Initialize the world
        world_ok = self._gazebo_wrapper.setupGazeboWorld(self._world_sdf)
        assert world_ok, "Failed to initialize the gazebo world"

        # Initialize the model
        model_ok = self._gazebo_wrapper.setupGazeboModel(self._model_sdf)
        assert model_ok, "Failed to initialize the gazebo model"

        # Initialize the plugin
        wrapper_ok = self._gazebo_wrapper.setupIgnitionPlugin(lib_name, class_name,
                                                              self.agent_rate)
        assert wrapper_ok, "Failed to setup the ignition plugin"

        # Initialize the ignition gazebo wrapper
        gazebo_initialized = self._gazebo_wrapper.initialize() # TODO
        assert gazebo_initialized, "Failed to initialize ignition gazebo"

        return self._gazebo_wrapper

    @property
    def robot(self) -> Robot:
        if self._robot:
            assert self._robot.valid(), "The robot object is not valid"
            return self._robot

        # Get the robot name
        model_names = self.gazebo.getModelNames()
        assert len(model_names) == 1, "The environment has more than one model"
        model_name = model_names[0]

        # Get the pointer to the Robot object
        self._robot = RobotSingleton_get().getRobot(model_name)

        assert self._robot, "Failed to get the Robot object"
        assert self._robot.valid(), "The Robot object is not valid"

        if self.robot_controller_rate:
            ok_dt = self._robot.setdt(1 / self.robot_controller_rate)
            assert ok_dt, "Failed to set the robot period"

        # Return the robot object
        return self._robot

    @gazebo.setter
    def gazebo(self, gazebo: GazeboWrapper) -> None:
        raise Exception("This attribute is read-only")

    @robot.setter
    def robot(self, robot: Robot) -> None:
        raise Exception("This attribute is read-only")

    def render(self, mode: str = 'human') -> None:
        if mode == 'human':
            gui_ok = self.gazebo.gui()
            assert gui_ok, "Failed to render the environment"
            return

        raise Exception("Render mode '{}' not supported".format(mode))

    def close(self) -> None:
        self.gazebo.close()

    def seed(self, seed: int = None) -> SeedList:
        if not seed:
            seed = np.random.randint(2**32 - 1)

        # Seed numpy
        self._np_random = np.random
        self._np_random.seed(seed)

        # Seed the spaces
        self.action_space.seed(seed)
        self.observation_space.seed(seed)

        return SeedList([seed])

    def _get_model_sdf(self) -> str:
        """Get the name of the sdf file containing the gazebo model

        The sdf file for the model contains the <model> element that describes the robot.
        The file should be relative to an entry of the IGN_GAZEBO_RESOURCE_PATH
        environment variable.

        Returns:
            str: The sdf file name of the model

        """
        raise NotImplementedError

    def _get_world_sdf(self) -> str:
        """Get the name of the sdf file containing the gazebo world

        The sdf file for the world contains the <world> element that describes the world.
        The file should be relative to an entry of the IGN_GAZEBO_RESOURCE_PATH
        environment variable.

        Returns:
            str: The sdf file name of the world

        """
        raise NotImplementedError
