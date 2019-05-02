# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
from gym_ignition.utils import logger
from gympp import RobotSingleton_get, GazeboWrapper, Robot


class IgnitionPythonEnv(gym.Env):
    """The main class to create Ignition environments in python

    This class implements the gym.Env class and provides interfacing with the Ignition
    Gazebo simulator. The users of this class need to implement the following methods:

    _get_model_sdf
    _get_world_sdf

    Furthermore, the remaining methods of the gym.Env class have to implemented with
    the logic of the environment.

    This class exposes the following read/write attributes

    iterations:  The number of gazebo iterations executed every step
    update_rate: The rate of the gazebo simulation in Hertz

    and the following read/only attributes:

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
        self._iterations = 1
        self._update_rate = 100.0

    @property
    def iterations(self) -> int:
        return self._iterations

    @property
    def update_rate(self) -> float:
        return self._update_rate

    @iterations.setter
    def iterations(self, iterations) -> None:
        if self._gazebo_wrapper:
            raise Exception("Gazebo server has been already created. You should set "
                            "the number of iteration before its initialization.")
        self._iterations = iterations

    @update_rate.setter
    def update_rate(self, update_rate) -> None:
        if self._gazebo_wrapper:
            raise Exception("Gazebo server has been already created. You should set "
                            "the update rate before its initialization.")
        self._iterations = update_rate

    @property
    def gazebo(self) -> GazeboWrapper:
        if not self._gazebo_wrapper:
            # Create the GazeboWrapper object
            self._gazebo_wrapper = GazeboWrapper(self.update_rate, self.iterations)

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
            wrapper_ok = self._gazebo_wrapper.setupIgnitionPlugin(lib_name, class_name)
            assert wrapper_ok, "Failed to setup the ignition plugin"

            # Initialize the ignition gazebo wrapper
            gazebo_initialized = self._gazebo_wrapper.initialize()
            assert gazebo_initialized, "Failed to initialize ignition gazebo"

        assert self._gazebo_wrapper.getNumberOfIterations() == self.iterations, \
            "Number of iterations modified after gazebo started"

        assert self._gazebo_wrapper.getUpdateRate() == self.update_rate, \
            "Update rate modified after gazebo started"

        return self._gazebo_wrapper

    @property
    def robot(self) -> Robot:
        if self._robot:
            return self._robot

        # Get the robot name
        model_names = self.gazebo.getModelNames()
        assert len(model_names) == 1, "The environment has more than one model"
        model_name = model_names[0]

        # Get the pointer to the Robot object
        self._robot = RobotSingleton_get().getRobot(model_name)

        assert self._robot, "Failed to get the Robot object"
        assert self._robot.valid(), "The Robot object is not valid"

        # Return the robot object
        return self._robot

    def render(self, mode: str = 'human') -> None:
        if mode == 'human':
            gui_ok = self.gazebo.gui()
            assert gui_ok, "Failed to render the environment"
            return

        raise Exception("Render mode '{}' not supported".format(mode))

    def close(self) -> None:
        self.gazebo.close()

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
