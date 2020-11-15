# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym_ignition_models
from gym_ignition import base, utils
from gym_ignition.base import runtime
from gym_ignition.utils import logger
from gym_ignition.utils.typing import *
from scenario import gazebo as scenario


class GazeboRuntime(runtime.Runtime):
    """
    Implementation of :py:class:`~gym_ignition.base.runtime.Runtime` for the Ignition
    Gazebo simulator.

    Args:
        task_cls: The class of the handled task.
        agent_rate: The rate at which the environment is called.
        physics_rate: The rate of the physics engine.
        real_time_factor: The desired RTF of the simulation.
        physics_engine: *(optional)* The physics engine to use.
        world: *(optional)* The path to an SDF world file. The world should not contain
            any physics plugin.

    Note:
        Physics randomization is still experimental and it could change in the future.
        Physics is loaded only once, when the simulator starts. In order to change the
        physics, a new simulator should be created.
    """

    metadata = {'render.modes': ['human']}

    def __init__(self,
                 task_cls: type,
                 agent_rate: float,
                 physics_rate: float,
                 real_time_factor: float,
                 physics_engine = scenario.PhysicsEngine_dart,
                 world: str = None,
                 **kwargs):

        # Gazebo attributes
        self._gazebo = None
        self._physics_rate = physics_rate
        self._real_time_factor = real_time_factor

        # Store the desired physics engine
        self._physics_engine = physics_engine

        # World attributes
        self._world = None
        self._world_sdf = world
        self._world_name = None

        # Create the Task object
        task = task_cls(agent_rate=agent_rate, **kwargs)

        if not isinstance(task, base.task.Task):
            raise RuntimeError("The task is not compatible with the runtime")

        # Wrap the task with the runtime
        super().__init__(task=task, agent_rate=agent_rate)

        # Trigger the initialization of the simulator and the world
        _ = self.gazebo

        # Initialize the spaces
        self.action_space, self.observation_space = self.task.create_spaces()

        # Store the spaces also in the task
        self.task.action_space = self.action_space
        self.task.observation_space = self.observation_space

        # Seed the environment
        self.seed()

    # =================
    # Runtime interface
    # =================

    def timestamp(self) -> float:

        return self.world.time()

    # =================
    # gym.Env interface
    # =================

    def step(self, action: Action) -> State:

        if not self.action_space.contains(action):
            logger.warn("The action does not belong to the action space")

        # Set the action
        self.task.set_action(action)

        # Step the simulator
        ok_gazebo = self.gazebo.run()
        assert ok_gazebo, "Failed to step gazebo"

        # Get the observation
        observation = self.task.get_observation()
        assert isinstance(observation, np.ndarray)

        if not self.observation_space.contains(observation):
            logger.warn("The observation does not belong to the observation space")

        # Get the reward
        reward = self.task.get_reward()
        assert isinstance(reward, float), "Failed to get the reward"

        # Check termination
        done = self.task.is_done()

        # Get info
        info = self.task.get_info()

        return State((Observation(observation), Reward(reward), Done(done), Info(info)))

    def reset(self) -> Observation:

        # Reset the task
        self.task.reset_task()

        # Execute a paused step
        ok_run = self.gazebo.run(paused=True)

        if not ok_run:
            raise RuntimeError("Failed to run Gazebo")

        # Get the observation
        observation = self.task.get_observation()
        assert isinstance(observation, np.ndarray)

        if not self.observation_space.contains(observation):
            logger.warn("The observation does not belong to the observation space")

        return Observation(observation)

    def render(self, mode: str = 'human', **kwargs) -> None:

        if mode != 'human':
            raise ValueError(f"Render mode '{mode}' not supported")

        gui_ok = self.gazebo.gui()

        if not gui_ok:
            raise RuntimeError("Failed to render the environment")

        return

    def close(self) -> None:

        ok_close = self.gazebo.close()

        if not ok_close:
            raise RuntimeError("Failed to close Gazebo")

    def seed(self, seed: int = None) -> SeedList:

        # This method also seeds the spaces. To create them, the task could use the world.
        # Here we check that is has been created.
        if not self.task.has_world():
            raise RuntimeError("The world has never been created")

        # Seed the task (it will also seed the spaces)
        seed = self.task.seed_task(seed)

        return seed

    # ==============================
    # Properties and Private Methods
    # ==============================

    @property
    def gazebo(self) -> scenario.GazeboSimulator:

        if self._gazebo is not None:
            assert self._gazebo.initialized()
            return self._gazebo

        # Compute the number of physics iteration to execute at every environment step
        num_of_steps_per_run = self._physics_rate / self.agent_rate

        if num_of_steps_per_run != int(num_of_steps_per_run):
            logger.warn("Rounding the number of iterations to {} from the nominal {}"
                        .format(int(num_of_steps_per_run), num_of_steps_per_run))

        # Create the simulator
        gazebo = scenario.GazeboSimulator(1.0 / self._physics_rate,
                                          self._real_time_factor,
                                          int(num_of_steps_per_run))

        # Store the simulator
        self._gazebo = gazebo

        # Insert the world (it will initialize the simulator)
        _ = self.world
        assert self._gazebo.initialized()

        return self._gazebo

    @property
    def world(self) -> scenario.World:

        if self._world is not None:
            assert self.gazebo.initialized()
            return self._world

        if self._gazebo is None:
            raise RuntimeError("Gazebo has not yet been created")

        # Help type hinting
        self._gazebo: scenario.GazeboSimulator

        if self._gazebo.initialized():
            raise RuntimeError("Gazebo was already initialized, cannot insert world")

        if self._world_sdf is None:
            self._world_sdf = ""
            self._world_name = utils.scenario.get_unique_world_name("default")
        else:
            sdf_world_name = scenario.get_world_name_from_sdf(self._world_sdf)
            self._world_name = utils.scenario.get_unique_world_name(sdf_world_name)

        # Load the world
        ok_world = self._gazebo.insert_world_from_sdf(self._world_sdf, self._world_name)

        if not ok_world:
            raise RuntimeError("Failed to load SDF world")

        # Initialize the simulator
        ok_initialize = self._gazebo.initialize()

        if not ok_initialize:
            raise RuntimeError("Failed to initialize Gazebo")

        if not self._gazebo.initialized():
            raise RuntimeError("Gazebo was not initialized")

        # Get the world
        world = self._gazebo.get_world(self._world_name)

        assert self._world_sdf is not None
        assert self._world_name in self._gazebo.world_names()

        if self._world_sdf == "":

            # Insert the ground plane
            ok_ground = world.insert_model(
                gym_ignition_models.get_model_file("ground_plane"))

            if not ok_ground:
                raise RuntimeError("Failed to insert the ground plane")

        # Set the world in the task
        self.task.world = world

        # Select the physics engine
        world.set_physics_engine(engine=self._physics_engine)

        # Store the world
        self._world = world

        return self._world
