# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym
from typing import cast
from gym_ignition import randomizers
from gym_ignition.utils import logger, typing
from gym_ignition.runtimes import gazebo_runtime
from typing import Callable, Dict, Optional, Union

MakeEnvCallable = Callable[[Optional[Dict]],gym.Env]


class GazeboEnvRandomizer(gym.Wrapper,
                          randomizers.abc.TaskRandomizer,
                          abc.ABC):
    """
    Base class to implement an environment randomizer for Ignition Gazebo.

    The randomizer is a :py:class:`gym.Wrapper` that extends the
    :py:meth:`gym.Env.reset` method. Objects that inherit from this class are used to
    setup the environment for the handled :py:class:`~gym_ignition.base.task.Task`.

    In its simplest form, a randomizer populates the world with all the models that need
    to be part of the simulation. The task could then operate on them from a
    :py:class:`~scenario.core.Model` object.

    More complex environments may require to randomize one or more simulated entities.
    Concrete classes that implement a randomizer could use
    :py:class:`~gym_ignition.randomizers.model.sdf.SDFRandomizer` to randomize the model
    and :py:class:`~gym_ignition.randomizers.abc.PhysicsRandomizer` to randomize
    the physics.

    Args:
        env: Defines the environment to handle. This argument could be either the string
            id if the environment does not need to be registered or a function that
            returns an environment object.
        physics_randomizer: Object that randomizes physics. The default physics engine is
            DART with no randomizations.

    Note:
        In order to randomize physics, the handled
        :py:class:`scenario.gazebo.GazeboSimulator` is destroyed and created again.
        This operation is demanding, consider randomizing physics at a low rate.
    """

    def __init__(self,
                 env: Union[str, MakeEnvCallable],
                 physics_randomizer: randomizers.abc.PhysicsRandomizer =
                 randomizers.physics.dart.DART(),
                 **kwargs):

        # Store the options
        self._env_option = env
        self._kwargs = dict(**kwargs, physics_engine=physics_randomizer.get_engine())

        # Create the environment
        env_to_wrap = self._create_environment(env=self._env_option, **self._kwargs)

        # Initialize the wrapper
        gym.Wrapper.__init__(self, env=env_to_wrap)

        # Store the physics randomizer
        self._physics_randomizer = physics_randomizer

    # ===============
    # gym.Env methods
    # ===============

    def reset(self, **kwargs) -> typing.Observation:

        # Reset the physics
        if self._physics_randomizer.physics_expired():

            # Get the random components of the task
            seed = self.env.task.seed
            np_random = self.env.task.np_random

            # Reset the runtime + task, creating a new Gazebo instance
            self.env.close()
            del self.env
            self.env = self._create_environment(self._env_option, **self._kwargs)

            # Restore the random components
            self.env.seed(seed=seed)
            assert self.env.task.seed == seed
            self.env.task.np_random = np_random

        # Mark the beginning of a new rollout
        self._physics_randomizer.increase_rollout_counter()

        # Reset the task through the TaskRandomizer
        self.randomize_task(task=self.env.task, gazebo=self.env.gazebo, **kwargs)

        ok_paused_run = self.env.gazebo.run(paused=True)

        if not ok_paused_run:
            raise RuntimeError("Failed to execute a paused Gazebo run")

        # Reset the Task
        return self.env.reset()

    # ===============
    # Private methods
    # ===============

    def _create_environment(self,
                            env: Union[str, MakeEnvCallable],
                            **kwargs) -> gazebo_runtime.GazeboRuntime:

        if isinstance(env, str):
            env_to_wrap = self._create_from_id(env_id=env, **kwargs)

        elif callable(env):
            env_to_wrap = self._create_from_callable(make_env=env, **kwargs)

        else:
            raise ValueError("The type of env object was not recognized")

        if not isinstance(env_to_wrap.unwrapped, gazebo_runtime.GazeboRuntime):
            raise ValueError("The environment to wrap is not a GazeboRuntime")

        return cast(gazebo_runtime.GazeboRuntime, env_to_wrap)

    @staticmethod
    def _create_from_callable(make_env: MakeEnvCallable,
                              **kwargs) -> gym.Env:

        with logger.gym_verbosity(level=gym.logger.WARN):
            env = make_env(**kwargs)

        return env

    @staticmethod
    def _create_from_id(env_id: str, **kwargs) -> gym.Env:

        with logger.gym_verbosity(level=gym.logger.WARN):
            env = gym.make(env_id, **kwargs)

        return env
