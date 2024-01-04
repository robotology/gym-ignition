# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
from typing import Callable, Dict, Optional, Union, cast

import gymnasium as gym
from gym_gz.randomizers.abc import PhysicsRandomizer, TaskRandomizer
from gym_gz.randomizers.physics import dart
from gym_gz.runtimes import gazebo_runtime
from gym_gz.utils import typing

MakeEnvCallable = Callable[[Optional[Dict]], gym.Env]


class GazeboEnvRandomizer(gym.Wrapper, TaskRandomizer, abc.ABC):
    """
    Base class to implement an environment randomizer for Gz sim.

    The randomizer is a :py:class:`gym.Wrapper` that extends the
    :py:meth:`gym.Env.reset` method. Objects that inherit from this class are used to
    setup the environment for the handled :py:class:`~gym_gz.base.task.Task`.

    In its simplest form, a randomizer populates the world with all the models that need
    to be part of the simulation. The task could then operate on them from a
    :py:class:`~scenario.core.Model` object.

    More complex environments may require to randomize one or more simulated entities.
    Concrete classes that implement a randomizer could use
    :py:class:`~gym_gz.randomizers.model.sdf.SDFRandomizer` to randomize the model
    and :py:class:`~gym_gz.randomizers.abc.PhysicsRandomizer` to randomize
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

    def __init__(
        self,
        env: Union[str, MakeEnvCallable],
        physics_randomizer: PhysicsRandomizer = dart.DART(),
        **kwargs,
    ):

        # Print the extra kwargs
        gym.logger.debug(f"GazeboEnvRandomizer: {dict(kwargs=kwargs)}")

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

    def reset(
        self, seed: int = None, options: Dict = {}, **kwargs
    ) -> typing.ResetReturn:

        # Reset the physics
        if self._physics_randomizer.physics_expired() and seed is None:

            # Get the random components of the task
            seed = self.env.unwrapped.task.seed
            np_random = self.env.task.np_random

            # Reset the runtime + task, creating a new Gazebo instance
            self.env.close()
            del self.env
            self.env = self._create_environment(self._env_option, **self._kwargs)

            # Restore the random components of the task
            assert self.env.unwrapped.task.seed == seed
            self.env.task.np_random = np_random

        # Mark the beginning of a new rollout
        self._physics_randomizer.increase_rollout_counter()

        # Reset the task through the TaskRandomizer
        self.randomize_task(
            task=self.env.unwrapped.task, gazebo=self.env.unwrapped.gazebo, **kwargs
        )

        ok_paused_run = self.env.unwrapped.gazebo.run(paused=True)

        if not ok_paused_run:
            raise RuntimeError("Failed to execute a paused Gazebo run")

        # Reset the Task
        return self.env.reset(seed=seed, options={})

    # ===============
    # Private methods
    # ===============

    def _create_environment(
        self, env: Union[str, MakeEnvCallable], **kwargs
    ) -> gazebo_runtime.GazeboRuntime:

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
    def _create_from_callable(make_env: MakeEnvCallable, **kwargs) -> gym.Env:

        env = make_env(**kwargs)
        return env

    @staticmethod
    def _create_from_id(env_id: str, **kwargs) -> gym.Env:

        env = gym.make(env_id, **kwargs)
        return env
