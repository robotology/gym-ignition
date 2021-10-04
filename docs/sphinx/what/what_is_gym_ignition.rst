.. _what_is_gym_ignition:

What is gym-ignition?
=====================

**gym-ignition** is a framework to create **reproducible robotics environments** for reinforcement learning research.

It is based on the :ref:`ScenarIO <what_is_scenario>` project which provides the low-level APIs to interface with the Ignition Gazebo simulator.
By default, RL environments share a lot of boilerplate code, e.g. for initializing the simulator or structuring the classes
to expose the ``gym.Env`` interface.
Gym-ignition provides the :py:class:`~gym_ignition.base.task.Task` and :py:class:`~gym_ignition.base.runtime.Runtime`
abstractions that help you focusing on the development of the decision-making logic rather than engineering.
It includes :py:mod:`~gym_ignition.randomizers` to simplify the implementation of domain randomization
of models, physics, and tasks.
Gym-ignition also provides powerful dynamics algorithms compatible with both fixed-base and floating-based robots by
exploiting `iDynTree <https://github.com/robotology/idyntree/>`_ and exposing
high-level functionalities (:py:mod:`~gym_ignition.rbd.idyntree`).

Gym-ignition does not provide out-of-the-box environments ready to be used.
Rather, its aim is simplifying and streamlining their development.
Nonetheless, for illustrative purpose, it includes canonical examples in the
:py:mod:`gym_ignition_environments` package.
