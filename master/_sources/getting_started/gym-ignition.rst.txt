.. _getting_started_gym_ignition:

gym-ignition
************

The previous sections reported the usage of ScenarIO to perform rigid-body simulations of any kind.
The :py:mod:`gym_ignition` Python package provides boilerplate code that use ScenarIO to create environments
for reinforcement learning research compatible with OpenAI Gym.

Beyond the abstractions provided by ScenarIO, gym-ignition introduces the following:

- :py:class:`~gym_ignition.base.runtime.Runtime`: Base class to abstract the runtime of an environment.
  It provides the code that steps the simulator for simulated environments or deals with real-time execution for
  environments running on real robots.
  The implementation for Ignition Gazebo is :py:class:`~gym_ignition.runtimes.gazebo_runtime.GazeboRuntime`.

- :py:class:`~gym_ignition.base.task.Task`: Base class providing the structure of the decision-making logic.
  The code of the task must be independent from the runtime, and only the ScenarIO APIs should be used.
  The active runtime will then execute the task on either simulated or real worlds.

- :py:mod:`gym_ignition.randomizers`: Utilities to develop ``gym.Wrapper`` classes that randomize the environment
  every rollout.
  The implementation for Ignition Gazebo is :py:class:`~gym_ignition.randomizers.gazebo_env_randomizer.GazeboEnvRandomizer`.

- :py:mod:`gym_ignition.rbd`: Utilities commonly used in robotic environments, like inverse kinematics and rigid-body
  dynamics algorithms.
  Refer to :py:class:`~gym_ignition.rbd.idyntree.inverse_kinematics_nlp.InverseKinematicsNLP` and
  :py:class:`~gym_ignition.rbd.idyntree.kindyncomputations.KinDynComputations` for more details.

.. tip::

    If you want to learn more about ``iDynTree``, the two classes we mainly use are ``iDynTree::KinDynComputations`` (`docs <https://robotology.github.io/idyntree/master/classiDynTree_1_1KinDynComputations.html>`__) and ``iDynTree::InverseKinematics`` (`docs <https://robotology.github.io/idyntree/master/classiDynTree_1_1InverseKinematics.html>`__).

    The theory and notation is summarized in `Multibody dynamics notation <https://pure.tue.nl/ws/portalfiles/portal/139293126/A_Multibody_Dynamics_Notation_Revision_2_.pdf>`_.

You can find demo environments created with ``gym-ignition`` in the
`gym_ignition_environments <https://github.com/robotology/gym-ignition/blob/master/python/gym_ignition_environments>`_ folder.
These examples show how to structure a new standalone Python package containing the environment with your robots.

For example, taking the cartpole balancing problem with discrete actions,
the components you need to implement are the following:

- A model :py:class:`~gym_ignition_environments.models.cartpole.CartPole`
  (`model/cartpole.py <https://github.com/robotology/gym-ignition/blob/master/python/gym_ignition_environments/models/cartpole.py>`_)

- A task :py:class:`~gym_ignition_environments.tasks.cartpole_discrete_balancing.CartPoleDiscreteBalancing`
  (`cartpole_discrete_balancing.py <https://github.com/robotology/gym-ignition/blob/master/python/gym_ignition_environments/tasks/cartpole_discrete_balancing.py>`_)

- A randomizer :py:class:`~gym_ignition_environments.randomizers.cartpole.CartpoleEnvRandomizer`
  (`randomizers/cartpole.py <https://github.com/robotology/gym-ignition/blob/master/python/gym_ignition_environments/randomizers/cartpole.py>`_)

- Environment registration as done in `__init__.py <https://github.com/robotology/gym-ignition/blob/master/python/gym_ignition_environments/__init__.py>`_

With all these resources in place, you can run a random policy of the environment as shown in
`launch_cartpole.py <https://github.com/robotology/gym-ignition/blob/master/examples/python/launch_cartpole.py>`_.
